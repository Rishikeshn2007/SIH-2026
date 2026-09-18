"""
main.py -- SIH-26126 UGV Layer-1 perception.

    python main.py                     # use config (IP cam or fallback webcam)
    python main.py --source 0          # force webcam 0
    python main.py --source clip.mp4   # force a video file
    python main.py --source rtsp://... # force an IP stream
    python main.py --headless --frames 60   # no window, process 60 frames, save
    python main.py --drive             # ALSO stream motor cmds to the ESP8266

Controls (windowed mode):
    q = quit    p = pause/resume    s = save current view
    a = ARM / disarm the drive (only with --drive)   SPACE = E-STOP

SAFETY: without --drive this is perception only (commands are just logged). With
--drive it starts DISARMED; the car cannot move until you press 'a', and the ESP
stops the motors on its own if the command stream ever drops.
"""

from __future__ import annotations

import os
import sys
import time
import argparse
from datetime import datetime

import numpy as np
import cv2

import config as C
from vision.camera import CameraStream
from vision.fusion import fuse
from vision.traversability import TraversabilityAnalyzer
from vision.planner import Planner, VehicleCommand, STOP
from vision.visualization import Visualizer
from vision.utils import resolve_device, FpsMeter, RateLimiter, frame_quality
from vision.motor_mixer import mix


def parse_args():
    ap = argparse.ArgumentParser(description="UGV Layer-1 perception")
    ap.add_argument("--source", default=None,
                    help="override camera source (0, path, or URL)")
    ap.add_argument("--headless", action="store_true",
                    help="no GUI window; save frames instead")
    ap.add_argument("--frames", type=int, default=0,
                    help="process at most N frames then exit (0 = unlimited)")
    ap.add_argument("--no-yolo", action="store_true")
    ap.add_argument("--no-midas", action="store_true")
    ap.add_argument("--drive", action="store_true",
                    help="ENABLE motor commands to the ESP8266 (starts DISARMED; "
                         "press 'a' to arm, SPACE to e-stop)")
    return ap.parse_args()


def _fit(img, max_w: int):
    """Shrink an image to fit the screen width (display only). The 3-panel view
    is wider than many laptop screens, which silently cuts off the right-hand
    panel and the ARMED badge."""
    if not max_w or img.shape[1] <= max_w:
        return img
    s = max_w / float(img.shape[1])
    return cv2.resize(img, (int(img.shape[1] * s), int(img.shape[0] * s)),
                      interpolation=cv2.INTER_AREA)


def resolve_source(args):
    if args.source is not None:
        s = args.source
        return int(s) if s.isdigit() else s
    if C.CAMERA_URL:
        return C.CAMERA_URL
    if C.USE_FALLBACK_SOURCE:
        print("[main] CAMERA_URL is empty -> using FALLBACK_SOURCE "
              f"({C.FALLBACK_SOURCE!r}). Set CAMERA_URL in config.py for the "
              "real IP camera.")
        return C.FALLBACK_SOURCE
    print("\nERROR: No camera configured.\n"
          "  -> Set CAMERA_URL in config.py (e.g. 'http://<phone-ip>:8080/video'),\n"
          "     or set USE_FALLBACK_SOURCE = True to use a webcam,\n"
          "     or pass --source 0 / --source clip.mp4 / --source rtsp://...\n")
    sys.exit(2)


def build_models(args, device):
    """Construct MiDaS + YOLO with clear diagnostics on failure."""
    depth_est = yolo = None
    if not args.no_midas:
        try:
            from vision.depth_estimator import DepthEstimator
            depth_est = DepthEstimator(
                model_type=C.MIDAS_MODEL_TYPE, hub_repo=C.MIDAS_HUB_REPO,
                device=device, use_fp16=C.USE_FP16,
                custom_weights=C.MIDAS_CUSTOM_WEIGHTS)
        except Exception as e:
            print(f"\n[main] Could not initialise MiDaS: {e}\n"
                  "  - first run needs internet to download weights\n"
                  "  - install torch: see requirements.txt\n")
            sys.exit(3)
    if not args.no_yolo:
        try:
            from vision.yolo_detector import YoloDetector
            yolo = YoloDetector(
                model_path=C.YOLO_MODEL, device=device, use_fp16=C.USE_FP16,
                conf=C.YOLO_CONF, imgsz=C.YOLO_IMGSZ, max_det=C.YOLO_MAX_DET)
        except Exception as e:
            print(f"\n[main] Could not initialise YOLO: {e}\n"
                  "  - first run needs internet to download yolo11n-seg.pt\n"
                  "  - install ultralytics: see requirements.txt\n")
            sys.exit(3)
    return depth_est, yolo


def print_geometry(analyzer):
    """Print the camera pose the system is ASSUMING, every run.

    Every distance the perception layer produces is derived from CAMERA_HEIGHT_M
    and CAMERA_TILT_DEG. A wrong pose does not error out -- it silently reports
    obstacles metres away as centimetres away and turns the far wall of a room into
    a wall across the bumper. Printing it makes the assumption impossible to miss.
    """
    H = C.PROC_HEIGHT
    hz = analyzer.horizon_row(H)
    roi = int(round(H * C.ROI_TOP_FRAC))
    print(f"[geom] assuming lens height {C.CAMERA_HEIGHT_M:.2f} m, tilt "
          f"{C.CAMERA_TILT_DEG:.0f} deg down, vfov {C.CAMERA_VFOV_DEG:.0f} deg")
    print(f"[geom] predicted horizon row {hz:.0f} of {H}; ROI top row {roi} "
          f"(covers ~{analyzer.row_to_distance(max(roi, int(hz) + 1), H):.2f} m "
          f"to {analyzer.row_to_distance(H - 1, H):.2f} m)")
    if roi < hz:
        print("[geom] !! ROI starts ABOVE the horizon -- it will be clamped. "
              "Your tilt/height almost certainly do not match the real mount.")
    print("[geom] CHECK IT: the cyan dashed line on the traversability panel must "
          "sit where the floor meets the far wall. If it does not, fix "
          "CAMERA_TILT_DEG / CAMERA_HEIGHT_M before trusting any distance.")


def main():
    args = parse_args()
    device = resolve_device(C.DEVICE)
    print(f"[main] device = {device}")

    source = resolve_source(args)
    cam = CameraStream(source, name="cam",
                       reconnect_delay_s=C.CAMERA_RECONNECT_DELAY_S,
                       max_reconnect_tries=C.CAMERA_MAX_RECONNECT_TRIES,
                       read_timeout_ms=C.CAMERA_READ_TIMEOUT_MS).start()

    depth_est, yolo = build_models(args, device)
    analyzer = TraversabilityAnalyzer(C)
    planner = Planner(C)
    viz = Visualizer(C)
    print_geometry(analyzer)

    rl_yolo = RateLimiter(C.YOLO_FPS)
    rl_midas = RateLimiter(C.MIDAS_FPS)
    fps = FpsMeter(30)

    proc_w, proc_h = C.PROC_WIDTH, C.PROC_HEIGHT
    headless = args.headless or not C.SHOW_WINDOW
    os.makedirs(C.SAVE_DIR, exist_ok=True)

    # ---- Layer 2: optional drive link to the ESP8266 ----------------------
    # OFF unless --drive (or config SEND_TO_ESP8266). Always starts DISARMED,
    # so nothing moves until you press 'a'. The link runs its own thread that
    # streams the latest command and feeds the ESP failsafe.
    link = None
    if args.drive or C.SEND_TO_ESP8266:
        from vision.esp_link import EspLink
        link = EspLink(C.ESP_IP, C.ESP_PORT, send_hz=C.ESP_SEND_HZ,
                       max_speed_pct=C.DRIVE_MAX_SPEED_PCT, start_armed=False,
                       invert_left=C.DRIVE_INVERT_LEFT,
                       invert_right=C.DRIVE_INVERT_RIGHT).start()
        print("[main] DRIVE ENABLED -> ESP at "
              f"{C.ESP_IP}:{C.ESP_PORT}.  'a'=ARM  SPACE=E-STOP  q=quit")
        if headless:
            print("[main] WARNING: headless mode has no keyboard -> cannot ARM "
                  "or E-STOP; the car will stay stopped. Use a window to drive.")

    dets, depth, depth_color = [], None, None
    last_cmd_key = None
    paused = False
    slow_count = 0
    n_done = 0
    win = "UGV Layer-1 Perception"

    print("[main] running. q=quit p=pause s=save")
    try:
        while True:
            loop_t0 = time.time()

            if not paused:
                ok, frame = cam.read()
                if not ok or frame is None:
                    # no fresh frame -> STOP driving (don't roll on a stale cmd),
                    # show a status card, keep trying
                    if link is not None:
                        link.set_motors(0, 0)
                    card = np.zeros((240, 640, 3), np.uint8)
                    msg = cam.status.message
                    cv2.putText(card, "WAITING FOR CAMERA", (20, 90),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 215, 255), 2)
                    cv2.putText(card, msg[:60], (20, 140),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
                    if not headless:
                        cv2.imshow(win, card)
                        if (cv2.waitKey(30) & 0xFF) == ord('q'):
                            break
                    else:
                        time.sleep(0.05)
                    continue

                frame = cv2.resize(frame, (proc_w, proc_h))

                t_y0 = time.time()
                if yolo is not None and rl_yolo.ready():
                    dets = yolo.infer(frame)
                yolo_ms = yolo.last_infer_ms if yolo is not None else 0.0

                if depth_est is not None and rl_midas.ready():
                    depth = depth_est.infer(frame)
                    depth_color = depth_est.colorize(depth)
                midas_ms = depth_est.last_infer_ms if depth_est is not None else 0.0

                if depth is None:
                    # MiDaS hasn't produced a frame yet (or disabled): warm-up view
                    if depth_color is None:
                        depth_color = np.zeros_like(frame)
                    depth = np.zeros(frame.shape[:2], np.float32)

                fr = fuse(depth, dets, hard_stop_classes=C.YOLO_HARD_STOP_CLASSES)
                tr = analyzer.process(depth, fr.hard_stop_mask)
                cmd = planner.decide(tr)

                # ---- BLIND-CAMERA GUARD (safety) ----
                # A black / textureless frame makes MiDaS invent a smooth
                # gradient that looks like open floor -> the planner would say
                # FORWARD and the car would drive blind. Override to STOP.
                blind_reason = ""
                if getattr(C, "BLIND_CHECK", True):
                    ok_q, why = frame_quality(
                        frame,
                        dark_mean=getattr(C, "BLIND_DARK_MEAN", 12.0),
                        min_std=getattr(C, "BLIND_MIN_STD", 6.0))
                    if not ok_q:
                        blind_reason = why
                        cmd = VehicleCommand(STOP, 0, 0, f"CAMERA BLIND: {why}")

                # Layer 2: convert the command to skid-steer motor percentages
                # and hand them to the link (it streams + caps + arms). If the
                # link is DISARMED nothing moves regardless of what we send.
                if link is not None:
                    lft, rgt = mix(cmd.command, cmd.speed, cmd.steering,
                                   turn_speed=C.DRIVE_TURN_SPEED_PCT,
                                   steer_mix=C.DRIVE_STEER_MIX,
                                   pivot=C.DRIVE_PIVOT_TURNS)
                    link.set_motors(lft, rgt)

                key = (cmd.command, cmd.speed // 5, cmd.steering // 10)
                if key != last_cmd_key:
                    print("-" * 24 + "\n" + cmd.as_log())
                    last_cmd_key = key

                fps.tick()
                total_ms = (time.time() - loop_t0) * 1000.0
                stats = dict(fps=fps.fps(), yolo_ms=yolo_ms, midas_ms=midas_ms,
                             total_ms=total_ms, res=f"{proc_w}x{proc_h}",
                             device=device, paused=paused, blind=blind_reason)
                if link is not None:
                    ls = link.stats()
                    stats["armed"] = ls["armed"]
                    stats["drive"] = (f"{'ARMED' if ls['armed'] else 'DISARMED'} "
                                      f"L={ls['left']:+d} R={ls['right']:+d}")
                view = viz.render(frame, fr.detections, depth_color, tr, cmd, stats)

                # slow-FPS guard
                if fps.fps() and fps.fps() < C.MIN_ACCEPTABLE_FPS:
                    slow_count += 1
                    if slow_count == 15:
                        print(f"[perf] FPS {fps.fps():.1f} < "
                              f"{C.MIN_ACCEPTABLE_FPS}.")
                        if C.AUTO_REDUCE_ON_SLOW and proc_w > 384:
                            proc_w = max(384, int(proc_w * 0.8))
                            proc_h = max(224, int(proc_h * 0.8))
                            print(f"[perf] auto-reducing proc size to "
                                  f"{proc_w}x{proc_h}")
                        slow_count = 0
                else:
                    slow_count = 0

                n_done += 1
                if headless:
                    if args.frames and n_done >= args.frames:
                        out = os.path.join(C.SAVE_DIR, "headless_last.png")
                        cv2.imwrite(out, view)
                        print(f"[main] wrote {out}")
                        break
                    continue

                cv2.imshow(win, _fit(view, getattr(C, "MAX_WINDOW_WIDTH", 0)))

            k = cv2.waitKey(1) & 0xFF
            if k == ord('q'):
                break
            elif k == ord('p'):
                paused = not paused
                if paused and link is not None:
                    link.estop()   # never sit paused while armed
                print(f"[main] {'paused' if paused else 'resumed'}")
            elif k == ord('a'):
                if link is not None:
                    link.toggle_arm()
                else:
                    print("[main] not in drive mode (start with --drive)")
            elif k == ord(' '):
                if link is not None:
                    link.estop()
            elif k == ord('s'):
                fn = os.path.join(
                    C.SAVE_DIR, datetime.now().strftime("cap_%Y%m%d_%H%M%S.png"))
                cv2.imwrite(fn, view)
                print(f"[main] saved {fn}")

            if args.frames and n_done >= args.frames:
                break

    except KeyboardInterrupt:
        print("\n[main] interrupted")
    finally:
        if link is not None:
            link.stop()          # sends disarm + STOP, then closes
        cam.stop()
        if not headless:
            cv2.destroyAllWindows()
        if link is not None:
            print("[main] stopped. Motors were commanded to STOP and disarmed.")
        else:
            print("[main] stopped. (No motor commands were ever transmitted.)")


if __name__ == "__main__":
    main()
