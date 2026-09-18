"""
Central configuration for the SIH-26126 UGV Layer-1 perception system.

Everything you might want to tune lives here so you never have to dig through
the modules. Values are grouped by concern. Read the comments -- several of
these directly control the behaviour that was previously misbehaving
(far scenes flagged as obstacles, floor treated as a wall, etc.).

NOTE: This is a PERCEPTION-ONLY stage. No motor commands are transmitted.
      The planner only *logs* a simulated VehicleCommand.
"""

# =============================================================================
# 1. CAMERA / VIDEO SOURCE
# =============================================================================

# Your phone / IP camera stream URL.
#
#   * Leave it as "" to use the fallback source below (webcam / video file).
#   * Fill it in for the real robot, e.g. from an "IP Webcam" style app:
#       RTSP  :  "rtsp://192.168.1.50:554/h264_ulaw.sdp"
#       MJPEG :  "http://192.168.1.50:8080/video"
#       MJPEG :  "http://192.168.1.50:8080/videofeed"
#
# Do NOT commit a real URL to source control. There is intentionally no
# default URL -- if this is "" and USE_FALLBACK_SOURCE is False you get a
# clear error telling you to set it.
CAMERA_URL = "http://192.168.137.233:8080/videofeed"

# Fallback test source, used when CAMERA_URL is empty (or as `--source` on CLI).
#   * 0            -> default laptop webcam
#   * "clip.mp4"   -> a local video file
USE_FALLBACK_SOURCE = True
FALLBACK_SOURCE = 0

# Camera read behaviour
CAMERA_RECONNECT_DELAY_S = 2.0     # wait between reconnect attempts
CAMERA_MAX_RECONNECT_TRIES = 0     # 0 = retry forever
CAMERA_READ_TIMEOUT_MS = 5000      # FFMPEG open/read timeout for network streams

# =============================================================================
# 2. PROCESSING RESOLUTION
# =============================================================================
# Frames are resized to this before inference. Smaller = faster.
# The display is scaled independently (see visualization).
PROC_WIDTH = 640
PROC_HEIGHT = 384          # keep ~16:9-ish; must be divisible by 32 ideally
KEEP_ASPECT_ON_RESIZE = True

# =============================================================================
# 3. DEVICE / PERFORMANCE
# =============================================================================
DEVICE = "auto"            # "auto" | "cuda" | "cpu"
USE_FP16 = True            # half precision on CUDA (ignored on CPU)

# Independent inference rates. The camera/display loop runs as fast as it can;
# YOLO and MiDaS only re-run this many times per second and their last result
# is reused in between. This is the main real-time lever.
YOLO_FPS = 8.0
MIDAS_FPS = 8.0

# If measured end-to-end FPS drops below this for a sustained period, print a
# warning and (optionally) auto-reduce processing resolution.
MIN_ACCEPTABLE_FPS = 6.0
AUTO_REDUCE_ON_SLOW = True

# =============================================================================
# 4. YOLO (YOLO11n-seg)
# =============================================================================
YOLO_MODEL = "yolo11n-seg.pt"   # auto-downloaded by ultralytics on first run
YOLO_CONF = 0.35
YOLO_IMGSZ = 640
YOLO_MAX_DET = 50
# Classes that ALWAYS force a STOP if detected near / in the driving corridor.
# (COCO names.) These override the geometric decision for safety.
YOLO_HARD_STOP_CLASSES = ("person",)

# =============================================================================
# 5. MiDaS (relative / inverse depth)
# =============================================================================
# torch.hub model type. "MiDaS_small" is the right default for real-time laptop
# use. Alternatives: "DPT_Hybrid" (slower, better), "DPT_Large" (slowest).
MIDAS_MODEL_TYPE = "MiDaS_small"
MIDAS_HUB_REPO = "intel-isl/MiDaS"

# --- OPTIONAL: reuse your own MiDaS ---
# If you have your own working MiDaS weights/model, set this to a .pt path and
# depth_estimator.py will load it instead of torch.hub. Leave "" to use hub.
MIDAS_CUSTOM_WEIGHTS = ""

# IMPORTANT: MiDaS output is RELATIVE inverse depth (bigger = nearer), with an
# unknown per-frame scale. We NEVER treat it as metres. See traversability.py.

# =============================================================================
# 6. GROUND-PLANE TRAVERSABILITY  (the core of the fix)
# =============================================================================
# The old approach ("normalize depth, threshold near = obstacle") fails because
# MiDaS is per-frame relative: an open field gets stretched so the least-far
# patch looks 'near', and the floor right in front of the wheels always looks
# 'near'. Instead we MODEL the ground plane and flag DEVIATIONS from it.

# Region of interest: only the lower part of the frame is the ground the robot
# will actually drive into. Everything above ROI_TOP_FRAC is ignored for
# driving decisions (it's horizon / far walls / ceiling -- irrelevant to a
# small robot that cares about the next ~2 m).
#
# THIS VALUE CONTROLS HOW FAR THE ROBOT CAN SEE. It used to be 0.45, which was
# too aggressive and is why plain walls were missed: for a camera at 15 cm
# tilted 20 deg down, row 0.45*H corresponds to only ~0.49 m of ground, so the
# BASE of any wall further than ~0.4 m fell ABOVE the ROI and was never even
# examined. The robot was literally not looking at the wall.
#
# 0.22 reaches ~1.5-2 m while still sitting below the horizon (~row 0.15*H for
# this geometry), so walls at 0.5-1.5 m are inside the ROI where the
# vertical-surface test can see them. Do NOT raise it above the horizon: sky
# and far walls carry no ground-plane information and only add noise.
# Run  python tools/roi_check.py  to print the ROI's real ground coverage for
# your camera height and tilt.
ROI_TOP_FRAC = 0.22        # 0.0=top, 1.0=bottom. Start of ROI as fraction of H.

# Number of vertical column bins the ROI is split into for free-space scanning.
NUM_COLUMNS = 15

# The bottom REF_STRIP_FRAC of the ROI is ASSUMED to be drivable ground right
# in front of the wheels, and is used to fit the floor's depth trend.
REF_STRIP_FRAC = 0.18

# Depth smoothing
DEPTH_BLUR_KSIZE = 5       # spatial gaussian blur kernel (odd). 0 = off.
DEPTH_TEMPORAL_ALPHA = 0.5 # EMA across frames: new = a*cur + (1-a)*prev. 1=off.

# Obstacle = actual inverse-depth exceeds the fitted floor trend by this margin.
# Units are fractions of the ROI's inverse-depth spread (scene-independent), so
# this stays meaningful whether the scene is a corridor or an open field.
RESIDUAL_MARGIN_FRAC = 0.18
MIN_OBSTACLE_RUN = 3       # consecutive rows above margin to confirm an obstacle

# Wall / imminent-surface guard: for real floor, inverse-depth should INCREASE
# toward the bottom of the ROI (nearer = larger) roughly linearly (this is the
# v-disparity ground line). If the fitted slope is much weaker than expected,
# the thing in front is a vertical surface (wall / big near object), not floor
# -> BLOCK. Expressed relative to the expected floor slope (1.0 = as expected).
WALL_SLOPE_REL = 0.15

# If the ground doesn't recede anywhere (the whole ROI is one flat surface
# filling the view), there's no drivable plane -> treat as a wall. This is the
# per-row median rise across the ROI as a fraction of the scene depth level.
WALL_RISE_MIN = 0.06

# ---- VERTICAL-SURFACE DETECTION (the plain-wall fix) -----------------------
# WALL_SLOPE_REL and WALL_RISE_MIN above are WHOLE-COLUMN / WHOLE-ROI tests, so
# they only fire once the wall fills the view -- i.e. when the bumper is already
# against it. A real camera sees a MIXED column: genuine floor at the bottom,
# wall above where they meet. The aggregate slope stays healthy (measured 0.655
# vs the 0.15 threshold for a wall at 1 m) so those guards never trip, and the
# residual test misses it too (max residual 0.0099 vs a 0.356 margin).
#
# The fix uses the LOCAL vertical gradient instead. For a camera at height h:
#     floor: range = h/sin(dep)  -> inverse depth RISES steadily going down
#     wall : range = D/cos(dep)  -> inverse depth is FLAT / falls slightly
# so a run of rows whose gradient has collapsed relative to the floor's own
# gradient is a vertical surface, and the lowest such row is its BASE.
# This is local, so it works on the mixed floor+wall column a real camera sees.
VERT_SURFACE_CHECK = True   # keep True; this is what stops the car at walls

# A row is "not floor" when its local gradient falls below this fraction of the
# floor gradient measured in the same column's near strip. It is a RATIO, so it
# is immune to MiDaS's arbitrary per-frame scale.
#   raise  (0.40-0.50) -> more sensitive: stops earlier, may flag ramps/texture
#   lower  (0.15-0.20) -> more permissive: only very flat surfaces count
VERT_SLOPE_REL = 0.30

# Consecutive collapsed rows needed to confirm a surface. This is the main
# false-positive guard: noise and floor texture produce isolated collapsed rows,
# a real wall produces a long run. Costs a little sensitivity at long range.
#   raise -> fewer false stops, detects walls slightly later
#   lower -> earlier detection, more twitchy on patterned floors
VERT_MIN_RUN = 12

# Half-width (rows) of the window the gradient is measured over. Wider = less
# noise but blurrier localisation of the wall base.
# NOTE: this and VERT_MIN_RUN are row counts tuned at PROC_HEIGHT and are rescaled
# automatically if AUTO_REDUCE_ON_SLOW shrinks the frame.
VERT_GRAD_HALF_WIN = 7

# A vertical surface FURTHER than this (metres) is ignored. This is not laziness:
# the far wall of a corridor is a real wall, and if "wall detected" always means
# "stop" then the vehicle can never drive down a corridor at all. Beyond this range
# the surface is left alone and re-detected as it comes closer.
# It also kills a whole class of false positives, because a distant surface's
# inverse depth is flat by definition -- the far end of any room looks exactly like
# a wall pressed against the bumper once you stop caring about range.
#   set 0 to disable the range gate entirely (not recommended)
VERT_MAX_RANGE_M = 2.0

# Rows above the horizon are not ground at ANY distance -- they are the far wall, a
# doorway, or sky -- so their inverse depth is flat and the vertical-surface test
# fires on all of them. With this on, the ROI top is clamped to sit just below the
# geometry-predicted horizon and a warning is printed once. Keep it True: it turns
# a silent full-width phantom wall into a message telling you the pose is wrong.
CLAMP_ROI_TO_HORIZON = True

# WALL vs OBSTACLE -- decided by SPAN, not by class.
# A wall spans the frame, so no turn escapes it and the honest answer is STOP.
# A box or a person is also a vertical surface, but a LOCAL one, so there is
# clear ground beside it and turning works. When this fraction of columns show a
# vertical surface AND one of them is close, the planner treats it as a wall.
#   raise (0.8-0.9) -> only near-full-width surfaces stop the car; it will try
#                      to squeeze past wide obstacles
#   lower (0.5-0.6) -> more things count as walls; safer, gives up sooner
WALL_SPAN_FRAC = 0.7

# Once a wall is detected, STOP when free space falls to this fraction of the
# planning horizon; above it the wall is known but still far, so the car creeps
# forward at WALL_CREEP_SPEED instead of stopping dead at first sight.
# With PLAN_HORIZON_M = 1.5, 0.60 means "stop at ~0.9 m from the wall".
WALL_STOP_FREE = 0.60
WALL_CREEP_SPEED = 18      # motor percent while closing on a known wall

# Ignore this fraction of rows at the very bottom (use if the robot's own
# chassis / a mount / shadow is visible at the bottom of the frame).
BOTTOM_IGNORE_FRAC = 0.0

# =============================================================================
# 7. ZONES + DECISION THRESHOLDS
# =============================================================================
# Columns are grouped into three driving zones.
# LEFT/CENTER/RIGHT split points as fractions of NUM_COLUMNS.
ZONE_SPLIT_LEFT = 0.34
ZONE_SPLIT_RIGHT = 0.66

# Free-space is reported as a fraction 0..1 of the ROI depth that is clear
# (1.0 = clear to the top of the ROI, 0.0 = obstacle right at the robot).
# Zone status thresholds on that fraction:
FREE_BLOCK_THRESH = 0.25   # below this -> BLOCKED
FREE_WARN_THRESH = 0.55    # below this -> WARNING, else OPEN

# Decision hysteresis (frames) to stop flicker between states.
HYST_FRAMES_ON = 3         # frames a zone must stay bad before we believe it
HYST_FRAMES_OFF = 4        # frames a zone must stay clear before we trust it

# Steering normally follows (left_free - right_free), which has one blind spot: an
# obstacle dead centre with EQUALLY clear ground either side gives a difference of
# zero, so the steer is zero and the car creeps straight into it until the centre
# finally reads BLOCKED. When a vertical surface is actually present in the centre
# zone, the planner commits at least this much steer to one side instead of
# splitting the difference.
#   0  -> old behaviour (drive straight at symmetric obstacles)
#   35-45 -> swerves earlier and harder; may look twitchy in cluttered spaces
PLAN_MIN_AVOID_STEER = 25

# ---- BLIND-CAMERA GUARD (safety) -------------------------------------------
# MiDaS returns *something* for any input -- point the camera at a lens cap or an
# unlit room and it invents a smooth gradient, which the ground-plane fit reads as
# "flat open floor" and the planner turns into FORWARD. That would drive the car
# blind, so a frame that carries no usable information forces STOP instead.
BLIND_CHECK = True         # keep True; this is a safety guard, not a nicety
BLIND_DARK_MEAN = 12.0     # grayscale mean below this = too dark (0..255)
BLIND_MIN_STD = 6.0        # grayscale std below this = no texture to see

# =============================================================================
# 8. CAMERA GEOMETRY -- A REQUIRED MEASUREMENT, NOT AN OPTION
# =============================================================================
# These three numbers convert image ROWS into DISTANCES. Every metric decision in
# the system depends on them, so if they do not match how the phone is actually
# mounted, the whole perception layer lies confidently.
#
# This has already bitten once. With the config below (0.15 m / 20 deg) but the
# phone actually near chest height and roughly level:
#     * the true horizon moved from row 34 to row ~105, so most of the ROI was
#       ABOVE it. Rows above the horizon are far wall, whose inverse depth is flat,
#       so the vertical-surface test fired in 15/15 columns and the HUD reported
#       "wall across path (100% of width)" in an empty hallway.
#     * row -> distance was wrong by 13-21x (row 175 reported as 0.20 m when the
#       truth was 3.02 m), so every zone read BLOCKED and the car sat still.
#
# HOW TO GET THEM RIGHT (2 minutes, no calibration rig):
#   1. Measure the lens height above the ground with a ruler -> CAMERA_HEIGHT_M.
#   2. Run main.py and look at the traversability panel. It draws the horizon the
#      geometry PREDICTS (cyan dashed line) plus distance ticks. Adjust
#      CAMERA_TILT_DEG until the drawn horizon sits on the real one -- where the
#      floor meets the far wall. Raise the tilt number if the drawn line is too
#      low, lower it if too high.
#   3. Run  python tools/roi_check.py  to confirm the ROI is below the horizon and
#      that obstacles at the ranges you care about are visible.
USE_CAMERA_GEOMETRY = False   # show metres on the HUD
CAMERA_HEIGHT_M = 0.15     # lens height above ground (your robot ~15 cm) -- MEASURE IT
CAMERA_TILT_DEG = 20.0     # downward tilt from horizontal (TILT THE CAM DOWN!)
CAMERA_VFOV_DEG = 55.0     # vertical field of view of the phone camera

# Draw the predicted horizon + distance ruler on the traversability panel. This is
# the fastest way to catch a pose mismatch: if the cyan line is not on the real
# horizon, nothing downstream can be trusted.
SHOW_GEOMETRY_OVERLAY = True

# ---- FREE SPACE IN METRES, NOT IMAGE ROWS ----------------------------------
# Image rows are a bad proxy for distance: perspective squeezes the far field
# into very few rows, so counting rows makes far obstacles look harmlessly
# distant. Once ROI_TOP_FRAC was extended to see walls, a wall at 1 m left 92%
# of the ROI rows "clear" -- classified OPEN -- so FREE_BLOCK_THRESH and
# FREE_WARN_THRESH stopped meaning what they say.
#
# With this on, free space = (distance to the obstacle) / PLAN_HORIZON_M, using
# the geometry above. The thresholds then mean real fractions of the planning
# horizon: with PLAN_HORIZON_M = 1.5, FREE_BLOCK_THRESH = 0.25 blocks at 0.38 m
# and FREE_WARN_THRESH = 0.55 warns at 0.83 m.
#
# The geometry only has to be ROUGHLY right, because the value is normalised by
# the horizon -- a 5 deg tilt error shifts the thresholds slightly, it does not
# break the logic. Set False to go back to raw row fractions.
METRIC_FREE_SPACE = True

# How far ahead the robot plans (m). Free space saturates at this distance --
# anything beyond it is equally "clear". Roughly: stopping distance plus a
# margin. Lower = more cautious/twitchy, higher = smoother but later reactions.
PLAN_HORIZON_M = 1.5

# =============================================================================
# 9. VISUALIZATION / OUTPUT
# =============================================================================
PANEL_HEIGHT = 384         # each of the 3 panels is scaled to this height
SHOW_WINDOW = True         # False -> headless (save frames only)
SAVE_DIR = "captures"      # where 's' key / headless mode writes PNGs
DRAW_COLUMN_DEBUG = True   # draw per-column free-space bars on the map
# The 3-panel view is ~3x the frame width, which is wider than many laptop
# screens -- the right-hand panel and the ARMED badge then get cut off. This
# shrinks the window (display only; saved PNGs stay full resolution).
MAX_WINDOW_WIDTH = 1500    # 0 = never shrink

# =============================================================================
# 10. LAYER 2 -- DRIVING THE CAR  (ESP8266 + L298N, 4WD skid-steer)
# =============================================================================
# This section actually MOVES the vehicle, so it is OFF by default and gated
# again at runtime. Nothing moves unless ALL of these are true:
#     (a) you launch main.py with  --drive   (or set SEND_TO_ESP8266 = True), AND
#     (b) you ARM it from the keyboard ('a')  -- it always starts DISARMED, AND
#     (c) the ESP is receiving packets (it self-stops after ESP_FAILSAFE_MS).
#
# The laptop is the brain: it converts each VehicleCommand into left/right motor
# percentages and streams them over UDP. The ESP8266 is a dumb actuator that
# just applies them to the L298N and STOPS on its own if the stream drops.
SEND_TO_ESP8266 = False    # master enable. --drive on the CLI also turns this on.

# The ESP8266's IP address on your router, and the UDP port it listens on.
# Find the IP from the ESP's serial monitor after it connects (it prints it),
# or from your router's client list. It MUST match firmware UDP_PORT.
ESP_IP = "192.168.137.76"    # <-- set to YOUR ESP's IP (printed on its serial)
ESP_PORT = 4210

# How often the laptop repeats the latest command. This must be COMFORTABLY
# faster than the ESP failsafe, or the car will stutter-stop. 20 Hz vs 400 ms
# gives ~8 packets per failsafe window.
ESP_SEND_HZ = 20.0
ESP_FAILSAFE_MS = 400      # MUST equal FAILSAFE_MS in the ESP firmware.

# Global speed cap applied to every motor command (percent). Keep this LOW for
# first drives -- raise it only once steering direction and stopping are proven.
DRIVE_MAX_SPEED_PCT = 40

# Skid-steer mixing (see vision/motor_mixer.py).
#   * FORWARD uses arcade mixing: steering trims the two sides.
#   * A LEFT / RIGHT command pivots in place (one side forward, one back) when
#     DRIVE_PIVOT_TURNS is True; otherwise it arcs (inner side stopped).
DRIVE_TURN_SPEED_PCT = 35  # magnitude used for LEFT/RIGHT pivot turns
DRIVE_STEER_MIX = 0.6      # 0..1: how strongly FORWARD steering splits the sides
DRIVE_PIVOT_TURNS = True

# Motor polarity fixes. If a side spins the WRONG way during the teleop test,
# flip its invert flag here instead of rewiring. If the whole car goes backwards
# on 'forward', flip BOTH.
DRIVE_INVERT_LEFT = False
DRIVE_INVERT_RIGHT = False

