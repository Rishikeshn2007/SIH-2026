# SIH-26126 UGV — Vision Navigation

Vision-based free-space perception and driving for a small outdoor Unmanned
Ground Vehicle. The AI brain runs on a laptop, reads a live camera stream from a
phone, works out *where the vehicle could drive*, and (Layer 2) streams motor
commands to an ESP8266 on the car.

- **Layer 1 — Perception** (default): reads the camera, produces a
  `VehicleCommand`, and only **logs** it. Nothing moves.
- **Layer 2 — Driving** (opt-in, `--drive`): streams that command to the
  ESP8266 → L298N → motors, behind an arm switch, a speed cap, and a hardware
  failsafe. See [Layer 2 — Driving the car](#layer-2--driving-the-car-esp8266--l298n).

```
   Phone (IP camera) ──Wi-Fi──▶ Laptop (AI brain: YOLO + MiDaS + ground-plane)
                                        │
                                        │  → LEFT / CENTER / RIGHT decision
                                        ▼
                                  VehicleCommand
                                        │  (Layer 1: logged only)
                                        │  (Layer 2: --drive, ARMED)
                                        ▼
                        UDP @20Hz ▶ ESP8266 ▶ L298N ▶ 4 motors (skid-steer)
                        (failsafe: motors stop if the stream drops)
```

---

## ⚠️ Safety model

By default this is **perception only** — the planner produces a `VehicleCommand`
(STOP / FORWARD / LEFT / RIGHT / REVERSE + speed + steering) and just prints it;
nothing is sent to the ESP8266.

Driving is opt-in and gated three ways, so the car cannot move by accident:

- **Enable gate:** motor packets are sent only with `--drive` (or
  `SEND_TO_ESP8266 = True`).
- **Arm gate:** it always starts **DISARMED**; you must press `a`. `SPACE` is an
  e-stop, and a global speed cap (`DRIVE_MAX_SPEED_PCT`) limits every command.
- **Failsafe:** the ESP stops the motors on its own if the command stream drops
  for `FAILSAFE_MS` (Wi-Fi loss, program closed, pause).

No AI runs on the ESP8266 — it is a dumb actuator. SLAM, global path-planning,
and mapping are still deliberately out of scope (see the end of this doc).

---

## The problem this solves

Earlier attempts made the vehicle **stop at far-away open ground**, **drive into
plain walls and floors**, and generally get "confused by floors, plain walls, far
objects, and near big objects."

The root cause is a perspective mistake. A phone on a ~15 cm-tall robot is **like
an ant**: its whole world is the patch of ground just ahead. But the naive
approach was *"run MiDaS depth, normalise it, and treat whatever looks nearest as
an obstacle."* That fails because:

1. **MiDaS depth is relative, not metric.** Its output is *inverse relative
   depth* (bigger = nearer) with an unknown scale that changes every frame. In an
   open field the nearest patch of empty ground gets stretched to look "near," so
   the robot brakes for nothing. You can **never** threshold it as metres.
2. **The floor is always the nearest thing.** Ground right in front of the wheels
   is genuinely the closest surface, so "near = obstacle" flags the floor itself.
3. **Plain walls are invisible to object detectors.** COCO YOLO has no *wall*,
   *curb*, or *rock* class, so a blank wall produces no detection at all.

### The fix: model the ground, flag deviations from it

Instead of asking *"what is near?"* we ask *"how far can I drive before the ground
stops behaving like ground?"*

For a forward camera looking over flat ground, inverse-depth rises **roughly
linearly** from the top of the region-of-interest (far) down to the bottom
(near). That straight line *is* the ground plane (the classic **v-disparity**
ground line). So the pipeline:

1. keeps only the **lower ROI** — the ground the robot will actually roll into;
2. splits it into vertical **columns** and, per row, takes the **median across the
   width** (an obstacle is a minority of the width, so the median tracks the
   *floor*, not the obstacle — this is what stops one near object from poisoning
   the whole scene);
3. fits the floor's depth trend from the near reference strip (assumed drivable);
4. marks a column **blocked** where actual inverse-depth rises *above* that floor
   line by a scene-relative margin (something is nearer than the floor should be);
5. adds a **wall guard**: if the ground doesn't recede like a floor anywhere, a
   vertical surface is filling the view → blocked;
6. smooths over time and applies **per-zone hysteresis** so decisions don't
   flicker.

Everything is expressed *relative to the scene's own depth spread*, so it behaves
the same in a tight corridor and an open field. YOLO is demoted to a **semantic +
safety** role: it fires a hard STOP when a hard-stop class (e.g. a person) is in
the driving corridor; the geometry does the general obstacle work.

### Round 2: why it still drove into plain walls

Steps 4 and 5 above passed their synthetic test and *still missed real walls*. Two
independent root causes, both now fixed:

**(a) The synthetic wall was not a wall.** The old test scene was
`np.full((H, W), 0.7)` — a perfectly uniform inverse-depth field, which means a
wall filling the *entire* frame with no floor visible. For a camera at 15 cm
tilted 20° down that only happens when the bumper is already touching it. A real
camera sees a **mixed column**: genuine floor at the bottom, wall above the line
where they meet. Steps 4 and 5 are *aggregate* tests — a whole-ROI rise and a
whole-column slope fit — and a mixed column is still monotonically increasing, so
the best-fit line just tilts to split the difference. Measured on a
geometrically-correct wall at 1 m put through a MiDaS-like degradation:

| guard | measured | threshold to fire | fired? |
|-------|----------|-------------------|--------|
| `WALL_RISE_MIN` (whole ROI) | rise ratio **+0.4974** | < 0.06 | no |
| `WALL_SLOPE_REL` (whole column) | rel. slope **+0.655** | < 0.15 | no |
| `RESIDUAL_MARGIN_FRAC` | max residual **+0.0099** | > 0.3562 | no (36× short) |

No amount of retuning fixes that — the signal genuinely isn't in the aggregate.

**(b) The ROI was clipped below the wall's base.** `ROI_TOP_FRAC = 0.45` is a
fraction of *image height*, which tells you nothing about ground coverage. For
this camera it corresponded to only **0.489 m** of ground, so a wall 1 m away had
its base *above* the ROI and was never examined. The robot was not failing to
recognise the wall; it was not looking at it. Run `python tools\roi_check.py` to
print this for your own height/tilt.

### The wall fix: the *local* vertical gradient

The discriminating physics is local, not aggregate. At depression angle `dep`
below the optical axis, for a camera at height `h`:

```
floor:  range = h / sin(dep)     → inverse depth RISES steadily going down
wall :  range = D / cos(dep)     → inverse depth is FLAT, and falls slightly
```

Measured over a 10-row step at 0.5 m: floor **+0.153**, wall **−0.026**. So a
**vertical surface is a run of rows whose local gradient has collapsed relative to
the floor's own gradient measured in the same column's near strip**, and the
lowest such row is the surface's **base** — which gives a real distance. Because
it is a *ratio* against the floor's own slope in that same frame, MiDaS's
arbitrary per-frame scale cancels out, and the floor can never trip it. Because it
is *local*, it works on the mixed floor+wall column a real camera actually sees.

Three more changes follow from it:

- **The ROI now reaches ~2.3 m** (`ROI_TOP_FRAC = 0.22`), still below the horizon
  (row 58 here), so wall bases at 0.5–2 m land inside it.
- **Free space is measured in metres, not image rows** (`METRIC_FREE_SPACE`,
  `PLAN_HORIZON_M`). Perspective squeezes the far field into very few rows, so
  once the ROI was extended a wall at 1 m left 92 % of rows "clear" → `OPEN`.
  Normalising by the planning horizon makes the thresholds mean real distances:
  block at 0.38 m, warn at 0.83 m, wall-stop at 0.90 m.
- **Wall vs obstacle is decided by SPAN, not by class** (`WALL_SPAN_FRAC = 0.7`).
  A wall spans the frame, so no turn escapes it → the honest answer is STOP. A box
  or a person is *also* a vertical surface, but a **local** one, so there is clear
  ground beside it → turn. This keeps the decision a traversability judgement
  rather than an identity one: `tools\smoke_test.py` scene **H** detects and
  steers around an unlabelled box with **no `Detection` object at all**.

MiDaS was **not** replaced and no `wall` class was added to YOLO. The failure was
in the geometry layer's aggregate fits and in the ROI extent, not in the depth
model. Both fixes are load-bearing — ablation in `tools\wall_probe.py`: disabling
`VERT_SURFACE_CHECK` fails every realistic-wall check, and reverting
`ROI_TOP_FRAC` to 0.45 fails them too. Neither alone is sufficient.

### Round 3: the phantom wall — why it then stopped dead in an empty hallway

The wall detector worked, and immediately produced the opposite failure. In an
open indoor corridor the HUD read `WALL ACROSS PATH (86%)`, every zone came back
`BLOCKED 0.13 / WARNING 0.26 / OPEN 0.57`, and the command was
`STOP  SPEED 0  STEER +0`. The link was fine — `sent=412 errors=0`, `ARMED
L=+0 R=+0` — so the brain was *deliberately* commanding zero. A false positive is
just as much a failure as a miss: a robot that stops for nothing never finishes a
run, and worse, it trains the operator to disbelieve the wall banner.

Two hypotheses were measured and **ruled out** before the real one was accepted:

- *"A receding floor looks flat far away, so the far ROI trips the vertical
  test."* For a true geometric floor at H=224 the inverse-depth gradient is
  **larger** far away (0.0289 at the ROI top) than near (0.0181 in the reference
  strip), because `d(sin dep)/dv = cos(dep)·f/(f² + (v−cy)²)`. A receding floor
  therefore *cannot* trip the test. Hypothesis dead.
- *"A corridor's side walls fill the outer columns, so the span count saturates."*
  A rendered corridor (`render_corridor`) at 640×384 and 384×224, with end walls
  at 3/5/8 m and with infinite open floor, returned `FORWARD` with `span = 0.00`
  every time. Hypothesis dead.

What *did* reproduce it, exactly, was a **camera-pose mismatch** — the config
claimed `CAMERA_HEIGHT_M = 0.15`, `CAMERA_TILT_DEG = 20` while the phone was near
chest height and roughly level:

```
scene rendered at the real pose, analysed with config's pose:
  h=0.15m tilt=20deg  ->  L=1.00 C=1.00 R=1.00  vert=0/15   span=0.00  FORWARD
  h=0.60m tilt=5deg   ->  L=0.12 C=0.25 R=0.12  vert=15/15  span=1.00  STOP
  h=1.00m tilt=2deg   ->  L=0.09 C=0.09 R=0.09  vert=15/15  span=1.00  STOP
  h=1.10m tilt=0deg   ->  L=0.09 C=0.09 R=0.09  vert=15/15  span=1.00  STOP
```

Compare row 2 with what the real robot printed: `0.13 / 0.26 / 0.57` at 86 %.

**Enter the horizon.** For a camera tilted `t` below horizontal with vertical FOV
`vfov`, the horizon sits at

```
f = (H/2) / tan(vfov/2)          horizon_row = H/2 + f·tan(−t)
```

Note what is *absent*: camera **height cancels out**. Rows above that line are not
ground at any distance — and their inverse depth is flat *by definition*, which is
precisely the signature of a wall against the bumper. Declaring 20° of tilt puts
the horizon at row 34; the phone's real ~2° put it at row ~105, so the ROI (top row
49) sat almost entirely **above** it. Result: the vertical test fired in **15 of 15
columns**. The same bad pose also corrupted every distance — row 175 was reported
as **0.20 m when the truth was 3.02 m** (14.8× under), row 140 as 0.29 m vs 6.03 m
(20.9×) — so every zone read BLOCKED independently.

Five minimal changes, no rewrite:

- **`CLAMP_ROI_TO_HORIZON`** — the ROI is clamped just below the predicted horizon,
  with a console warning naming the cause and the fix.
- **`VERT_MAX_RANGE_M = 2.0`** — a vertical surface beyond this is ignored. The far
  wall of a corridor *is* a wall, but if "wall detected" always meant "stop", the
  vehicle could never drive down a corridor at all.
- **Row-count knobs scale with resolution.** `AUTO_REDUCE_ON_SLOW` had quietly
  dropped the frame 640×384 → 384×224, so the ROI went from 300 rows to 175 and
  `VERT_MIN_RUN = 12` silently meant something else. It is now scaled by
  `H / PROC_HEIGHT`.
- **The planner uses travel direction, not the worst zone.** `min(cf, lf, rf)`
  looked safer and was a trap: in *any* corridor the side zones legitimately see
  the side walls, so the minimum is always small and once `wall_ahead` fired the
  car could never move again. It now closes carefully if the centre is clear,
  escapes sideways to whichever side is open, and only reports "wall across path"
  when neither is.
- **The assumption is now visible** — a cyan dashed horizon line and distance ticks
  are drawn on the traversability panel every frame, and `main.py` prints the
  assumed pose at startup.

One more thing fell out of the same investigation. Steering was
`(left_free − right_free) × gain`, which is the right instinct with one blind spot: an
obstacle **dead centre with equally clear ground either side** gives a difference of
exactly zero, so the steer was `+0` and the car crept straight into something it had
correctly detected. `PLAN_MIN_AVOID_STEER` now makes the planner **commit to a side**
whenever a vertical surface is actually present in the centre zone (using the new
per-zone `zone_vert` — `wall_frac` alone cannot answer *which way is out*), and it
starts easing around while the centre still reads OPEN rather than waiting for
WARNING. Scene **L** pins it.

The honest limit: an **undeclared** pose is garbage-in and cannot be recovered.
Scene **J** in the smoke test still (correctly) STOPs. Scene **K** feeds the
*identical* depth map to an analyzer that has simply been *told* the real pose —
no threshold touched — and it drives. That pair is the whole lesson:

> **Camera pose is a measurement, not a configuration default. A wrong pose does
> not raise an error; it lies confidently, by a factor of 15.**


---

## Project structure

```
Unmanned_vehicle/
├── config.py                 # ALL tunables live here (read the comments)
├── main.py                   # entry point: capture → infer → decide → drive/display
├── RUNBOOK.md                # ⭐ ordered Step 0–6 bring-up with checkpoints
├── requirements.txt
├── README.md
├── vision/
│   ├── camera.py             # threaded IP/webcam/video capture + auto-reconnect
│   ├── depth_estimator.py    # MiDaS (relative inverse depth) via torch.hub
│   ├── yolo_detector.py      # YOLO11n-seg (semantic layer + hard-stop safety)
│   ├── fusion.py             # combine depth + segmentation, build hard-stop mask
│   ├── traversability.py     # ⭐ the ground-plane free-space core (the fix)
│   ├── planner.py            # VehicleCommand decision (LEFT/RIGHT/FORWARD/STOP)
│   ├── motor_mixer.py        # Layer 2: command → left/right skid-steer percent
│   ├── esp_link.py           # Layer 2: threaded UDP sender + arm gate + speed cap
│   ├── visualization.py      # 3-panel live view + HUD
│   └── utils.py              # device pick, FPS meter, rate limiter, blind check
├── firmware/
│   └── esp8266_l298n/
│       └── esp8266_l298n.ino # the ESP sketch: UDP → L298N, + 400 ms failsafe
├── tools/
│   ├── scene_gen.py          # geometrically-correct synthetic floor/wall/box depth
│   ├── smoke_test.py         # offline perception test, synthetic scenes (no GPU)
│   ├── roi_check.py          # ⭐ "can the ROI even see a wall at 1 m?" geometry check
│   ├── wall_probe.py         # ⭐ why a wall was / wasn't detected, step by step
│   ├── test_drive.py         # offline mixer + UDP link test (no ESP needed)
│   ├── check_link.py         # ⭐ "why won't it move?" — safe link diagnostic
│   └── teleop.py             # manual keyboard driving (w/a/s/d)
└── captures/                 # saved frames ('s' key, headless, smoke test)
```

---

## Installation (Windows)

You need Python 3.10 or 3.11 and, ideally, an NVIDIA GPU for real-time speed.

```bat
py -3.11 -m venv .venv
.venv\Scripts\activate
python -m pip install --upgrade pip
```

**Install PyTorch first** (pick the one that matches your machine):

```bat
:: NVIDIA GPU (CUDA 12.1) — recommended
pip install torch torchvision --index-url https://download.pytorch.org/whl/cu121

:: OR CPU only (works but slow — lower YOLO_FPS/MIDAS_FPS in config.py)
pip install torch torchvision --index-url https://download.pytorch.org/whl/cpu
```

Then the rest:

```bat
pip install -r requirements.txt
```

The **first run needs internet** — `ultralytics` downloads `yolo11n-seg.pt` and
`torch.hub` downloads the MiDaS weights (both cached afterwards, then it runs
offline). If you already have your own MiDaS weights, set
`MIDAS_CUSTOM_WEIGHTS` in `config.py` to skip the hub download.

---

## Phone IP-camera setup

1. Install an IP-camera app on the phone (e.g. "IP Webcam" on Android). Start the
   server; it shows a URL like `http://192.168.1.50:8080`.
2. Put the phone and the laptop on the **same Wi-Fi network**.
3. Set `CAMERA_URL` in `config.py` to the stream endpoint:

   ```python
   CAMERA_URL = "http://192.168.1.50:8080/video"      # MJPEG
   # or
   CAMERA_URL = "rtsp://192.168.1.50:554/h264_ulaw.sdp"  # RTSP
   ```

   (There is intentionally **no** default URL — use your phone's real address.)
4. Test connectivity by opening that URL in the laptop's browser first.

No phone yet? Leave `CAMERA_URL = ""` and it falls back to the laptop webcam
(`FALLBACK_SOURCE = 0`), or pass `--source path\to\clip.mp4`.

---

## Camera mounting — the real-world half of the fix

The software models the ground plane, but it can only see ground if the camera is
pointed at ground. For an ant-sized robot this matters more than anything:

- **Tilt the camera DOWN**, roughly 15–25° below horizontal. The bottom of the
  frame should be the ground ~20–40 cm in front of the wheels; the top should be
  around the horizon. If the camera stares straight ahead, it sees mostly sky and
  distant clutter and has almost no drivable ground to reason about.
- **Mount it rigidly.** Every wobble shifts the whole depth field and the ground
  line jumps. A shaky phone clamp is a real source of false stops. Bolt/zip-tie
  it; don't rely on friction.
- **Mount it as high on the chassis as practical** and keep the lens clean. A
  little more height gives the ground line a longer, cleaner run.
- **Keep the robot's own body out of the frame.** If a bumper or wheel shows at
  the very bottom, raise `BOTTOM_IGNORE_FRAC` (e.g. `0.05`) to crop it.

### Then MEASURE it — `CAMERA_HEIGHT_M` and `CAMERA_TILT_DEG` are not optional

Every distance this system produces is derived from those two numbers, and a wrong
pose does not raise an error — **it lies**. With config claiming 0.15 m / 20° while
the phone sat at chest height and level, row 175 was reported as 0.20 m when the
truth was 3.02 m (14.8× under), and the vehicle sat still in an empty hallway
reporting `WALL ACROSS PATH (100%)`. No amount of threshold tuning fixes that; only
declaring the real mount does. This is the single most common cause of "it stops for
nothing".

Calibrate in about a minute:

1. **Measure the lens height** above the ground with a ruler → `CAMERA_HEIGHT_M`.
2. **Run `python main.py`.** The traversability panel draws the horizon this
   geometry *predicts* as a **cyan dashed line**, plus distance ticks at
   0.5 / 1.0 / 1.5 / 2.0 m. Adjust `CAMERA_TILT_DEG` until that line lands where the
   floor meets the far wall — too low, raise the tilt; too high, lower it. Then sanity-check
   a tick: put a shoe at 1.0 m and see whether it sits on the `1.0m` mark.
3. **Run `python tools\roi_check.py`** to confirm the ROI is below the horizon and
   covers the distances you care about.

`main.py` prints the assumed pose on every startup and warns if the ROI would begin
above the horizon, so the assumption is never silent.

---

## Running

> **First time, or the car isn't moving? Follow [`RUNBOOK.md`](RUNBOOK.md)
> instead.** It is an ordered Step 0–6 bring-up (power → link → motors → camera →
> autonomy wheels-up → autonomy on the ground), each with a checkpoint that must
> pass before the next. The commands below are the reference, not the procedure.

```bat
python main.py                      :: use config (IP cam, else webcam fallback)
python main.py --source 0           :: force laptop webcam
python main.py --source clip.mp4    :: force a video file
python main.py --source rtsp://...  :: force an IP stream
python main.py --headless --frames 60   :: no window; process 60 frames, save PNG
python main.py --no-yolo            :: geometry only (skip YOLO)
python main.py --no-midas           :: skip depth (YOLO only; for debugging)
```

**`main.py` on its own never transmits anything** — it is perception only. Add
`--drive` to enable motion, and even then it starts DISARMED until you press `a`.

### Controls (windowed)

| key | action |
|-----|--------|
| `q` | quit |
| `p` | pause / resume |
| `s` | save the current 3-panel view to `captures/` |

### The 3-panel display

- **Left — Camera + YOLO:** the live frame with segmentation masks and labels;
  each object is tagged NEAR/far from the fused depth.
- **Middle — Depth (MiDaS):** the relative inverse-depth as a MAGMA colormap
  (brighter = nearer). Labelled *relative* on purpose — **display only.**
- **Right — Traversability:** the ROI shaded green→red per column by how far it's
  clear, obstacle "feet" marked, walls flagged `W`, and the LEFT/CENTER/RIGHT
  zones boxed and coloured by status. A **HARD STOP** banner appears for hard-stop
  objects.
  - An **orange line with hatching above it and a `V` marker** is a detected
    **vertical surface** — the line is its base, i.e. the line the car must not
    cross. This is the fastest way to tell a real wall from a depth artefact: a
    wall marks `V` in nearly every column at a consistent height, noise marks it
    in one or two at random heights.
  - **`WALL ACROSS PATH (n%)`** means enough columns flagged a vertical surface
    that no turn escapes it (`n` ≥ `WALL_SPAN_FRAC`). Below that threshold the
    same surface is treated as a local obstacle and the planner steers around it.
  - A **cyan dashed line** is the horizon your `CAMERA_TILT_DEG` *predicts*, and
    the grey ticks on the left edge are 0.5 / 1.0 / 1.5 / 2.0 m. **Check them
    against the real world.** If the cyan line is not where the floor meets the far
    wall, the pose is wrong and every number on this panel is wrong with it — that
    is exactly how the robot came to report a wall across an empty hallway. Turn the
    overlay off with `SHOW_GEOMETRY_OVERLAY = False` once you trust the mount.

A HUD bar underneath shows FPS, YOLO ms, MiDaS ms, total ms, resolution, device,
and the current logged command (`CMD / SPEED / STEER / reason`).

---

## Tuning (`config.py`)

You rarely need to touch code — the knobs are all in `config.py`. The ones that
matter most for behaviour:

| Symptom | Try |
|---------|-----|
| **`WALL ACROSS PATH` in an open space / stops for nothing** | **Check the pose first, not the thresholds.** Is the cyan horizon line where the floor meets the far wall? If not, fix `CAMERA_TILT_DEG` / `CAMERA_HEIGHT_M` and re-run `roi_check.py`. This was the cause every time so far |
| Stops for far / open ground | raise `RESIDUAL_MARGIN_FRAC` (e.g. 0.22–0.28); confirm the camera is tilted down |
| Ignores real obstacles | lower `RESIDUAL_MARGIN_FRAC`; lower `MIN_OBSTACLE_RUN` to 2 |
| **Doesn't stop for plain walls** | first run `python tools\roi_check.py` — if the wall's distance says `INVISIBLE` the ROI is the problem, lower `ROI_TOP_FRAC`. Otherwise raise `VERT_SLOPE_REL` (0.40–0.50) or lower `VERT_MIN_RUN` (8–10) |
| Stops for a wall it should just drive towards | lower `VERT_MAX_RANGE_M` (1.5) so distant verticals are ignored |
| Doesn't react to a wall until very close | raise `VERT_MAX_RANGE_M` (2.5–3.0) — but confirm the ROI reaches that far in `roi_check.py` first |
| Stops for ramps / patterned floors | lower `VERT_SLOPE_REL` (0.15–0.20) and/or raise `VERT_MIN_RUN` (14–18) |
| Stops too late at a wall | raise `WALL_STOP_FREE` (0.70 → stops ~1.05 m out) |
| Gives up instead of squeezing past a wide obstacle | raise `WALL_SPAN_FRAC` (0.8–0.9) |
| Treats a local box as a wall | raise `WALL_SPAN_FRAC`; check the panel — a wall shows `V` in nearly every column |
| Behaviour changed on its own mid-run | look for `[perf] auto-reducing proc size` — the frame shrank. Row-count knobs are auto-scaled for this, but if it keeps happening lower `MIDAS_FPS` instead |
| Detects an obstacle but drives at it anyway | raise `PLAN_MIN_AVOID_STEER` (35–45). With equal clearance either side, `left−right` is zero, so this is the only thing that breaks the tie |
| Swerves too eagerly in cluttered spaces | lower `PLAN_MIN_AVOID_STEER` (0 restores pure `left−right` steering) |
| Decisions flicker | raise `HYST_FRAMES_ON` / `HYST_FRAMES_OFF`, or `DEPTH_TEMPORAL_ALPHA` toward 0.3 |
| Too slow / low FPS | lower `YOLO_FPS` & `MIDAS_FPS`; lower `PROC_WIDTH/HEIGHT`; keep `AUTO_REDUCE_ON_SLOW=True` |
| Robot body in frame | raise `BOTTOM_IGNORE_FRAC` |
| Turns too wide of decisions | adjust `ZONE_SPLIT_LEFT` / `ZONE_SPLIT_RIGHT`, `FREE_BLOCK_THRESH`, `FREE_WARN_THRESH` |

**Before any of the above:** if the console prints `[trav] ROI_TOP_FRAC=... puts the
ROI top ... ABOVE the horizon`, stop tuning and fix the camera pose. Rows above the
horizon have flat inverse depth, which is indistinguishable from a wall at the
bumper, so the vertical test will fire in every column no matter what the thresholds
say.

`ROI_TOP_FRAC` (0.22) sets how far ahead the robot can see. **Never tune it
blind** — it is a fraction of image height, so it says nothing about ground
coverage on its own. `tools\roi_check.py` converts it to metres for your camera
height and tilt, and warns if you push it above the horizon (rows above the
horizon carry no ground-plane information and only add noise).

`WALL_SLOPE_REL` and `WALL_RISE_MIN` are the older whole-column / whole-ROI
guards. They only fire when a surface fills the view, so they are now a backstop;
the vertical-gradient knobs above are what actually stops the car at walls.

Stopping distances are set by `PLAN_HORIZON_M` (1.5 m) together with
`FREE_BLOCK_THRESH`, `FREE_WARN_THRESH` and `WALL_STOP_FREE` — multiply them to
get metres, or just read them off `roi_check.py`.

---

## Testing

**Offline logic test (no GPU, no camera, no torch needed):**

```bat
python tools\smoke_test.py
```

This byte-compiles every module and runs the fusion → traversability → planner →
visualization pipeline on eleven synthetic MiDaS-style scenes that reproduce the
exact bugs, asserting the right decision each time:

| scene | expected |
|-------|----------|
| A flat floor ahead | FORWARD (floor is **not** an obstacle) |
| B floor + near centre object | CENTER blocked → turn LEFT/RIGHT |
| C gently receding far ground | FORWARD (**not** falsely flagged) |
| D uniform wall filling the frame | STOP (old whole-column guard) |
| E person in the corridor | STOP (hard-stop) |
| F **realistic** floor+wall at 0.6 m | STOP, by geometry, **no labels** |
| G same wall at 2.0 m | FORWARD (not a false stop) |
| H unlabelled box, no `Detection` | turn — a *local* vertical surface |
| I **open corridor**, side walls + end wall | FORWARD — **not** a phantom wall |
| J same corridor at an **undeclared** pose | STOP — garbage-in must not be trusted |
| K same depth map, pose **declared** | FORWARD — the pose was the bug |
| L centred box, sides *equally* clear | FORWARD **with steer ≠ 0** — must not creep straight in |

Scenes F–K come from `tools\scene_gen.py`, which renders from actual geometry
(`h/sin(dep)` on the floor, `D/cos(dep)` on the wall) and then degrades it the way
`MiDaS_small` does — 256×256 inference bicubic-upsampled, compressive near field,
arbitrary per-frame affine scale. Testing against *perfect* depth flatters the
pipeline and is how the missed-wall bug hid: **scene D passed the whole time**.

**I** is the hardest honest case for a span-based wall test: a corridor's side walls
genuinely *are* vertical surfaces and genuinely *do* fill the outer columns, so a
naive full-width span count reaches 100 % in a hallway you could drive down all day.

**J and K are a matched pair, and the pair is the point.** Both are handed the
*identical* depth map — a corridor rendered at 1.0 m / 2° while the config claims
0.15 m / 20°. J analyses it with the wrong declared pose and must STOP; K analyses it
with the pose declared correctly, with **no threshold changed**, and drives. If K
ever fails, the horizon clamp has regressed. If J ever "passes" by driving, the
system has started trusting a pose it has no right to trust.

**L exists because scene H was a weak test.** H's box is close enough that the centre
reads BLOCKED, so it exits through the turn rule and would pass even if steering were
dead. L puts the box far enough out that the centre only reads WARNING *and* leaves
equal clearance both sides, so `left_free − right_free` is exactly zero — the case
where the old planner steered `+0` and crept straight into an obstacle it had
correctly detected. Ablation:

```
PLAN_MIN_AVOID_STEER= 0   centre_vert=0.60  lf-rf=+0.000  ->  FORWARD steer=+0
PLAN_MIN_AVOID_STEER=25   centre_vert=0.60  lf-rf=+0.000  ->  FORWARD steer=-25
```

Same detection either way; the difference is whether "saw it" becomes "avoided it".
Keep D, but treat F/G/H as the regression guards.

It writes an annotated panel per scene plus `captures/smoke_ALL.png` so you can
eyeball the result. All 14 checks currently pass.

**Geometry sanity check — run this whenever you move or re-tilt the camera:**

```bat
python tools\roi_check.py
```

It converts `ROI_TOP_FRAC` into the ground distance the ROI actually covers, warns
if the ROI has crossed the horizon, prints a table of whether an obstacle at
0.2–3.0 m has its base inside the ROI at all, and converts the decision thresholds
to metres. Anything marked `INVISIBLE — base is above the ROI` **cannot** be
detected no matter how good the depth is.

**Why was / wasn't that wall detected:**

```bat
python tools\wall_probe.py
```

Walks the wall logic step by step across the old uniform scene, geometrically
correct walls at several distances with perfect depth, the same through the
MiDaS-like degradation, and a box control — printing per-scene free space, how many
columns flagged a vertical surface, the old guards' verdict, and the final command.
This is the tool to reach for when the live view disagrees with your eyes.

**Offline drive-path test (no ESP needed — uses a local UDP loopback):**

```bat
python tools\test_drive.py
```

This checks the skid-steer mixer (correct left/right for every command) and the
`EspLink` sender (well-formed packets, starts disarmed, speed cap, invert flags,
e-stop, and that packets keep streaming to feed the failsafe). All pass.

**Link diagnostic — run this whenever the car does not move (nothing spins):**

```bat
python tools\check_link.py          :: safe: ARMED but zero speed, no motion
python tools\check_link.py --spin   :: also pulses the motors — WHEELS UP ONLY
```

It dumps the drive config, checks whether the laptop and the ESP are even on the
same subnet, pings the ESP, then streams armed zero-speed packets for 5 s. **The
NodeMCU's blue LED is the verdict** — the firmware lights it solid only when an
armed packet actually arrives:

| LED during the 5 s | Meaning | Where the fault is |
|--------------------|---------|--------------------|
| **solid on** | packets are arriving | **electrical** — motor battery, L298N power, OUT wiring, or `MIN_PWM` too low. Try `--spin`. |
| **off** | packets are not arriving | **network** — wrong/stale `ESP_IP`, different subnet, Windows Firewall, router AP isolation |
| **blinking** | ESP still joining Wi-Fi | SSID/password in the sketch |

Remember that UDP is unacknowledged: a successful `sendto` proves the packet left
the laptop, **not** that the ESP received it. Only the LED (or motion) proves the
far end. `main.py --drive` now also prints `[esp] !! CANNOT SEND ...` on a bad
route and a 2-second `[esp] ARMED L=.. R=.. sent=N errors=N` heartbeat, so a dead
link is visible in the console instead of silent.

**Live tests to run on the vehicle's laptop:**

1. Webcam sanity: `python main.py --source 0`, wave a hand in close → CENTER
   should block and the command turn/stop.
2. IP camera: set `CAMERA_URL`, walk the phone toward a wall → STOP; point down an
   open corridor → FORWARD.
3. Disconnect test: kill the phone stream mid-run → a "WAITING FOR CAMERA" card,
   no crash; restart it → it reconnects on its own.
4. Objects: try a person (hard STOP), a box in the centre (turn), open ground
   (forward).

---

## Layer 2 — Driving the car (ESP8266 + L298N)

Layer 2 actually **moves the vehicle**. Your car is a **4-wheel skid-steer**
(tank-style) base: four TT gear motors driven by an **L298N** dual H-bridge,
commanded by a **NodeMCU ESP8266**. There is no steering servo — it turns by
running the left and right wheel pairs at different speeds/directions.

The laptop stays the brain. Each `VehicleCommand` is converted (in
`vision/motor_mixer.py`) into left/right motor percentages and streamed over
**UDP at ~20 Hz** to the ESP, which just applies them to the L298N. The ESP is
deliberately dumb, and three independent safety gates stand between the code and
a moving car:

1. **Enable gate** — nothing is sent unless you launch with `--drive` (or set
   `SEND_TO_ESP8266 = True`).
2. **Arm gate** — it always starts **DISARMED**. The car cannot move until you
   press `a`. `SPACE` is an e-stop (disarm) at any time.
3. **Failsafe** — the ESP stops the motors on its own if it doesn't receive a
   fresh packet within `FAILSAFE_MS` (400 ms). Close the program, lose Wi-Fi, or
   pause, and the car stops by itself.

There is also a global **speed cap** (`DRIVE_MAX_SPEED_PCT`, default 40%).

### Wiring (NodeMCU → L298N)

This is **your** wiring (as flashed):

| NodeMCU pin | L298N pin | meaning |
|-------------|-----------|---------|
| D5 (GPIO14) | ENA       | LEFT side speed (PWM) |
| D1 (GPIO5)  | IN1       | LEFT direction |
| D2 (GPIO4)  | IN2       | LEFT direction |
| D8 (GPIO15) | ENB       | RIGHT side speed (PWM) |
| D6 (GPIO12) | IN3       | RIGHT direction |
| D7 (GPIO13) | IN4       | RIGHT direction |
| GND         | GND       | **must share ground with the ESP** |

Motors: both **left** wheel motors → L298N `OUT1/OUT2`; both **right** wheel
motors → `OUT3/OUT4` (parallel each side). Battery `+/–` → L298N `12V/GND`.

The firmware bakes in your tested direction convention (LEFT forward = IN1 HIGH,
RIGHT forward = IN3 LOW — the right side is mirror-wired), so a `FORWARD` command
drives the whole car forward with no rewiring. If a side still runs backwards,
flip `DRIVE_INVERT_LEFT` / `DRIVE_INVERT_RIGHT` in `config.py`.

> Note: GPIO15 (D8) must be LOW at boot. The L298N `ENB` input doesn't pull it
> up, so the board boots fine with this connected. If the ESP ever won't boot,
> unplug the ENB wire, boot, then reconnect it.

**Powering the ESP:** the cleanest and most reliable option is a **separate 5 V
source (a small USB power bank) for the NodeMCU**, with its ground tied to the
L298N ground. Powering the ESP from noisy motor rails can brown it out and reset
it mid-drive. (If you must use the L298N's 5 V output, only do so with its
5 V-enable jumper on and the motor supply ≤ 12 V.)

### Flash the firmware

1. Arduino IDE → **File ▸ Preferences ▸ Additional Boards Manager URLs**, add:
   `https://arduino.esp8266.com/stable/package_esp8266com_index.json`
2. **Tools ▸ Board ▸ Boards Manager** → install **esp8266**. Then select
   **NodeMCU 1.0 (ESP-12E Module)** and the right COM port.
3. Open `firmware/esp8266_l298n/esp8266_l298n.ino`. `WIFI_SSID` / `WIFI_PASS`
   are pre-filled for your router (`JioFiber-Kedar4G`) — change them if you move
   to a different network. Confirm `UDP_PORT` (4210) and `FAILSAFE_MS` (400)
   match `config.py`.
4. Upload. Open **Serial Monitor at 115200**. When it connects it prints its IP,
   e.g. `IP = 192.168.1.60`.
5. Put that IP into `config.py` → `ESP_IP`. Keep the phone, laptop, and ESP all
   on the **same router**.

### Bring-up procedure — do this in order, wheels UP first

> **Put the car up on a box so the wheels spin freely** until step 4. Have a
> hand on the e-stop (`SPACE`) and know that pulling laptop Wi-Fi also stops it.

1. **Teleop, wheels up.** `python tools\teleop.py` → press `e` to arm → tap
   `w/s/a/d`. Confirm forward is forward and turns go the right way. If a side
   spins backwards, set `DRIVE_INVERT_LEFT`/`DRIVE_INVERT_RIGHT` in `config.py`
   (no rewiring needed). If the wheels buzz but don't turn, raise `MIN_PWM` in
   the firmware and re-flash.
2. **Failsafe check, wheels up.** While driving in teleop, **close the window** —
   the wheels must stop within ~0.4 s. Repeat by pulling the laptop off Wi-Fi.
3. **Autonomy, wheels up.** `python main.py --drive`, point the camera around,
   press `a` to arm, and watch the wheels follow the on-screen command (open
   ground → both forward; obstacle in center → pivot; wall/person → stop).
4. **On the ground, low and clear.** Only now put it down, in an open area, with
   `DRIVE_MAX_SPEED_PCT` low (30–40). Arm, keep `SPACE` under your finger, and
   raise the cap gradually as you gain confidence.

### Drive controls (in `main.py --drive`)

| key | action |
|-----|--------|
| `a` | ARM / disarm the drive |
| `SPACE` | E-STOP (disarm immediately) |
| `p` | pause — also auto-disarms |
| `q` | quit — sends STOP + disarm |

The HUD shows a red **ARMED** / grey **DISARMED** badge and the live `L`/`R`
motor values.

### Drive tuning (`config.py` / firmware)

| Want | Change |
|------|--------|
| Slower/faster overall | `DRIVE_MAX_SPEED_PCT` (start low) |
| A side runs backwards | `DRIVE_INVERT_LEFT` / `DRIVE_INVERT_RIGHT` |
| Sharper vs gentler auto-turns | `DRIVE_TURN_SPEED_PCT`, `DRIVE_PIVOT_TURNS` |
| Stronger steering while going forward | `DRIVE_STEER_MIX` (0–1) |
| Wheels buzz but don't move | raise `MIN_PWM` in the firmware, re-flash |
| Car stutter-stops | ensure `ESP_SEND_HZ` × `FAILSAFE_MS` ≫ 1 (defaults are fine); check Wi-Fi |

---

## What's deliberately **not** here yet (later layers)

Layers 1 and 2 give you sound perception and safe, closed-loop driving. Still
deferred, on purpose: **visual odometry / SLAM and mapping**, **global A-to-B
path planning**, **reverse/recovery behaviours** when fully boxed in, and
**metric calibration** of distances. Add these only once the drive loop is
trusted at low speed — and keep the failsafe, arming, and speed cap in place as
you do.

