# RUNBOOK — running the UGV fully autonomous

Follow these in order. **Every step has a checkpoint — do not move to the next
step until the checkpoint passes.** Skipping ahead is how you end up with a car
that either does nothing or runs away.

Open PowerShell in the project folder and activate the environment first:

```bat
cd F:\Unmanned_vehicle
.venv\Scripts\activate
```

---

## Step 0 — Power up the hardware

1. Motor battery connected to the L298N `12V` and `GND` terminals.
2. NodeMCU powered (USB power bank is best) and its `GND` tied to L298N `GND`.
3. **Prop the car up on a box so all four wheels spin freely.** It stays up until
   Step 5.

**Checkpoint:** the L298N's little red power LED is on.

---

## Step 1 — Confirm the ESP is on the network

Open the **Arduino IDE → Tools → Serial Monitor at 115200** and reset the ESP.

**Checkpoint:** it prints

```
[esp] connected. IP = 172.21.81.7
[esp] listening for UDP on port 4210
```

- If it prints dots forever → wrong Wi-Fi name/password in the sketch.
- **If the IP is different from `172.21.81.7`, update `ESP_IP` in `config.py`.**
  Routers reassign addresses, so re-check this whenever the car has been off.

---

## Step 2 — Test the laptop → ESP link (nothing moves)

```bat
python tools\check_link.py
```

This is safe: it sends *armed but zero-speed* packets, so the ESP responds but
the wheels stay still. **Watch the small blue LED on the NodeMCU.**

**Checkpoint: the blue LED goes SOLID ON.**

| LED | Meaning | Fix |
|-----|---------|-----|
| **Solid on** | Packets are arriving. Link is good. | Go to Step 3. |
| **Off** | Packets are NOT arriving. | Same router for laptop+ESP; correct `ESP_IP`; allow Python through Windows Firewall; turn off router "AP isolation"/guest mode. |
| **Blinking** | ESP still joining Wi-Fi. | Check SSID/password in the sketch. |

The script also warns you if the laptop and ESP are on **different subnets**,
which is the most common silent cause of "nothing happens."

---

## Step 3 — Test the motors (wheels UP)

```bat
python tools\check_link.py --spin
```

It pulses: both forward → both reverse → pivot left → pivot right.

**Checkpoint: each motion matches its printed label.**

| Symptom | Fix |
|---------|-----|
| Nothing moves, LED was solid | Motor battery not connected / L298N power / OUT wiring |
| Buzzing, no rotation | Raise `MIN_PWM` (90 → 120) in the sketch, re-flash |
| One side spins backwards | Set `DRIVE_INVERT_LEFT` or `DRIVE_INVERT_RIGHT = True` in `config.py` |
| Only one side moves | That side's `ENA`/`ENB` or `OUT` wiring |

Optionally also try manual driving: `python tools\teleop.py`, press `e` to arm,
then `w`/`a`/`s`/`d`.

---

## Step 4 — Check the camera and perception (still wheels UP, no driving)

```bat
python main.py
```

Note: **no `--drive` here**, so nothing can move. You are only checking that it
*sees* correctly.

**First, calibrate the camera pose.** Do this before judging any decision, because
every distance the system reports is computed from `CAMERA_HEIGHT_M` and
`CAMERA_TILT_DEG`, and if they are wrong the numbers are wrong by 10–20× without any
error message. Startup prints the assumption:

```
[geom] assuming lens height 0.15 m, tilt 20 deg down, vfov 55 deg
[geom] predicted horizon row 34 of 384; ROI top row 84 (covers ~2.31 m to 0.14 m)
```

On the traversability panel, the **cyan dashed line must sit where the floor meets
the far wall**, and the grey ticks (0.5 / 1.0 / 1.5 / 2.0 m) must land at roughly
those distances — put a shoe at 1 m and check. If the line is too low, raise
`CAMERA_TILT_DEG`; too high, lower it. Measure the lens height with a ruler.

If you see `[geom] !! ROI starts ABOVE the horizon` or
`[trav] ROI_TOP_FRAC=... ABOVE the horizon`, **stop and fix the pose** — nothing
downstream is meaningful until you do.

**Checkpoint, all four:**

1. The **left panel shows a real live image** — not black. If it is black or you
   see the red **CAMERA BLIND** banner, fix the camera before going on
   (set `CAMERA_URL` in `config.py` to your phone stream and open that URL in a
   browser first to confirm it works).
2. Point at **open floor** → `CMD: FORWARD`. If instead you get
   `STOP (wall across path)` in open space, the pose is wrong — go back and
   calibrate. Do **not** tune thresholds to work around it.
3. Put a **box / your hand** in the middle, close → `CMD: LEFT` or `RIGHT`.
4. Point at a **wall**, or step in front → `CMD: STOP`.

If the decisions are wrong here, they will be wrong when driving. Tune with the
table in `README.md` ("Tuning") before continuing.

Press `q` to quit.

---

## Step 5 — Autonomous, wheels UP

This is the real thing, but with the car still on the box.

```bat
python main.py --drive
```

1. The HUD shows a grey **DISARMED** badge. Nothing moves yet — this is correct.
2. Click the video window so it has keyboard focus.
3. **Press `a` to ARM.** The badge turns **red ARMED**.

**Checkpoint:** the wheels now spin in the direction the HUD command says. Open
floor spins both forward; an obstacle in the centre pivots; a wall or a person
stops them.

> **This is almost certainly why the car did not move before: plain
> `python main.py` sends nothing, and even with `--drive` it starts DISARMED
> until you press `a`.**

Controls while running:

| key | action |
|-----|--------|
| `a` | ARM / disarm |
| `SPACE` | **E-STOP** — disarm immediately |
| `p` | pause (also auto-disarms) |
| `q` | quit (sends STOP + disarm) |

---

## Step 6 — Autonomous on the ground

Only after Step 5 is clean.

1. Set a low cap in `config.py`: `DRIVE_MAX_SPEED_PCT = 30`.
2. Put the car on the floor in an **open area**, nothing fragile nearby.
3. Tilt the camera **down 15–25°** so it sees the ground just ahead, and make
   sure the mount is rigid.
4. `python main.py --drive`, then press `a` to arm.
5. **Keep a finger on `SPACE`.** Walk beside the car.
6. Raise `DRIVE_MAX_SPEED_PCT` in steps of 5 only once it behaves.

**If anything looks wrong: press `SPACE`, or just close the laptop lid / kill the
window — the ESP stops the motors by itself within 0.4 s.**

---

## Quick reference

| Command | What it does | Can it move the car? |
|---------|--------------|----------------------|
| `python tools\check_link.py` | Tests the Wi-Fi link | No |
| `python tools\check_link.py --spin` | Pulses motors to verify wiring | Yes (wheels up!) |
| `python tools\teleop.py` | Manual keyboard driving | Yes (press `e`) |
| `python main.py` | Perception only | **No** |
| `python main.py --drive` | **Full autonomy** | Yes (press `a`) |

---

## If the car still will not move

Work down this list; each item is ruled out by the step above it.

1. Did you use `--drive`? Plain `main.py` transmits nothing at all.
2. Did you press `a`, and did the badge actually turn red **ARMED**? The video
   window must have keyboard focus — click it first.
3. Is `ESP_IP` in `config.py` the IP the Serial Monitor printed *this session*?
4. Does `check_link.py` make the LED go solid? If not, it is a network problem,
   not a code problem.
5. Is the motor battery connected and not flat? Measure it if you can.
6. Does `--spin` move the wheels? If no, it is wiring/power. If yes, autonomy
   will move them too.
7. Is the camera showing a real image, or is the **CAMERA BLIND** banner up? A
   blind camera forces STOP on purpose — the wheels will not turn.
8. Is the command actually a moving one? If the HUD says `STOP`, the vehicle is
   correctly obeying perception. Point it at open floor.
9. **Does the HUD say `STOP` with `wall across path (n%)` while you are looking at
   open floor?** Then perception is being lied to about where the ground is. Check
   the badge: `ARMED L=+0 R=+0` with `errors=0` means the link is fine and the brain
   is deliberately commanding zero — this is *not* a network or wiring fault, so
   items 1–6 do not apply. Look at the **cyan horizon line**: if it is not where the
   floor meets the far wall, `CAMERA_TILT_DEG` / `CAMERA_HEIGHT_M` do not match the
   real mount. Fix them (Step 4) and re-run `python tools\roi_check.py`.

> **How to tell the two failure families apart in one glance:**
> `sent=` climbing with `errors=0` and non-zero `L=`/`R=` → the laptop is commanding
> motion and the fault is downstream (network, wiring, power).
> `sent=` climbing with `errors=0` and `L=+0 R=+0` → the laptop is *choosing* not to
> move, and the fault is in perception or the camera pose.

