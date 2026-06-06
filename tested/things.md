# Bot2 Pool Test Guide (No PC Connected)

Use this after flashing **Bot2TOP** + **Bot2Bottom**. You will only have the **top LCD** and **gamepad** in the pool — no serial monitor.

---

## Before you leave the bench

- [ ] Flash **Bot2TOP** (top Teensy)
- [ ] Flash **Bot2Bottom** (bottom Teensy)
- [ ] Wait **~8 seconds** after bottom power-on (ESC startup) before thrusters
- [ ] Gamepad connects (LCD line 2 may show `Dev ID xxxx:xxxx` briefly at plug-in)
- [ ] Tether connected, ROV in water at surface
- [ ] Camera unplugged (`CAMERA_TILT_ENABLED 0` — expected)
- [ ] Bring a phone or notepad to record results

---

## LCD layout (what to look at)

| Line | Label | What it means |
|------|-------|----------------|
| **0** | `Slow Mode:ON/OFF` | Precision mode (LB+RB toggles) |
| **1** | `Battery  xx.x V` | Battery voltage from bottom |
| **2** | `Depth    xx.xx Feet` | Depth display (offset applied) |
| **3** | `PID: ±x.xx ft ON/OFF` | Depth-hold error and status |

**Line 2** = what you use for depth calibration.  
**Line 3** = what you use for PID tuning.  
**Lines 0–3** update whenever telemetry is received — if they freeze, comms or bottom power is the problem, not a tuning constant.

---

## Gamepad reference

| Input | Function |
|-------|----------|
| Left stick Y | Forward / reverse |
| Left stick X | Strafe left / right |
| Right stick X | Steer (rotate) |
| Right stick Y | Dive / surface (manual) |
| LT / RT | Claw open / close |
| D-pad L/R | LED dim |
| **LB + RB** (together) | Toggle **Slow Mode** |

---

## Test order (do in this sequence)

1. Comms + telemetry  
2. Motor directions (each axis once)  
3. Drive straight + pitch trim  
4. Strafe  
5. Claw  
6. Depth display  
7. Depth hold (PID)  
8. Slow mode  

Stop and fix anything marked **FAIL** before moving on.

---

## Test 1 — Comms and telemetry

**How:** Power both boards. Leave sticks centered 10 seconds.

**Look at LCD:**
- Line 1 battery voltage updates (roughly 10–16 V depending on pack)
- Line 2 depth changes when you move ROV up/down in the water
- Line 0 shows `Slow Mode:OFF`

| Pass | Fail |
|------|------|
| Battery and depth update live | Lines frozen or blank |
| Depth rises when you push ROV deeper | Depth never changes |

**If FAIL:** Check tether, bottom power, RS-485 — not a gain constant issue.

---

## Test 2 — Motor directions (quick stick check)

**How:** One axis at a time, **short** stick taps in slow mode if available.

Toggle slow mode first: **LB + RB** → line 0 should say `Slow Mode:ON`.

| Stick | ROV should |
|-------|------------|
| Left Y forward | Move forward |
| Left Y back | Move reverse |
| Left X left | Strafe left |
| Left X right | Strafe right |
| Right X left | Rotate left |
| Right X right | Rotate right |
| Right Y up | Surface (rise) |
| Right Y down | Dive (sink) |

**Record any reversed axis:**

```
Reversed: _______________________
```

**Fix later in code:** `motDirs` or `*MotDir` defines in Bot2TOP — not covered in this pool session.

---

## Test 3 — Drive straight and pitch trim

**Constant:** `DRIVE_PITCH_TRIM` (currently **-0.12**)

**How:**
1. `Slow Mode:OFF`
2. Center tether as best you can (zip-tied to middle)
3. From still water, push **left stick forward** to ~50%, hold 3–5 seconds
4. Watch ROV attitude (level vs nose-up vs nose-down)
5. Repeat at ~80% forward briefly

**Look at:**
- Does the **front/claw rise** out of the water? (nose-up)
- Does the **front dive** under? (nose-down)
- Does ROV **drift left/right** with stick centered forward? (separate trim — Test 3b)

| What you see | Meaning | Change next flash |
|--------------|---------|-------------------|
| Nose **still rises** on forward | Not enough down-trim | `DRIVE_PITCH_TRIM` **-0.16** or **-0.18** |
| Nose **dives** on forward | Too much down-trim | `DRIVE_PITCH_TRIM` **-0.08** or **-0.06** |
| Nose **rises on reverse** | Trim sign wrong | Flip to **+0.12** |
| Stays ** fairly level** | Good | Leave **-0.12** |

**Write down:**

```
Forward 50%:  nose up / level / nose down
Forward 80%:  nose up / level / nose down
DRIVE_PITCH_TRIM to try next: _______
```

### Test 3b — Drive straight (left/right drift)

**Constants:** `DriveGainL`, `DriveGainR` (both **0.60**)

**How:** Forward at 50%, tether centered, no steer input.

| What you see | Change next flash |
|--------------|-------------------|
| Drifts **right** | `DriveGainL` **0.62**, `DriveGainR` **0.58** |
| Drifts **left** | `DriveGainL` **0.58**, `DriveGainR` **0.62** |
| Straight | Leave both **0.60** |

```
Drift: none / left / right
DriveGainL / DriveGainR to try: _______ / _______
```

---

## Test 4 — Strafe strength

**Constant:** `StrafeGain` (currently **0.58**)

**How:**
1. Hold depth roughly constant with right stick if needed
2. Left stick **full left** 2 sec, then **full right** 2 sec
3. Compare feel to forward drive (left Y)

**Look at:**
- Does it slide sideways confidently?
- Does it feel weak compared to forward?

| What you see | Change next flash |
|--------------|-------------------|
| Still **weak** | `StrafeGain` **0.62** or **0.65** |
| **Too twitchy** sideways | `StrafeGain` **0.55** |
| Feels **balanced** | Leave **0.58** |

```
Strafe feel: weak / good / too strong
StrafeGain to try next: _______
```

---

## Test 5 — Steering feel

**Constant:** `SteerGain` (currently **0.60**)

**How:** Right stick X left/right at 50% while barely moving or stationary.

| What you see | Change next flash |
|--------------|-------------------|
| Spins **too fast** | `SteerGain` **0.50** |
| **Sluggish** turn | `SteerGain` **0.65** |
| Comfortable | Leave **0.60** |

---

## Test 6 — Claw

**Constants:** `GripperGain` **0.35**, bottom slew limits (no LCD for claw)

**How:**
1. RT squeeze → claw closes
2. LT squeeze → claw opens
3. Repeat 3–4 times

**Look at:**
- Smooth motion vs jerky
- **Teensy reboot** or ROV resets (brownout)

| What you see | Change next flash |
|--------------|-------------------|
| Reboots on claw | `GripperGain` **0.28** (top) |
| Too weak | `GripperGain` **0.40** (careful) |
| Works smoothly | Leave **0.35** |

```
Claw: OK / weak / reboots
GripperGain to try: _______
```

---

## Test 7 — Depth display calibration

**Constants:**
- `DEPTH_DISPLAY_OFFSET_FT` (**42.7**)
- `DEPTH_FEET_PER_VOLT` (**90.0**)
- `DEPTH_SURFACE_VOLT` (**0.0**)

**How:**

### 7a — Surface
ROV at surface, hold still 10 sec.

**Look at line 2:** `Depth    xx.xx Feet`

| Pass | Target |
|------|--------|
| Reads about **0.0 ± 0.5 ft** | Good offset |
| Reads **-1.3** (example) | Adjust offset |

**Offset rule:**  
New offset = old offset + (reading at surface)  
Example: reads **-1.3** → `42.7 + (-1.3)` = **41.4**

### 7b — Known depth
Hold ROV at ~**5 ft** (estimate OK).

**Look at line 2** vs true depth.

| Example | Fix |
|---------|-----|
| LCD **4.5** at true 5 ft (~10% low) | `DEPTH_FEET_PER_VOLT` **99** (90 × 5/4.5) |
| LCD **5.5** at true 5 ft (~10% high) | `DEPTH_FEET_PER_VOLT` **82** |

```
Surface depth LCD: _______ ft  →  DEPTH_DISPLAY_OFFSET_FT: _______
5 ft depth LCD:    _______ ft  →  DEPTH_FEET_PER_VOLT: _______
```

---

## Test 8 — Depth hold (PID)

**Constants (Bot2TOP):**

| Constant | Current | Controls |
|----------|---------|----------|
| `DEPTH_HOLD_ACTIVATION_DELAY_MS` | 750 | Wait after releasing stick before hold |
| `DEPTH_HOLD_DEADBAND` | 0.35 ft | “Close enough” zone |
| `MAX_VERTICAL_PID_OUTPUT` | 0.18 | Max auto vertical power |
| `DEPTH_KP` | 0.20 | Correction strength |
| `DEPTH_KI` | 0.005 | Steady-state fix |
| `DEPTH_KD` | 0.14 | Damping |
| `DEPTH_PID_SLEW_RATE` | 1.0 | Smoothness of hold pulses |
| `DEPTH_PID_OUTPUT_SIGN` | -1 | Direction (flip if wrong) |
| `VERTICAL_DEADBAND` | 0.05 | Stick must be centered for hold |

### 8a — Enable hold

**How:**
1. Drive to **~3–5 ft** manually (right stick Y)
2. **Center** right stick Y completely
3. Hold still **~1 second**
4. Watch **line 3**

**Look for:** `PID: +0.xx ft ON` (error small, **ON**)

| What you see | Meaning |
|--------------|---------|
| Stays **OFF** | Stick not centered, bad telemetry, or moved stick too soon |
| Shows **ON** | Hold engaged |

### 8b — Hold quality

**How:** With **ON**, hands off right stick 15–20 seconds. Optionally nudge ROV slightly up/down with stick, release, wait for **ON** again.

**Look at:**
- Line 3 error (`±x.xx ft`) — should stay roughly **under 0.5 ft**
- ROV body — gentle corrections vs violent bobbing
- Racing to surface or bottom

| What you see | Change next flash |
|--------------|-------------------|
| **Races** up or down | Flip `DEPTH_PID_OUTPUT_SIGN` to **+1.0** OR lower `DEPTH_KP` to **0.15** |
| **Bobbing** / pulsing | Lower `MAX_VERTICAL_PID_OUTPUT` **0.14**, `DEPTH_PID_SLEW_RATE` **0.7** |
| **Sluggish** return to depth | Raise `DEPTH_KP` **0.25** slightly |
| **Hunting** (overshoots back and forth) | Lower `DEPTH_KI` **0.003**, raise `DEPTH_KD` **0.18** |
| **Gentle hold** | Leave as-is |

### 8c — Manual override

**How:** While **ON**, move right stick Y deliberately.

**Look for:** Line 3 → **OFF** immediately; manual control returns.

```
PID hold: ON works? yes / no
Hold quality: good / bobbing / races up / races down
Line 3 error typical: _______ ft
Changes to try: _________________________________
```

---

## Test 9 — Slow mode

**Constant:** `SLOW_MODE_SCALE` (**0.40**)

**How:** **LB + RB** → line 0 `Slow Mode:ON`. Repeat forward, strafe, dive briefly.

**Look for:** All motion ~40% of normal — easier fine control.

Toggle off: **LB + RB** again → `Slow Mode:OFF`.

---

## Quick reference — all tunable values (current)

### Driving (Bot2TOP)

```cpp
float DriveGainL  = 0.60;
float DriveGainR  = 0.60;
float SteerGain   = 0.60;
float DiveGainL   = 0.60;
float DiveGainR   = 0.60;
float StrafeGain  = 0.58;
#define DRIVE_PITCH_TRIM (-0.12f)
float GripperGain = 0.35;
```

### Depth display

```cpp
#define DEPTH_SURFACE_VOLT 0.0f
#define DEPTH_FEET_PER_VOLT 90.0f
#define DEPTH_DISPLAY_OFFSET_FT 42.7f
```

### Depth hold PID

```cpp
#define DEPTH_HOLD_DEADBAND 0.35f
#define MAX_VERTICAL_PID_OUTPUT 0.18f
#define DEPTH_KP 0.20f
#define DEPTH_KI 0.005f
#define DEPTH_KD 0.14f
#define DEPTH_PID_SLEW_RATE 1.0f
#define DEPTH_PID_OUTPUT_SIGN (-1.0f)
#define VERTICAL_DEADBAND 0.05f
#define DEPTH_HOLD_ACTIVATION_DELAY_MS 750
```

---

## One-page field log (copy to phone)

```
Date: ___________  Pool: ___________

[ ] Telemetry live (battery + depth update)
[ ] All stick directions correct

Pitch trim (forward 50%):  up / level / down  → DRIVE_PITCH_TRIM: _______
Drift forward:             none / L / R        → DriveGainL/R: _______

Strafe:                    weak / OK / strong → StrafeGain: _______
Steer:                     fast / OK / slow   → SteerGain: _______
Claw:                      OK / weak / reboot → GripperGain: _______

Surface depth LCD: _______ ft  → OFFSET: _______
~5 ft depth LCD:   _______ ft  → FEET_PER_VOLT: _______

PID hold ON:               yes / no
Hold behavior:             good / bob / race up / race down
PID changes to try: _________________________________

Notes:
_____________________________________________________
_____________________________________________________
```

---

## Safety reminders

- Keep **neutral sticks** if anything runs away; release → thrusters should stop commanded motion
- **Claw + full forward + full vertical** together draws the most current — test axes separately first
- If bottom **reboots** (claw or hard maneuver), reduce `GripperGain` or `DriveGain` before continuing
- Depth hold is a **assist**, not autopilot — stay ready to take manual control

---

## Files to flash

| Board | Sketch |
|-------|--------|
| Top | `ONLY PUT WORKING CODE IN HERE/Bot2TOP/Bot2TOP.ino` |
| Bottom | `ONLY PUT WORKING CODE IN HERE/Bot2Bottom/Bot2Bottom.ino` |

Only **Bot2TOP** needs reflashing for drive/PID/depth tuning changes. Reflash **Bot2Bottom** only if claw or telemetry code changed.
