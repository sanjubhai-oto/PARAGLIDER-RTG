# Copter Pusher Firmware — torque feed-forward compensation

Custom ArduCopter build for the **MetroAir Hexa + X-axis pusher (EDF)** drone.
It adds **feed-forward cancellation of the disturbance moments produced by the
pusher propeller**, so the main rotors counter the pusher *before* an attitude
error develops — instead of the rate loop reacting late and saturating, which is
what destroyed the aircraft in crash log `00000170.BIN`.

```
firmware/copter_pusher/
├── ardupilot_patch/
│   ├── pusher_ff.patch      git-apply patch (2 files in libraries/AC_AttitudeControl)
│   └── MANUAL_APPLY.md      hand-apply instructions if the patch rejects
├── params/
│   └── metroair_pusher.param  full vehicle config + the new ATC_PUSH_* params
├── lua/
│   └── pusher_calib.lua     measures the disturbance so you can pick the gains
└── README.md               (this file)
```

---

## 1. Why this exists (the problem, from the logs)

Across four crash logs the pattern was identical: when the pusher spins up, the
airframe pitches **nose-down on its own** and the pilot/controller cannot stop
it. In `00000170.BIN` the attitude controller demanded the **maximum +75 °/s
nose-up** recovery and **saturated all six lift motors (~1950 µs)**, yet the
vehicle still pitched to **−84°** and dove from 22 m.

Cause: the pusher's thrust line is offset from the centre of gravity, so its
thrust produces a **moment** the stock multicopter controller has no model of.
It only reacts *after* the error appears — too late at high pusher thrust.

## 2. The model (from `torque_controlling_equations.pdf`)

A body **+X** pusher produces force and moment:

```
F_p   = [ T_p , 0 , 0 ]                       forward thrust
tau_p = [ sigma*Q_p , z_p*T_p , -h*T_p ]      roll , pitch , yaw
        \_______/   \______/   \_____/
        motor        vertical    Y-offset
        reaction     offset      of thrust
        torque       of thrust   line (h)
                     line (z_p)
```

`T_p` and `Q_p` scale with `Omega^2`, i.e. with **(pusher command)²**.

> Note: the PDF only writes the **roll** (`sigma*Q_p`) and **yaw** (`-h*T_p`)
> terms. The **pitch** term `z_p*T_p` — a *vertical* offset of the thrust line —
> is the one that actually crashed the aircraft, so it is included here as the
> primary gain (`ATC_PUSH_PIT`).

The fix is the PDF's feed-forward: the main rotors generate
`tau_main = tau_des − tau_p`, i.e. add **−tau_p** as feed-forward.

## 3. The implementation (theory → code)

Patched into `AC_AttitudeControl_Multi::rate_controller_run()`, right after the
rate PIDs set the motor demands:

```cpp
pusher_norm = (RC[ATC_PUSH_RC] mapped idle->0 , full->1)
p2 = pusher_norm * pusher_norm                 // ~ T_p, Q_p  (command^2)
set_roll ( get_roll()  + ATC_PUSH_RLL * p2 )   // cancels  sigma*Q_p
set_pitch( get_pitch() + ATC_PUSH_PIT * p2 )   // cancels  z_p*T_p   (the crash term)
set_yaw  ( get_yaw()   + ATC_PUSH_YAW * p2 )   // cancels  -h*T_p
```

Each gain folds in all the unknown physical constants (`z_p, h, sigma, kT, kQ`)
into one tunable number, calibrated empirically. **All gains default to 0 and
`ATC_PUSH_EN` defaults to 0 → the firmware is bit-for-bit stock behaviour until
you deliberately turn it on.** Read-modify-write (`get_pitch()`+ff→`set_pitch()`)
is used so the patch is robust to minor version differences in the rate-PID call.

### New parameters

| Param | Default | Meaning |
|-------|---------|---------|
| `ATC_PUSH_EN`  | 0 | Master enable (0 = stock behaviour) |
| `ATC_PUSH_RC`  | 8 | RC channel that drives the pusher |
| `ATC_PUSH_PIT` | 0 | Pitch FF gain — **positive commands nose-up** |
| `ATC_PUSH_RLL` | 0 | Roll FF gain |
| `ATC_PUSH_YAW` | 0 | Yaw FF gain |

## 4. Build for MicoAir743v2

```bash
# 1. Get the source (matched to the board's running version, e.g. 4.6)
git clone --recurse-submodules -b Copter-4.6 https://github.com/ArduPilot/ardupilot.git
cd ardupilot

# 2. Apply the compensation
git apply /path/to/firmware/copter_pusher/ardupilot_patch/pusher_ff.patch
#   …if it rejects, follow ardupilot_patch/MANUAL_APPLY.md (only 2 files)

# 3. Verify it's in
grep -c "Pusher (X-axis propulsion) torque feed-forward" \
    libraries/AC_AttitudeControl/AC_AttitudeControl_Multi.cpp   # -> 1

# 4. Build + flash
./waf configure --board MicoAir743v2
./waf copter
./waf --upload          # or flash build/MicoAir743v2/bin/arducopter.apj via Mission Planner
```

Optional SITL sanity check before flashing hardware:

```bash
./waf configure --board sitl && ./waf copter
Tools/autotest/sim_vehicle.py -v ArduCopter -f hexa --console --map
# set ATC_PUSH_EN 1 / ATC_PUSH_PIT 0.05 and confirm a pitch bias appears with RC8
```

## 5. Load the config

Load `params/metroair_pusher.param` (Mission Planner → Full Parameter List →
Load from file → Write). It also moves the **flight-mode switch off RC8 to RC7**
and enables the **battery failsafes** — both contributed to the crashes.

## 6. Calibration procedure (READ THIS — safety critical)

The compensation term is **new and untested on your airframe**. Bring it up
incrementally; never start with a guessed non-zero gain.

1. **Mechanical first.** The best fix is still to put the EDF thrust line through
   the CG height (torque PDF §7). The feed-forward is for the *residual* offset.
   Until the pitch gain is calibrated, **do not run the pusher above ~50%**.

2. **Flight 0 — prove it's stock.** With `ATC_PUSH_EN=0`, fly a normal hover in
   ALT_HOLD. Behaviour must be unchanged. (Move mode switch to RC7 first.)

3. **Measure the disturbance.** Put `lua/pusher_calib.lua` in `APM/scripts/`,
   set `SCR_ENABLE=1`. Fly ALT_HOLD, **keep the pitch stick neutral**, and walk
   the pusher up in steps (10–20–30–40–50%). After landing read the `PUSH-CAL`
   table on the GCS: it shows mean pitch (and roll, and yaw-rate) per pusher
   band. A negative mean pitch at higher pusher = the nose-down disturbance.

4. **Tune `ATC_PUSH_PIT`.** Set `ATC_PUSH_EN=1`. Start `ATC_PUSH_PIT=0.02`. Fly
   the same stepped-pusher test. If the nose still drops, increase in 0.01–0.02
   steps; if it now pitches *up*, you overshot (or flip the sign). Goal: mean
   pitch stays ≈ 0 as pusher rises, pitch stick neutral. Keep pusher ≤50% until
   this holds, then extend the range.

5. **Trim roll / yaw.** With pitch handled, repeat to null any residual roll
   (`ATC_PUSH_RLL`) and yaw drift (`ATC_PUSH_YAW`).

6. **Sanity limit.** If any |gain| needs to exceed ~0.3 to null the disturbance,
   the mechanical offset is too large — fix the mount, don't mask it in software.

## 7. What this does and does NOT change

- ✅ Adds pusher disturbance feed-forward (pitch/roll/yaw), parameter-gated, off
  by default.
- ✅ Pusher stays an **independent forward-thrust output** (`SERVO7=58`
  passthrough). With the mode switch on RC7 and flying ALT_HOLD, *pitch-neutral +
  pusher slider = move forward* — the behaviour `firmware_new_RQRMENT.pdf` asks
  for, now without the nose-down runaway.
- ❌ Does not touch the rate/attitude PIDs, the mixer geometry, or any other mode.
- ❌ Does not replace the mechanical fix; it complements it.
