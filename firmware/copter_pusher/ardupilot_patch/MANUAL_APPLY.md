# Manual application of the pusher feed-forward compensation

If `git apply pusher_ff.patch` reports rejects (line numbers drift between
ArduCopter point releases), apply these three edits by hand. They only touch
**two files**, both in `libraries/AC_AttitudeControl/`.

The logic uses read-modify-write (`_motors.get_pitch()` + ff -> `set_pitch()`),
so it does **not** depend on the exact rate-PID call signature of your version.

---

## Edit 1 — `AC_AttitudeControl_Multi.h`

Find the member variable `_thr_mix_man` and add the pusher members right after it:

```cpp
    AP_Float                _thr_mix_man;

    // ---- Pusher (X-axis propulsion) torque feed-forward compensation ----
    AP_Int8                 _push_en;     // enable
    AP_Int8                 _push_rc;     // RC input channel driving the pusher (1..16)
    AP_Float                _push_pit;    // pitch FF gain (normalised per pusher_norm^2)
    AP_Float                _push_rll;    // roll  FF gain
    AP_Float                _push_yaw;    // yaw   FF gain
```

---

## Edit 2 — `AC_AttitudeControl_Multi.cpp`  (includes)

Near the top with the other `#include` lines, add:

```cpp
#include <RC_Channel/RC_Channel.h>
```

---

## Edit 3 — `AC_AttitudeControl_Multi.cpp`  (parameter table)

In `const AP_Param::GroupInfo AC_AttitudeControl_Multi::var_info[]`, find the
`THR_MIX_MAN` entry and the trailing `AP_GROUPEND`. Insert these five entries
**before** `AP_GROUPEND`:

```cpp
    // @Param: PUSH_EN
    // @DisplayName: Pusher torque compensation enable
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO("PUSH_EN", 30, AC_AttitudeControl_Multi, _push_en, 0),

    // @Param: PUSH_RC
    // @DisplayName: Pusher RC input channel
    // @Range: 1 16
    // @User: Advanced
    AP_GROUPINFO("PUSH_RC", 31, AC_AttitudeControl_Multi, _push_rc, 8),

    // @Param: PUSH_PIT
    // @DisplayName: Pusher pitch feed-forward gain
    // @Range: -0.5 0.5
    // @Increment: 0.005
    // @User: Advanced
    AP_GROUPINFO("PUSH_PIT", 32, AC_AttitudeControl_Multi, _push_pit, 0.0f),

    // @Param: PUSH_RLL
    // @DisplayName: Pusher roll feed-forward gain
    // @Range: -0.5 0.5
    // @Increment: 0.005
    // @User: Advanced
    AP_GROUPINFO("PUSH_RLL", 33, AC_AttitudeControl_Multi, _push_rll, 0.0f),

    // @Param: PUSH_YAW
    // @DisplayName: Pusher yaw feed-forward gain
    // @Range: -0.5 0.5
    // @Increment: 0.005
    // @User: Advanced
    AP_GROUPINFO("PUSH_YAW", 34, AC_AttitudeControl_Multi, _push_yaw, 0.0f),
```

> **Index check:** indices 30-34 must be unique within this table. Verify:
> ```bash
> grep -oE 'AP_GROUPINFO\("[A-Z_]+", [0-9]+' libraries/AC_AttitudeControl/AC_AttitudeControl_Multi.cpp
> ```
> If 30-34 are taken, renumber to any free values (the number is just a storage
> key; the param name `ATC_PUSH_*` is unchanged).

---

## Edit 4 — `AC_AttitudeControl_Multi.cpp`  (the compensation, in `rate_controller_run()`)

At the **end** of `void AC_AttitudeControl_Multi::rate_controller_run()`, insert
this block **just before** `control_monitor_update();`:

```cpp
    // ---- Pusher (X-axis propulsion) torque feed-forward compensation ----
    // Main rotors generate tau_main = tau_des - tau_p, i.e. add -tau_p as
    // feed-forward so the pusher disturbance is cancelled BEFORE an attitude
    // error develops. T_p,Q_p ~ (pusher command)^2. Gains fold in z_p,h,sigma,
    // kT,kQ and are calibrated on the bench/in flight. Default 0 = stock.
    if (_push_en) {
        float pusher_norm = 0.0f;
        if (rc().has_valid_input()) {
            const RC_Channel *pch = rc().channel(constrain_int16(_push_rc, 1, 16) - 1);
            if (pch != nullptr) {
                pusher_norm = (pch->norm_input_ignore_trim() + 1.0f) * 0.5f; // idle->0, full->1
            }
        }
        pusher_norm = constrain_float(pusher_norm, 0.0f, 1.0f);
        const float p2 = pusher_norm * pusher_norm;
        _motors.set_roll (_motors.get_roll()  + _push_rll * p2);
        _motors.set_pitch(_motors.get_pitch() + _push_pit * p2);
        _motors.set_yaw  (_motors.get_yaw()   + _push_yaw * p2);
    }
```

---

## Verify it compiled in

```bash
grep -c "Pusher (X-axis propulsion) torque feed-forward" \
    libraries/AC_AttitudeControl/AC_AttitudeControl_Multi.cpp
# expect: 1   (the rate_controller_run block; the var_info comment may add more)
```

After flashing, the params `ATC_PUSH_EN, ATC_PUSH_RC, ATC_PUSH_PIT,
ATC_PUSH_RLL, ATC_PUSH_YAW` will appear in Mission Planner.
