# Troubleshooting

Documented gotchas + fixes from development.

## Vehicle drifts when idle

**Symptom:** Spawned at (0,0,0.10), within seconds moves to (−5, 3, 0.10), keeps drifting.

**Cause:** Bullet Featherstone physics engine ignores `<velocity_decay>` and mishandles friction on articulated bodies. Initial settle impulse propagates forever.

**Fix:** Switch to DART. In `world.sdf`:
```xml
<plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics">
  <engine><filename>gz-physics-dartsim-plugin</filename></engine>
</plugin>
```

## Vehicle rockets to orbit on throttle

**Symptom:** Apply throttle, vehicle climbs to km altitude in seconds. Doesn't stop.

**Cause:** `MulticopterMotorModel` produces force = `motorConstant * ω²`. Tiny motorConstant changes produce 100× force differences. Plus LiftDrag lift = 0.5·ρ·v²·Cl·A — unbounded at speed.

**Fix:**
1. Disable MulticopterMotorModel force (keep for visual): `<motorConstant>0.0</motorConstant>`
2. Disable LiftDrag plugins (comment out)
3. Use bridge `EntityWrench` with bounded `THRUST_FWD_MAX = 5.0` N

## Vehicle flips on throttle

**Symptom:** Apply throttle, vehicle nose pitches up 90° or rolls sideways, crashes.

**Cause:** Motor reaction torque + low fuselage inertia + no stabilization. Once vehicle yaws/pitches, thrust direction changes → unstable.

**Fix:**
1. Set fuselage inertia ≥ 0.10 (not too low)
2. Bridge auto-level torques: `τ = -K·attitude - D·omega` with `K_RP=0.8, D_RP=0.40`
3. Firmware pitch damper: brake at >20° pitch

## "PreArm: GPS 1: not healthy"

**Symptom:** AP refuses to arm. `gps.is_healthy()` returns false even with fix_type=6, sats=16.

**Cause:** SITL JSON backend GPS has issues with AP's health check. Even valid GPS data flagged unhealthy.

**Fix:** `ARMING_SKIPCHK=4194303` bypasses all pre-arm checks. Standard arm cmd then succeeds.

## "PreArm: Gyro 0 rate 76Hz < loop rate*1.8 90Hz"

**Symptom:** Pre-arm fails on gyro rate.

**Cause:** Bridge UDP recv timeout (100ms) blocked send loop. Only 76 sensor packets/sec reached AP, but AP loop rate 50Hz expects ≥ 90Hz.

**Fix:**
1. Bridge: `rx.settimeout(0.001)` (near non-blocking)
2. Lower AP loop rate: `SCHED_LOOP_RATE=25` (threshold becomes 45 Hz)

## "Arm: AHRS: not using configured AHRS type"

**Symptom:** Even with `AHRS_EKF_TYPE=3` (EKF3), AP says not using configured type.

**Cause:** EKF3 has GPS/compass sanity checks fail with SITL JSON backend.

**Fix:** `AHRS_EKF_TYPE=10` — SITL fake AHRS, bypasses EKF entirely. Uses raw bridge attitude.

## ArduPlane dies after a few minutes

**Symptom:** `tail /tmp/ap.log` shows `!!! SITL parent pid XXX died, exit(1) !!!`. arduplane process gone.

**Cause:** arduplane binary detects parent process death and exits. Direct `nohup` doesn't provide stable parent.

**Fix:** Always launch via `sim_vehicle.py` wrapper, which provides stable parent process.

## Forwarder shows "Connection refused" repeatedly

**Symptom:** `tail /tmp/fwd.log` shows `[fwd] AP not ready: [Errno 61] Connection refused`.

**Cause:** ArduPlane died or never started. TCP 5760 not listening.

**Fix:** Check `ps aux | grep arduplane` — restart AP if dead.

## Vehicle moves but wrong direction

**Symptom:** Throttle up, vehicle moves opposite of expected (e.g., backward instead of forward).

**Cause:** Thrust direction sign mismatch between bridge body frame and vehicle nose convention. Vehicle "forward" = body `-Y` per fuselage layout (prop cage at `+Y` end).

**Fix:** Flip sign in bridge:
```python
fwd_x =  math.sin(yaw)    # or -math.sin(yaw)
fwd_y = -math.cos(yaw)    # or +math.cos(yaw)
```
Try both signs — vehicle yaw at startup may make either feel "wrong".

## Vehicle drifts after AP dies

**Symptom:** AP crashes, vehicle keeps moving for minutes, drifts kilometers.

**Cause:** Bridge keeps publishing last-known wrench based on stale PWM packet.

**Fix:** PWM watchdog in bridge — zero wrench if no PWM for >0.5s:
```python
if time.time() - last_pwm_time > 0.5 and ap_addr is not None:
    zw = EntityWrench()
    zw.entity.name = "paraglider_uav::fuselage"
    zw.entity.type = Entity.LINK
    pub_wrench.publish(zw)
```

## QGC virtual joystick doesn't move arms

**Symptom:** Stick in QGC moves vehicle but arms stay still.

**Cause:** Forwarder may strip RC override packets in UDP→TCP direction (mavlink version mismatch).

**Fix:**
1. Use TCP direct (kill forwarder first, connect pymavlink to `tcp:127.0.0.1:5760`)
2. OR enable arms via bridge — bridge reads PWM[0]/[1] directly and drives `/arm/left`, `/arm/right` topics regardless of firmware mixer

## Propeller appears to orbit instead of spin

**Symptom:** Prop visual sweeps a circle around joint origin instead of spinning in place.

**Cause:** Joint axis doesn't pass through prop hub. Link origin offset from mesh hub.

**Fix:** Set link pose = CAD-true hub center; visual `<pose>` = `-mesh_hub_natural` so mesh hub renders at link origin. Joint axis through link origin → spins in place:
```xml
<link name="propeller">
  <pose>0.050 0.13973 0.035 0 0 0</pose>   <!-- link origin = fuselage cage center -->
  <visual name="vis">
    <pose>-0.0735 -0.1398 -0.0305 0 0 0</pose>   <!-- shift mesh to land hub on origin -->
    <geometry><mesh><uri>...propeller_asm.stl</uri></mesh></geometry>
  </visual>
</link>
```

## Throttle doesn't work in LOITER

**Symptom:** Advance throttle stick in LOITER mode — motor doesn't spin.

**Cause:** ArduPlane LOITER uses TECS to auto-throttle based on altitude. Pilot stick ignored.

**Fix:** Custom firmware mixer override:
```cpp
// Always force pilot RC3 → k_throttle, in ANY mode
if (arming.is_armed_and_safety_off() && !failsafe.rc_failsafe) {
    RC_Channel *rc3 = rc().channel(2);
    if (rc3) {
        float thr = max(0, min(1, rc3->norm_input()));
        SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, thr * 100.0f);
    }
}
```

## gz sim crashes ("Service call to /gazebo/worlds timed out")

**Symptom:** `gz model -m paraglider_uav --pose` times out. gz process hung.

**Cause:** Physics blow-up — usually too-aggressive motorConstant or force magnitude.

**Fix:**
1. Kill all gz processes: `ps aux | grep "gz sim" | grep -v grep | awk '{print $2}' | xargs kill -9`
2. Reduce bridge `THRUST_FWD_MAX` (e.g. 5 → 2)
3. Reduce model.sdf motorConstant if motor plugin active
4. Restart from step 2 of LAUNCH.md

## Build errors in IDE diagnostics

**Symptom:** clang reports `'AP_HAL/AP_HAL.h' file not found` in `servos.cpp`.

**Cause:** IDE clang doesn't know ArduPilot's complex include paths.

**Fix:** Ignore. `./waf plane` build succeeds with full include resolution. The diagnostics are IDE noise.

## Bridge "Address already in use"

**Symptom:** Bridge restart fails with `OSError: [Errno 48] Address already in use`.

**Cause:** Old bridge still has UDP 9002 bound.

**Fix:**
```bash
for p in $(lsof -iUDP:9002 -t); do kill -9 $p; done
sleep 2
python3 -u ~/paraglider_sim/firmware/bridge/paraglider_bridge.py > /tmp/bridge.log 2>&1 &
```
