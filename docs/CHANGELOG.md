# Development Timeline

Chronological notes from the project. Lessons learned + key milestones.

## Phase 1: Physics Engine

**Problem:** Vehicle drifted 0.4 m/s sideways at idle. Spent hours tuning friction, mass, inertia — no fix.

**Investigation:**
- Tried thicker ground (box vs plane) — no help
- Disabled motor plugin — no help
- Disabled LiftDrag — no help
- Removed arm/prop collision boxes — no help
- Increased mass, inertia, damping — no help

**Root cause:** Bullet Featherstone physics engine. Ignores `<velocity_decay>` (ODE-only) and friction misapplies on articulated bodies.

**Fix:** Single line change in `world.sdf`:
```diff
- <engine><filename>gz-physics-bullet-featherstone-plugin</filename></engine>
+ <engine><filename>gz-physics-dartsim-plugin</filename></engine>
```

Drift fully gone. Pose locked to 1e-6 m precision over 30s.

## Phase 2: Firmware Mixer

Custom paraglider mixer in `~/ardupilot/ArduPlane/servos.cpp`:
- Reads raw RC (bypasses attitude controller)
- Maps roll → differential brake, pitch → both brakes
- Outputs to `k_flaperon_left`, `k_flaperon_right`
- Throttle override forces RC3 → motor in ANY mode (bypasses LOITER TECS)

## Phase 3: Pre-Arm Bypasses

Many pre-arm checks fail with SITL JSON backend:
- GPS not healthy (despite fix=6, sats=16)
- Gyros inconsistent (multi-IMU disagreement)
- AHRS type mismatch
- Gyro rate too low

Solutions:
- `ARMING_SKIPCHK=4194303` (bypass all)
- `INS_USE2=0, INS_USE3=0` (single IMU)
- `COMPASS_USE=0`
- `AHRS_EKF_TYPE=10` (SITL fake AHRS)
- `SCHED_LOOP_RATE=25` + bridge non-blocking recv (gyro rate met)
- `BRD_SAFETYENABLE=0`

## Phase 4: Bridge Bandwidth

Bridge UDP recv timeout was 100ms blocking send loop to 76 Hz. AP gyro check requires ≥ 90 Hz.

**Fix:** `rx.settimeout(0.001)` near non-blocking. Bridge now sends 250 Hz independent of PWM packet rate.

## Phase 5: Motor Model Battle

**Problem:** `MulticopterMotorModel` force = `motorConstant·ω²`. Tiny constant changes produce 100× force differences.

Tuning attempts:
| motorConstant | Trajectory |
|---|---|
| 0.0025 (T=1.6N target) | Barely moves |
| 0.025 | 5m forward in 5s |
| 0.05 | 22m in 4s, peak 0.64m alt |
| 0.1 | 18 m/s avg, 93m flight |
| 0.5 | Sim blows up, gz hangs |

Plus `rotorVelocitySlowdownSim=10` was scaling thrust 100× down silently. Set to 1.

Plus prop joint axis direction mattered for force application convention. Tried (0,1,0) and (0,-1,0).

**Final solution:** Replaced entirely with **bounded thrust via bridge ApplyLinkWrench**. Linear in throttle, capped at `THRUST_FWD_MAX`. Predictable, tunable.

## Phase 6: Lift Battle

LiftDrag plugins computed unbounded lift at speed. Force = 0.5·ρ·v²·Cl·A. At v=15 m/s with full area 1.8 m²: lift = 111 N (vehicle weight 5 N). Vehicle rocketed.

AI Engineer research recommendations:
- Wing loading 1-3 kg/m² (we had 0.28)
- `a0=-0.14` (high-camber fabric)
- `alpha_stall=0.16` (ram-air stalls early)
- Area per panel 0.36 → 0.10

Iterative testing: vehicle reached peak altitudes 3-9m but always descended back (phugoid). Real paraglider has pendulum stability (canopy above pilot). Tried:
- Canopy z=1.5m + fixed riser → thrust suppressed, vehicle stuck
- Canopy z=1.5m + ball riser → ODE assertion failure, sim crash
- Canopy z=1.5m + universal riser → tumbles onto side
- Canopy with built-in pitch incidence → 5°/10° tested, no lift improvement

**Final solution:** Disabled all LiftDrag plugins. Bridge applies virtual lift = vehicle weight (canopy always inflated) + variable up from throttle/pitch.

## Phase 7: Tumbling Battle

Even with bounded forces, vehicle yaws on motor start (reaction torque) then tumbles. Auto-level needed.

**Solution:** Bridge applies world-frame restoring torques:
```python
torque_x = -K_RP * roll  - D_RP * omega_x
torque_y = -K_RP * pitch - D_RP * omega_y
```

Gains tuned: K_RP=0.8, D_RP=0.40. Vehicle stays level under thrust.

Firmware pitch damper added redundantly: brake arms at >20° pitch.

## Phase 8: PWM Watchdog

User reported vehicle "kept flying after I disconnected QGC." Vehicle drifted 438m on stale wrench.

**Cause:** Bridge published last-known wrench based on stale PWM after AP died.

**Fix:**
```python
if time.time() - last_pwm_time > 0.5 and ap_addr is not None:
    pub_wrench.publish(EntityWrench(entity=..., wrench=0))  # zero forces
```

Vehicle now freezes when AP dies. Safe.

## Phase 9: Agent Framework

Installed [msitarzewski/agency-agents](https://github.com/msitarzewski/agency-agents) (184 agents) at `~/.claude/agents/`.

Used parallel sub-agents for:
- **Embedded Firmware Engineer** — applied 4 safety patches (failsafe gates, saturation fix)
- **Rapid Prototyper** — 10-iteration parameter sweep
- **AI Engineer** — researched paragliding physics from published sources
- **Software Architect** — designed RC→force/torque control mapping
- **Reality Checker** — independent validation

## Final State

**Working:**
- Vehicle moves forward on throttle (bounded ~5 m/s ground roll)
- Arms respond to RC sticks (direct visual + flight control)
- Auto-level prevents tumbling
- Bounded thrust (no rocket-to-orbit)
- PWM watchdog (vehicle freezes if AP dies)
- Custom firmware compiles + flashes

**Pending (out of scope):**
- Sustained level cruise (vehicle climbs then descends, no altitude hold)
- Active TECS-style pitch+throttle controller (requires firmware PID)
- Real canopy pendulum dynamics (canopy mass + LiftDrag CP geometry need redesign)
- Full QGC autopilot mission (RTL, waypoints) — works in principle but not stress-tested

## Lessons Learned

1. **Open-loop physics is fundamentally unstable for flying vehicles.** Either stalls or rockets. Active control needed.
2. **gz plugins have hidden gotchas.** MulticopterMotorModel `rotorVelocitySlowdownSim` silently scaled thrust 100× down. LiftDrag has unbounded force.
3. **DART >> Bullet Featherstone for articulated bodies.** One-line change saved days of debugging.
4. **Virtual physics beats real physics for prototyping.** Bridge ApplyLinkWrench with constant gravity-cancel + bounded thrust = predictable, tunable, no rocketing.
5. **Direct RC reading in firmware bypasses AP attitude controller.** Necessary for paramotor where stock fixed-wing mixer doesn't apply.
6. **PWM watchdog is essential for any sim that simulates control authority.** Without it, vehicle drifts on AP crash.
7. **Pre-arm checks are unbypassable without `ARMING_SKIPCHK`** — `ARMING_CHECK` was renamed in modern AP.
8. **Parallel agents save time but can fail when env-detection wrong.** Some agents refused to run on macOS thinking it lacked toolchain.

## Files Modified

| File | Purpose |
|------|---------|
| `~/paraglider_sim/sdf/world.sdf` | DART physics, ApplyLinkWrench plugin |
| `~/paraglider_sim/sdf/model.sdf` | Geometry, mass, joints, disabled LiftDrag |
| `~/paraglider_sim/firmware/bridge/paraglider_bridge.py` | Custom physics + control + watchdog |
| `~/paraglider_sim/firmware/scripts/mav_forwarder.py` | TCP↔UDP MAVLink forwarder |
| `~/paraglider_sim/firmware/params/paraglider.parm` | RC + servo + pre-arm config |
| `~/ardupilot/ArduPlane/servos.cpp` | Custom paraglider mixer + throttle override |

## Acknowledgments

- ArduPilot project (GPLv3 firmware)
- Gazebo simulator
- AI agency framework: [msitarzewski/agency-agents](https://github.com/msitarzewski/agency-agents)
- AI Engineer research sources: NPS Snowflake parafoil, Cal Poly paraglider thesis, PX4 Paramotor Project #2 (Junwoo Hwang)
