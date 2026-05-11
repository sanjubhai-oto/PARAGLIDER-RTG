# Physics Model (`sdf/model.sdf` and `sdf/world.sdf`)

## Vehicle Composition

Total mass ~0.38 kg (paramotor scale).

| Link | Mass (kg) | Gravity | Notes |
|------|-----------|---------|-------|
| `fuselage` | 0.20 | yes | Main body, IMU sensor, pilot cage. Inertia 0.10 kg·m² diag. |
| `arm_left` | 0.03 | yes | Brake-line arm, mesh on left of fuselage |
| `arm_right` | 0.03 | yes | Brake-line arm, mesh on right |
| `propeller` | 0.10 | yes | Pusher prop. Visual only — motorConstant=0, no thrust force |
| `canopy` | 0.02 | **false** | Ram-air canopy. `<gravity>false</gravity>` simulates buoyant inflated wing |

Effective weight = (0.20 + 0.06 + 0.10) × 9.81 = **3.53 N** (canopy excluded from gravity).

## Joints

| Joint | Type | Parent → Child | Axis | Limits | Damping | Purpose |
|-------|------|----------------|------|--------|---------|---------|
| `arm_left_joint` | revolute | fuselage → arm_left | (0, -1, 0) | 0..0.6 rad | 1.0 | Pull command rotates arm tip DOWN |
| `arm_right_joint` | revolute | fuselage → arm_right | (0, 1, 0) | 0..0.6 rad | 1.0 | Mirror axis vs left (same "pull" direction visually) |
| `propeller_joint` | revolute | fuselage → propeller | (0, 1, 0) | none | 0.005 | Prop spins around hub center |
| `riser_joint` | **fixed** | fuselage → canopy | — | — | — | Canopy rigidly attached (was universal, reverted to fixed for stability) |

**Important:** arm joints have mirrored axes because arm mesh CoM offsets have opposite X signs. Pull command (positive joint angle) rotates both arms DOWN in world frame.

## Propeller Geometry

- Link pose: `(0.050, 0.13973, 0.035)` — TRUE fuselage cage center (computed from mesh bbox of cage zone Y>130mm in `fuselage_asm.stl`)
- Visual `<pose>(-0.0735, -0.1398, -0.0305)`: shifts mesh so prop hub renders at link origin
- Joint axis through link origin = hub center → prop spins in place, no spiral
- Collision: cylinder rotated 90° around X (`<pose>0 0 0 1.5708 0 0</pose>`), radius 0.10m, length 0.02m

## Aerodynamics (LiftDrag) — DISABLED

All 5 `gz-sim-lift-drag-system` plugins commented out. Reason: gz LiftDrag computes force = 0.5·ρ·v²·Cl·A which is unbounded; vehicle rockets to orbit when airspeed builds. Bridge handles aero virtually via:
- Constant upward force (cancels gravity)
- Quadratic body-frame drag (acts as natural air resistance)

If re-enabled, recommended values from AI Engineer research (NPS Snowflake, Cal Poly parafoil thesis):
- `a0` = -0.14 rad (–8°, high-camber fabric)
- `cla` = 4.2 /rad (AR≈3 fabric wing)
- `cda` = 0.10 (with line drag)
- `alpha_stall` = 0.16 rad (9°, ram-air stalls early)
- `area` = 0.10 m² per panel × 5 = 0.5 m² total (wing loading 1.0 kg/m²)

## Motor Plugin

```xml
<plugin filename="gz-sim-multicopter-motor-model-system" name="...MulticopterMotorModel">
  <jointName>propeller_joint</jointName>
  <linkName>propeller</linkName>
  <turningDirection>cw</turningDirection>
  <maxRotVelocity>1500</maxRotVelocity>
  <motorConstant>0.0</motorConstant>           <!-- ZERO force - visual only -->
  <momentConstant>0.0</momentConstant>
  <rotorDragCoefficient>0</rotorDragCoefficient>
  <rotorVelocitySlowdownSim>1</rotorVelocitySlowdownSim>
</plugin>
```

motorConstant=0 means plugin only animates joint rotation; produces zero thrust force. Real thrust comes from bridge's `EntityWrench`.

## World (`world.sdf`)

- Physics engine: **DART** (`gz-physics-dartsim-plugin`)
- Step size: 0.002s
- Ground: thick box (-z=-0.5 to z=0, size 1000×1000×1) — flat plane caused Bullet Featherstone instability; box works for both engines
- Friction: mu=0.1 ground + 0.1 vehicle = effective 0.1 (low rolling resistance)
- Wind: disabled

Required plugins in world:
```xml
<plugin filename="gz-physics-dartsim-plugin"/>
<plugin filename="gz-sim-user-commands-system"/>
<plugin filename="gz-sim-scene-broadcaster-system"/>
<plugin filename="gz-sim-sensors-system"><render_engine>ogre2</render_engine></plugin>
<plugin filename="gz-sim-imu-system"/>
<plugin filename="gz-sim-apply-link-wrench-system"/>   <!-- enables /world/.../wrench topic -->
```

## Tuning History (Lessons)

| Test | Outcome | Verdict |
|------|---------|---------|
| Bullet Featherstone | 0.4 m/s sideways drift | FAIL — switch to DART |
| motorConstant=0.025 (linear-form) | 5 m in 5s, no climb | borderline |
| motorConstant=0.10 (linear) | 18 m/s, rockets | too aggressive |
| motorConstant=0.5 | sim blew up | crash |
| LiftDrag a0=0.10 cla=4.5 area=0.36 | reaches 13km altitude in seconds | force unbounded |
| Canopy pendulum z=1.5 (fixed joint) | thrust suppressed, vehicle stuck | failed |
| Canopy pendulum z=1.5 (ball joint) | ODE assertion failure, sim crash | failed |
| Canopy pendulum z=1.5 (universal joint) | tumbles onto side | needs heavy canopy + proper LiftDrag CP |
| Bridge ApplyLinkWrench (current) | 53m in 8s smooth roll | WORKING |
| Bridge auto-level K_RP=0.8 D_RP=0.40 | no tumble, stable | WORKING |
| PWM watchdog | no drift on AP death | WORKING |
