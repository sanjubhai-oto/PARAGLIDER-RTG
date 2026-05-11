"""ArduParaglider SITL bridge: ArduPilot <-> Gazebo Harmonic.

ARCHITECTURE: stock ArduPlane firmware with native paraglider arm-mixer
              (added in ArduPlane/servos.cpp). Bridge translates servo PWM
              to Gazebo topics. No Lua, no scripting.

Mapping (servo packet -> gz topics):
  PWM ch3 (throttle, K_THROTTLE)            -> /paraglider_uav/command/motor_speed (RPM)
  PWM ch5 (left arm,  K_FLAPERON_LEFT=24)   -> /arm/left   (rad)
  PWM ch6 (right arm, K_FLAPERON_RIGHT=25)  -> /arm/right  (rad)

Native firmware mixer in ArduPlane:
  arm_common = max(0, -elev_demand)  # nose-down demand pulls both arms
  arm_left   = clamp(arm_common + ail_demand, 0, 0.85)
  arm_right  = clamp(arm_common - ail_demand, 0, 0.85)

Bridge sends state JSON back to AP (imu/velocity/position/attitude in NED).
"""
import socket, struct, json, time, math, threading
from gz.transport13 import Node
from gz.msgs10.actuators_pb2 import Actuators
from gz.msgs10.double_pb2 import Double
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.msgs10.imu_pb2 import IMU
from gz.msgs10.entity_wrench_pb2 import EntityWrench
from gz.msgs10.entity_pb2 import Entity

GZ_TOPIC_MOTOR  = "/paraglider_uav/command/motor_speed"
GZ_TOPIC_ARM_L  = "/arm/left"
GZ_TOPIC_ARM_R  = "/arm/right"
GZ_TOPIC_WRENCH = "/world/paraglider_world/wrench"

# Bounded thrust: replaces MulticopterMotorModel's unbounded omega^2 force.
# T/W=0.5 for 0.5kg vehicle -> 2.5 N peak. Linear in throttle => smooth, tunable.
THRUST_FWD_MAX  = 5.0     # N forward thrust at full (test: more speed -> more aero lift)
ROLL_TORQUE_MAX = 0.05    # N.m at full stick (vehicle inertia ~0.006 kg.m^2)
PITCH_TORQUE_MAX= 0.05
YAW_TORQUE_MAX  = 0.03
GZ_TOPIC_POSE   = "/world/paraglider_world/dynamic_pose/info"
GZ_TOPIC_IMU    = "/world/paraglider_world/model/paraglider_uav/link/fuselage/sensor/imu_sensor/imu"

AP_RX_PORT  = 9002    # bridge listens here for AP outputs (PWM)
AP_TX_PORT  = 9003    # bridge sends physics state here

MAX_RPM   = 800.0    # bumped up so thrust can overcome friction for ground roll + climb
ARM_MAX   = 0.50     # rad max arm pull; deeper pulls would trigger asym stall

# GPS home (matches sim_vehicle -L CMAC)
HOME_LAT = -35.363261
HOME_LON = 149.16523
HOME_ALT = 584.0
DEG_PER_M_LAT = 1.0 / 111320.0
DEG_PER_M_LON = 1.0 / (111320.0 * math.cos(math.radians(HOME_LAT)))

class State:
    def __init__(self):
        self.lock = threading.Lock()
        self.pos = [0.0, 0.0, 0.0]      # x, y, z (m, world)
        self.att = [0.0, 0.0, 0.0]      # roll, pitch, yaw (rad)
        self.vel = [0.0, 0.0, 0.0]      # vx, vy, vz (m/s, world)
        self.gyro = [0.0, 0.0, 0.0]     # body rates (rad/s)
        self.accel = [0.0, 0.0, -9.81]  # body accel incl gravity (m/s^2)
        self.t0 = time.time()
        self.t_last_pose = None
        self.quat = [1.0, 0.0, 0.0, 0.0]   # w x y z, world<-body

state = State()

def quat_to_euler(w, x, y, z):
    sinr = 2*(w*x + y*z); cosr = 1 - 2*(x*x + y*y)
    roll = math.atan2(sinr, cosr)
    sinp = 2*(w*y - z*x)
    pitch = math.asin(max(-1, min(1, sinp)))
    siny = 2*(w*z + x*y); cosy = 1 - 2*(y*y + z*z)
    yaw = math.atan2(siny, cosy)
    return roll, pitch, yaw

def quat_rotate(qw, qx, qy, qz, vx, vy, vz):
    """Rotate vector v by quaternion (world <- body or vice versa)."""
    # v' = q * v * q*
    ix =  qw*vx + qy*vz - qz*vy
    iy =  qw*vy + qz*vx - qx*vz
    iz =  qw*vz + qx*vy - qy*vx
    iw = -qx*vx - qy*vy - qz*vz
    rx = ix*qw + iw*(-qx) + iy*(-qz) - iz*(-qy)
    ry = iy*qw + iw*(-qy) + iz*(-qx) - ix*(-qz)
    rz = iz*qw + iw*(-qz) + ix*(-qy) - iy*(-qx)
    return rx, ry, rz

def on_pose(msg: Pose_V):
    for p in msg.pose:
        if p.name == "paraglider_uav":
            with state.lock:
                now = time.time()
                if state.t_last_pose:
                    dt = now - state.t_last_pose
                    if dt > 1e-4:
                        state.vel = [(p.position.x - state.pos[0]) / dt,
                                     (p.position.y - state.pos[1]) / dt,
                                     (p.position.z - state.pos[2]) / dt]
                state.t_last_pose = now
                state.pos = [p.position.x, p.position.y, p.position.z]
                state.att = list(quat_to_euler(p.orientation.w, p.orientation.x,
                                                p.orientation.y, p.orientation.z))
                state.quat = [p.orientation.w, p.orientation.x,
                              p.orientation.y, p.orientation.z]
            break

def on_imu(msg: IMU):
    with state.lock:
        # gz body axes (Y=fwd, X=right, Z=up) -> AP body (X=fwd, Y=right, Z=down)
        gx, gy, gz = msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z
        ax, ay, az = msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z
        state.gyro  = [gy, gx, -gz]
        state.accel = [ay, ax, -az]

def main():
    node = Node()
    node.subscribe(Pose_V, GZ_TOPIC_POSE, on_pose)
    node.subscribe(IMU,    GZ_TOPIC_IMU,  on_imu)

    pub_motor  = node.advertise(GZ_TOPIC_MOTOR, Actuators)
    pub_arm_l  = node.advertise(GZ_TOPIC_ARM_L, Double)
    pub_arm_r  = node.advertise(GZ_TOPIC_ARM_R, Double)
    pub_wrench = node.advertise(GZ_TOPIC_WRENCH, EntityWrench)

    rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rx.bind(("127.0.0.1", AP_RX_PORT))
    rx.settimeout(0.001)   # near non-blocking so send loop can hit 250Hz independent of PWM RX
    # AP listens on its own ephemeral port (the source it sent FROM). Capture and reply there.
    ap_addr = None

    print(f"[bridge] AP RX on UDP {AP_RX_PORT}, replying to AP source addr", flush=True)
    print(f"[bridge] Gazebo motor: {GZ_TOPIC_MOTOR}", flush=True)
    print(f"[bridge] Gazebo arms : {GZ_TOPIC_ARM_L} / {GZ_TOPIC_ARM_R}", flush=True)
    last_send = 0.0
    last_pwm_time = 0.0
    pkt_count = 0
    while True:
        # Stale-PWM watchdog: if AP silent >0.5s, ZERO all forces (don't drift)
        if time.time() - last_pwm_time > 0.5 and ap_addr is not None:
            zw = EntityWrench(); zw.entity.name = "paraglider_uav::fuselage"
            zw.entity.type = Entity.LINK
            pub_wrench.publish(zw)
        # 1. Try recv PWM from AP
        try:
            data, src = rx.recvfrom(1024)
            ap_addr = src
            last_pwm_time = time.time()
            pkt_count += 1
            if pkt_count <= 3 or pkt_count % 500 == 0:
                print(f"[bridge] PWM pkt #{pkt_count} from {src}, len={len(data)}", flush=True)
            # AP JSON SITL sends "magic\nframe_count\npwm[0..15]" as packed binary
            # New format (libraries/SITL/SIM_JSON_Master.cpp): struct {uint16 magic; uint16 framerate; uint32 frame_count; uint16 pwm[16]}
            if len(data) >= 8 + 32:
                magic, framerate, frame_count = struct.unpack("<HHI", data[:8])
                pwm = struct.unpack("<16H", data[8:8+32])
                # Read all RC channels (raw stick inputs)
                throttle  = max(0.0, (pwm[2] - 1000) / 1000.0)            # 0..1
                ail_norm  = max(-1.0, min(1.0, (pwm[0] - 1500) / 500.0))  # -1..+1
                elev_norm = max(-1.0, min(1.0, (pwm[1] - 1500) / 500.0))  # -1..+1
                # Stick BACK (low elev PWM) = nose-up. Convention: pwm<1500 = up demand.
                pitch_up   = max(0.0, -elev_norm)
                pitch_down = max(0.0,  elev_norm)
                # Drive arm visuals DIRECTLY from RC sticks (bypass firmware mixer
                # since firmware no longer drives flight; bridge handles aero).
                # Roll RIGHT (ail_norm > 0) -> right arm pulls. Roll LEFT -> left arm.
                # Pitch DOWN (elev_norm > 0) -> both arms (brake/flare).
                ail_pos = max(0.0,  ail_norm)   # 0..1 right
                ail_neg = max(0.0, -ail_norm)   # 0..1 left
                pd_norm = max(0.0,  elev_norm)  # 0..1 pitch-down (brake)
                arm_l = min(ARM_MAX, (ail_neg + pd_norm) * ARM_MAX)
                arm_r = min(ARM_MAX, (ail_pos + pd_norm) * ARM_MAX)
                rpm   = throttle * MAX_RPM
                act = Actuators(); act.velocity.append(rpm)
                pub_motor.publish(act)
                d = Double(); d.data = arm_l; pub_arm_l.publish(d)
                d = Double(); d.data = arm_r; pub_arm_r.publish(d)

                # ---------- Explicit body-frame wrench ----------
                # Body axes (fuselage link): -Y = forward (nose), +Z = up, +X = right.
                # Pitch axis = X (rotation about +X tilts nose DOWN -> nose-up = -tau_x).
                # Roll axis  = Y (rotation about +Y tilts right wing DOWN -> roll right = +tau_y).
                # Yaw axis   = Z.
                # Vehicle total mass ~0.38 kg -> weight ~3.73 N. Apply CONSTANT canopy
                # lift = weight (vehicle hangs under inflated wing always) + throttle/pitch extras.
                VEHICLE_WEIGHT   = 3.53   # N — exact gravity compensation (canopy always inflated)
                THRUST_FWD       = 0.5    # N forward at full throttle
                THRUST_UP        = 0.8    # N extra climb component at full throttle
                PITCH_UP_FORCE   = 2.0    # N extra lift on full back-stick
                PITCH_DOWN_FORCE = 0.3    # N "spill" lift loss on full fwd-stick (dive)
                PITCH_UP_TORQUE  = 0.02   # N.m nose-up torque at full back-stick
                PITCH_DOWN_TORQUE= 0.02   # N.m nose-down torque at full fwd-stick
                ROLL_TORQUE      = 0.02   # N.m roll torque at full aileron

                # WORLD-FRAME wrench — model vehicle as a stable paraglider:
                # canopy keeps it upright, motor pushes along YAW heading only.
                # Vehicle "forward" in MODEL body = -Y_body. At yaw=0, that maps to
                # world -Y. So world fwd unit = R_yaw * (0,-1,0) = (-sin(yaw)*?? ,..).
                # Using AP yaw convention (state.att[2] yaw, +Z up), at yaw=0 fwd = -Y_world.
                with state.lock:
                    yaw = state.att[2]
                    roll, pitch, _ = state.att
                    omega = list(state.gyro)  # body rates
                # World-frame forward unit vector (rotate body -Y by yaw around Z):
                # R_z(yaw) * (0,-1,0) = (sin(yaw), -cos(yaw), 0)
                fwd_x =  math.sin(yaw)
                fwd_y = -math.cos(yaw)
                # World-frame "right" unit (for roll input -> lateral nudge for visualization):
                rgt_x =  math.cos(yaw)
                rgt_y =  math.sin(yaw)

                fwd_mag = throttle * THRUST_FWD
                up_mag  = (VEHICLE_WEIGHT + throttle * THRUST_UP
                           + pitch_up * PITCH_UP_FORCE
                           - pitch_down * PITCH_DOWN_FORCE)
                # Lateral force from roll input (banks -> turn); also add small lateral nudge
                # to validate roll input changes trajectory in TEST D.
                lat_mag = ail_norm * 1.0   # 1 N lateral push at full stick

                fxw = fwd_mag * fwd_x + lat_mag * rgt_x
                fyw = fwd_mag * fwd_y + lat_mag * rgt_y
                fzw = up_mag

                # Attitude stabilization: damp roll/pitch back to 0 (canopy-rigid riser).
                # Yaw torque from roll input (banking turn).
                K_RP = 0.8   # restoring stiffness
                D_RP = 0.40  # damping
                tau_world_x = -K_RP * roll  - D_RP * omega[0]
                tau_world_y = -K_RP * pitch - D_RP * omega[1]
                tau_world_z = -ail_norm * 0.05 - 0.1 * omega[2]   # roll cmd -> yaw turn

                ew = EntityWrench()
                ew.entity.name = "paraglider_uav::fuselage"
                ew.entity.type = Entity.LINK
                ew.wrench.force.x  = fxw
                ew.wrench.force.y  = fyw
                ew.wrench.force.z  = fzw
                ew.wrench.torque.x = tau_world_x
                ew.wrench.torque.y = tau_world_y
                ew.wrench.torque.z = tau_world_z
                pub_wrench.publish(ew)
        except socket.timeout:
            pass

        # 2. Send state to AP at ~250 Hz
        now = time.time()
        if now - last_send < 0.004:
            continue
        last_send = now
        with state.lock:
            # ENU (gz) -> NED (AP). Clamp velocity to prevent EKF blowup.
            vel_ned = [state.vel[1], state.vel[0], -state.vel[2]]
            for i in range(3):
                if vel_ned[i] >  50: vel_ned[i] = 50
                if vel_ned[i] < -50: vel_ned[i] = -50
            lat = HOME_LAT + state.pos[1] * DEG_PER_M_LAT
            lon = HOME_LON + state.pos[0] * DEG_PER_M_LON
            alt = HOME_ALT + state.pos[2]
            # AP requires attitude or quaternion or it stalls main loop
            att_ned = [state.att[1], state.att[0], -state.att[2]]
            payload = {
                "timestamp": now - state.t0,
                "imu": {
                    "gyro":       state.gyro,
                    "accel_body": state.accel,
                },
                "velocity": vel_ned,
                "attitude": att_ned,
                "latitude":  lat,
                "longitude": lon,
                "altitude":  alt,
            }
        if ap_addr is None:
            continue   # don't know where to send until first PWM packet arrives
        msg = b"\n" + json.dumps(payload).encode() + b"\n"
        try:
            rx.sendto(msg, ap_addr)   # reply on same socket to AP's source addr
        except OSError:
            pass

if __name__ == "__main__":
    main()
