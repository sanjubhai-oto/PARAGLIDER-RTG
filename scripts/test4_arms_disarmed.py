"""Test 4: arm servos respond to RC stick when DISARMED. Read-only, never arms."""
import time, subprocess, json, sys
from pymavlink import mavutil

PORT = "udp:127.0.0.1:14551"

def reset_pose():
    subprocess.run([
        "gz","service","-s","/world/paraglider_world/set_pose",
        "--reqtype","gz.msgs.Pose","--reptype","gz.msgs.Boolean","--timeout","2000",
        "--req",'name:"paraglider_uav", position:{x:0,y:0,z:0.15}, orientation:{w:1}'
    ], capture_output=True, timeout=5)

def gz_arm_joint_pos():
    """Read arm joint positions via gz model."""
    try:
        out = subprocess.run(
            ["gz","model","-m","paraglider_uav","--joint-info","-j","arm_left_joint"],
            capture_output=True, text=True, timeout=3
        ).stdout
        out2 = subprocess.run(
            ["gz","model","-m","paraglider_uav","--joint-info","-j","arm_right_joint"],
            capture_output=True, text=True, timeout=3
        ).stdout
        return out, out2
    except Exception as e:
        return f"err:{e}", ""

def main():
    print("[t4] connecting mavlink", flush=True)
    m = mavutil.mavlink_connection(PORT, source_system=255, source_component=190)
    m.wait_heartbeat(timeout=10)
    print(f"[t4] heartbeat sys={m.target_system} comp={m.target_component}", flush=True)

    # Request streams
    m.mav.request_data_stream_send(m.target_system, m.target_component, 0, 20, 1)

    # Verify disarmed
    armed = None
    for _ in range(20):
        hb = m.recv_match(type="HEARTBEAT", blocking=True, timeout=2)
        if hb:
            armed = bool(hb.base_mode & 0x80)
            print(f"[t4] mode_flag=0x{hb.base_mode:02x} armed={armed} custom={hb.custom_mode}", flush=True)
            break
    if armed:
        print("[t4] FAIL: vehicle armed at start, aborting"); sys.exit(2)

    reset_pose(); time.sleep(1)

    subtests = [
        ("A_pitch_down", 1500, 1900, 1500),
        ("B_roll_right", 1900, 1500, 1500),
        ("C_roll_left",  1100, 1500, 1500),
    ]

    results = {}
    for name, ail, elev, thr in subtests:
        print(f"\n[t4] === {name} ail={ail} elev={elev} thr={thr} ===", flush=True)
        # Verify still disarmed
        hb = m.recv_match(type="HEARTBEAT", blocking=True, timeout=3)
        armed_now = bool(hb.base_mode & 0x80) if hb else None
        print(f"[t4] armed pre-subtest={armed_now}", flush=True)

        # Sustained 4s of override at 20Hz
        t0 = time.time()
        samples = []
        rc_samples = []
        while time.time() - t0 < 4.0:
            # rc_channels_override: ch1=ail, ch2=elev, ch3=thr, rest=0 (no override)
            m.mav.rc_channels_override_send(
                m.target_system, m.target_component,
                ail, elev, thr, 1500, 0, 0, 0, 0
            )
            # Sample servos + RC
            so = m.recv_match(type="SERVO_OUTPUT_RAW", blocking=False)
            if so:
                samples.append((so.servo1_raw, so.servo2_raw, so.servo3_raw,
                                so.servo4_raw, so.servo5_raw, so.servo6_raw))
            rc = m.recv_match(type="RC_CHANNELS", blocking=False)
            if rc:
                rc_samples.append((rc.chan1_raw, rc.chan2_raw, rc.chan3_raw))
            time.sleep(0.05)

        # Release override (sentinel 0 = no override) just for this subtest gap
        for _ in range(5):
            m.mav.rc_channels_override_send(
                m.target_system, m.target_component, 0,0,0,0,0,0,0,0
            )
            time.sleep(0.05)

        # Use last second of samples (steady state)
        steady = samples[-20:] if len(samples) >= 20 else samples
        rc_steady = rc_samples[-10:] if rc_samples else []

        if steady:
            avg = [sum(c)/len(c) for c in zip(*steady)]
        else:
            avg = [None]*6
        if rc_steady:
            rc_avg = [sum(c)/len(c) for c in zip(*rc_steady)]
        else:
            rc_avg = [None]*3

        gz_l, gz_r = gz_arm_joint_pos()
        results[name] = {
            "rc_sent": (ail, elev, thr),
            "rc_seen_avg": rc_avg,
            "servo_avg": avg,
            "n_servo_samples": len(samples),
            "gz_arm_left": gz_l.strip()[-200:],
            "gz_arm_right": gz_r.strip()[-200:],
        }
        print(f"[t4] {name} servo_avg s1={avg[0]} s2={avg[1]} s3={avg[2]} s5={avg[4]} s6={avg[5]}", flush=True)
        print(f"[t4] {name} rc_seen ch1={rc_avg[0]} ch2={rc_avg[1]} ch3={rc_avg[2]}", flush=True)
        time.sleep(1.0)

    print("\n========== RESULTS ==========")
    print(json.dumps(results, indent=2, default=str))

    # Pass criteria
    rA = results["A_pitch_down"]["servo_avg"]
    rB = results["B_roll_right"]["servo_avg"]
    rC = results["C_roll_left"]["servo_avg"]
    print("\n========== PASS/FAIL ==========")
    if rA[0] is not None:
        print(f"A: s5={rA[4]:.0f} s6={rA[5]:.0f} -> both>1400? {rA[4]>1400 and rA[5]>1400}")
        print(f"B: s5={rB[4]:.0f} s6={rB[5]:.0f} -> s6>s5? {rB[5]>rB[4]}")
        print(f"C: s5={rC[4]:.0f} s6={rC[5]:.0f} -> s5>s6? {rC[4]>rC[5]}")
        print(f"throttle blocked: A_s3={rA[2]:.0f} B_s3={rB[2]:.0f} C_s3={rC[2]:.0f} -> all==1000? {rA[2]==1000 and rB[2]==1000 and rC[2]==1000}")

if __name__ == "__main__":
    main()
