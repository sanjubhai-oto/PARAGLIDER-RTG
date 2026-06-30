#!/usr/bin/env python3
"""
SITL probe for the pusher feed-forward patch.  RUN ON YOUR MAC (where ArduPilot
source + a running Copter SITL exist).  Requires: pip install pymavlink

What it proves (software-in-the-loop):
  SITL's stock hexa has NO pusher, so it cannot reproduce the aerodynamic
  disturbance. Instead this checks the PATCH ITSELF:
    * with ATC_PUSH_EN=0  -> sweeping the pusher RC channel does NOTHING to pitch
    * with ATC_PUSH_EN=1  -> the FF term injects a pitch command proportional to
      (pusher_norm^2), so the vehicle holds a measurable pitch bias that scales
      with the pusher and with ATC_PUSH_PIT, and flips sign with the gain sign.
  That confirms the term is wired in, reads the right channel, and has the
  expected sign/magnitude before you ever fly it.

Usage:
  # terminal 1 (in your ardupilot checkout, patched + built):
  Tools/autotest/sim_vehicle.py -v ArduCopter -f hexa --console
  # terminal 2:
  python3 sitl_pusher_probe.py --conn udp:127.0.0.1:14550 --gain 0.12
"""
import argparse, time, statistics
from pymavlink import mavutil

PUSH_CH = 8           # RC channel the patch reads (ATC_PUSH_RC)
SWEEP   = [1095, 1300, 1500, 1700, 1935]   # pusher PWM steps (idle..full)

def pset(m, name, val):
    m.mav.param_set_send(m.target_system, m.target_component,
                         name.encode(), float(val), mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
    time.sleep(0.3)

def rc_override(m, ch_vals):
    # ch_vals: dict {1..8: pwm}; 0 = release to sim RC
    ch = [ch_vals.get(i, 0) for i in range(1, 9)]
    m.mav.rc_channels_override_send(m.target_system, m.target_component, *ch)

def mean_pitch(m, secs=3.0):
    vals = []
    t0 = time.time()
    while time.time() - t0 < secs:
        msg = m.recv_match(type='ATTITUDE', blocking=True, timeout=2)
        if msg:
            vals.append(msg.pitch * 57.2958)  # rad->deg
    return statistics.mean(vals) if vals else float('nan')

def wait_mode(m, name):
    m.set_mode(name)
    time.sleep(1)

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--conn', default='udp:127.0.0.1:14550')
    ap.add_argument('--gain', type=float, default=0.12, help='ATC_PUSH_PIT to test')
    args = ap.parse_args()

    m = mavutil.mavlink_connection(args.conn)
    m.wait_heartbeat()
    print(f"connected sys={m.target_system}")

    pset(m, 'ATC_PUSH_RC', PUSH_CH)
    pset(m, 'ATC_PUSH_PIT', args.gain)
    pset(m, 'ATC_PUSH_RLL', 0.0)
    pset(m, 'ATC_PUSH_YAW', 0.0)

    results = {}
    for en in (0, 1):
        pset(m, 'ATC_PUSH_EN', en)
        wait_mode(m, 'ALT_HOLD')
        m.arming.is_armed() if hasattr(m, 'arming') else None
        m.mav.command_long_send(m.target_system, m.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
        time.sleep(2)
        # climb: throttle mid (+ keep roll/pitch/yaw centered)
        rc_override(m, {1: 1500, 2: 1500, 3: 1650, 4: 1500, PUSH_CH: 1095})
        time.sleep(5)
        rc_override(m, {1: 1500, 2: 1500, 3: 1500, 4: 1500, PUSH_CH: 1095})  # hold alt
        time.sleep(3)

        row = {}
        for pwm in SWEEP:
            rc_override(m, {1: 1500, 2: 1500, 3: 1500, 4: 1500, PUSH_CH: pwm})
            time.sleep(2)
            row[pwm] = mean_pitch(m, 3.0)
        results[en] = row

        rc_override(m, {1: 1500, 2: 1500, 3: 1100, 4: 1500, PUSH_CH: 1095})  # descend
        time.sleep(6)
        m.mav.command_long_send(m.target_system, m.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 0, 0, 0, 0, 0, 0)
        time.sleep(2)

    print("\n==== RESULT: mean pitch (deg) vs pusher PWM ====")
    print(f"ATC_PUSH_PIT = {args.gain}")
    print(f"{'pusher PWM':>10} | {'EN=0 (stock)':>14} | {'EN=1 (FF on)':>14} | {'delta':>8}")
    for pwm in SWEEP:
        a, b = results[0][pwm], results[1][pwm]
        print(f"{pwm:>10} | {a:>14.2f} | {b:>14.2f} | {b-a:>8.2f}")
    print("\nPASS if: EN=0 column stays ~flat with PWM, and EN=1 pitch grows with")
    print("PWM (sign matches your ATC_PUSH_PIT sign). That = the FF term is live.")

if __name__ == '__main__':
    main()
