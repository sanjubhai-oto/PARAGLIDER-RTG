#!/usr/bin/env python3
"""Typhoon+pusher: auto takeoff to altitude, hover, then MANUAL pusher control.

PX4 SITL (gazebo-classic typhoon_h480_pusher) must already be running.
Connect, arm, AUTO takeoff -> hover, switch to ALTITUDE mode (manual attitude,
auto altitude) so the PUSHER actually produces forward velocity (POSITION mode
would fight it). Then you drive the pusher by typing 0-100 at the prompt.

  pip install pymavlink
  python3 fly_manual.py            # connects udp:127.0.0.1:14540

Commands at the prompt:
  0..100   set pusher thrust %         (forward velocity)
  p +/-    nudge pitch setpoint (fwd)  (optional steering)
  land     land & disarm
  q        quit
"""
import sys, time, threading, select
from pymavlink import mavutil

CONN = sys.argv[1] if len(sys.argv) > 1 else "udp:127.0.0.1:14540"
TAKEOFF_ALT = 10.0

# PX4 custom main modes
MAIN={"MANUAL":1,"ALTCTL":2,"POSCTL":3,"AUTO":4,"ACRO":5,"OFFBOARD":6,"STABILIZED":7}
AUTO_SUB={"READY":1,"TAKEOFF":2,"LOITER":3,"MISSION":4,"RTL":5,"LAND":6}

m = mavutil.mavlink_connection(CONN)
print(f"[fly] waiting heartbeat on {CONN} ...", flush=True); m.wait_heartbeat()
print(f"[fly] connected sys={m.target_system}", flush=True)
state={"alt":0.0,"mode":0,"armed":False,"pusher":0.0,"pitch_sp":0.0}

def set_mode(main, sub=0):
    m.mav.command_long_send(m.target_system,m.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE,0,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, main, sub,0,0,0,0)

def arm(v=1):
    m.mav.command_long_send(m.target_system,m.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,0,v,0,0,0,0,0,0)

def set_pusher(pct):
    state["pusher"]=pct
    val=max(0.0,min(1.0,pct/100.0))           # 0..1 forward
    # Offboard Actuator Set 1 -> param1 ; index group 0 (param7)
    m.mav.command_long_send(m.target_system,m.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_ACTUATOR,0, val,0,0,0,0,0, 0)

def telem():
    while True:
        msg=m.recv_match(blocking=True,timeout=1)
        if not msg: continue
        t=msg.get_type()
        if t=="HEARTBEAT":
            state["armed"]=bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        elif t in ("GLOBAL_POSITION_INT",):
            state["alt"]=msg.relative_alt/1000.0
threading.Thread(target=telem,daemon=True).start()

# wait for EKF/GPS
print("[fly] waiting for position estimate...",flush=True)
t0=time.time()
while time.time()-t0<40:
    g=m.recv_match(type="GLOBAL_POSITION_INT",blocking=True,timeout=1)
    if g and abs(g.lat)>1000: break

print("[fly] arming + AUTO takeoff",flush=True)
set_mode(MAIN["AUTO"],AUTO_SUB["TAKEOFF"]); time.sleep(0.5)
arm(1); time.sleep(1)
m.mav.command_long_send(m.target_system,m.target_component,
    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,0,0,0,0,0,0,0,TAKEOFF_ALT)
t0=time.time()
while time.time()-t0<25 and state["alt"]<TAKEOFF_ALT-1: time.sleep(0.3)
print(f"[fly] reached {state['alt']:.1f} m — hovering",flush=True)
time.sleep(3)

print("[fly] switching to ALTITUDE mode (manual attitude, auto altitude)",flush=True)
set_mode(MAIN["ALTCTL"]); time.sleep(1)
set_pusher(0)
print("\n=== MANUAL PUSHER CONTROL ===")
print("Type 0-100 to set pusher %, 'land' to land, 'q' to quit.\n", flush=True)

while True:
    sys.stdout.write(f"pusher[{state['pusher']:.0f}%] alt={state['alt']:.1f}m > "); sys.stdout.flush()
    r,_,_=select.select([sys.stdin],[],[],2.0)
    if not r:
        set_pusher(state["pusher"])     # keep alive
        continue
    cmd=sys.stdin.readline().strip()
    if cmd=="q": break
    if cmd=="land":
        print("[fly] LANDING",flush=True); set_pusher(0); set_mode(MAIN["AUTO"],AUTO_SUB["LAND"]); time.sleep(8); break
    try:
        set_pusher(float(cmd)); print(f"  pusher -> {state['pusher']:.0f}%",flush=True)
    except ValueError:
        print("  ? enter 0-100, 'land', or 'q'",flush=True)
print("[fly] done.",flush=True)
