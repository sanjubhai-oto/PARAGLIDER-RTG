"""Sample /arm/left, /arm/right while sending RC override."""
import time, subprocess, threading
from pymavlink import mavutil

def sample_topic(topic, dur, out):
    p = subprocess.Popen(["gz","topic","-e","-t",topic],
                         stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True)
    t0=time.time()
    while time.time()-t0 < dur:
        line = p.stdout.readline()
        if not line: break
        out.append(line.strip())
    p.terminate(); p.wait(timeout=1)

m = mavutil.mavlink_connection("udp:127.0.0.1:14551", source_system=255, source_component=190)
m.wait_heartbeat(timeout=10)

cases = [("A",1500,1900,1500),("B",1900,1500,1500),("C",1100,1500,1500)]
for name,ail,elev,thr in cases:
    print(f"=== {name} ail={ail} elev={elev} ===", flush=True)
    L,R = [],[]
    tL = threading.Thread(target=sample_topic, args=("/arm/left",4,L), daemon=True)
    tR = threading.Thread(target=sample_topic, args=("/arm/right",4,R), daemon=True)
    tL.start(); tR.start()
    t0=time.time()
    while time.time()-t0 < 4:
        m.mav.rc_channels_override_send(m.target_system, m.target_component,
                                         ail, elev, thr, 1500, 0,0,0,0)
        time.sleep(0.05)
    tL.join(timeout=2); tR.join(timeout=2)
    print(f"L last3: {L[-6:]}")
    print(f"R last3: {R[-6:]}")
    # release
    for _ in range(5):
        m.mav.rc_channels_override_send(m.target_system, m.target_component, 0,0,0,0,0,0,0,0)
        time.sleep(0.05)
    time.sleep(1)
