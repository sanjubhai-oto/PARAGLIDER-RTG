#!/usr/bin/env python3
"""Live web dashboard for the PX4 typhoon+pusher SITL.  http://localhost:8088

Reads PX4 MAVLink telemetry (mode, alt, speed, attitude, RC, motor/pusher outputs)
and serves a self-refreshing page. Run alongside PX4 SITL.

  pip install pymavlink flask
  python3 dashboard.py                 # connects udp:127.0.0.1:14550
"""
import sys, threading, time
from pymavlink import mavutil
from flask import Flask, jsonify

CONN = sys.argv[1] if len(sys.argv) > 1 else "udp:127.0.0.1:14550"
PX4_MODE={1:"MANUAL",2:"ALTITUDE",3:"POSITION",4:"AUTO",5:"ACRO",6:"OFFBOARD",7:"STABILIZED"}
S={"mode":"-","armed":False,"alt":0.0,"gspd":0.0,"roll":0.0,"pitch":0.0,"yaw":0.0,
   "rc":[0]*8,"mot":[0]*8,"pusher":0}

def reader():
    m=mavutil.mavlink_connection(CONN); m.wait_heartbeat()
    m.mav.request_data_stream_send(m.target_system,m.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_ALL,20,1)
    while True:
        msg=m.recv_match(blocking=True,timeout=1)
        if not msg: continue
        t=msg.get_type()
        if t=="HEARTBEAT":
            S["armed"]=bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            main=(msg.custom_mode>>16)&0xff
            S["mode"]=PX4_MODE.get(main,f"#{main}")
        elif t=="GLOBAL_POSITION_INT": S["alt"]=msg.relative_alt/1000.0
        elif t=="VFR_HUD": S["gspd"]=msg.groundspeed
        elif t=="ATTITUDE":
            import math
            S["roll"]=math.degrees(msg.roll); S["pitch"]=math.degrees(msg.pitch); S["yaw"]=math.degrees(msg.yaw)
        elif t=="RC_CHANNELS": S["rc"]=[getattr(msg,f"chan{i}_raw") for i in range(1,9)]
        elif t=="SERVO_OUTPUT_RAW": S["mot"]=[getattr(msg,f"servo{i}_raw") for i in range(1,9)]
        elif t=="ACTUATOR_OUTPUT_STATUS":
            a=list(msg.actuator)[:8]
            if any(a): S["mot"]=[int(x) for x in a]
            if len(msg.actuator)>6: S["pusher"]=int(msg.actuator[6])

threading.Thread(target=reader,daemon=True).start()
app=Flask(__name__)

@app.route("/data.json")
def data(): return jsonify(S)

@app.route("/")
def index():
    return """<!doctype html><html><head><meta charset=utf-8><title>Typhoon+Pusher</title>
<style>body{background:#14141a;color:#eee;font-family:monospace;margin:0;padding:16px}
h1{color:#6cf;margin:0 0 2px}.sub{color:#888;margin-bottom:14px}
.big{font-size:30px;color:#7fffa0}.row{margin:3px 0}.lab{display:inline-block;width:150px;color:#bbb}
.barwrap{display:inline-block;width:300px;height:14px;background:#222;border:1px solid #555;vertical-align:middle}
.bar{height:14px}.val{margin-left:8px}.sec{color:#fc8;margin:14px 0 6px;font-weight:bold}</style></head>
<body><h1>MicoAir / Typhoon H480 + Pusher</h1><div class=sub>PX4 SITL + Gazebo-classic &mdash; live</div>
<div class=row><span class=lab>MODE</span><span id=mode class=big>-</span>
  <span id=armed style=margin-left:20px></span></div>
<div class=row><span class=lab>ALT</span><span id=alt></span> m
  <span class=lab style=width:90px>SPEED</span><span id=spd></span> m/s</div>
<div class=row><span class=lab>ATT (R/P/Y)</span><span id=att></span> deg</div>
<div class=sec>RC INPUTS</div><div id=rc></div>
<div class=sec>MOTOR / PUSHER OUTPUTS</div><div id=mot></div>
<script>
const N=["Roll","Pitch","Thr","Yaw","ch5","ch6","ch7","ch8"];
function bar(v,lo,hi,c){let f=Math.max(0,Math.min(1,(v-lo)/(hi-lo)));
 return `<span class=barwrap><span class=bar style="width:${(f*300)|0}px;background:${c}"></span></span><span class=val>${v}</span>`;}
async function tick(){let d=await (await fetch('/data.json')).json();
 mode.textContent=d.mode; armed.innerHTML=d.armed?"<span style=color:#7fa>ARMED</span>":"<span style=color:#f77>DISARMED</span>";
 alt.textContent=d.alt.toFixed(1); spd.textContent=d.gspd.toFixed(1);
 att.textContent=d.roll.toFixed(0)+" / "+d.pitch.toFixed(0)+" / "+d.yaw.toFixed(0);
 rc.innerHTML=d.rc.map((v,i)=>`<div class=row><span class=lab>ch${i+1} ${N[i]}</span>${bar(v,1000,2000,'#5af')}</div>`).join('');
 mot.innerHTML=d.mot.slice(0,6).map((v,i)=>`<div class=row><span class=lab>M${i+1}</span>${bar(v,1000,2000,'#5d8')}</div>`).join('')
   +`<div class=row><span class=lab>M7 PUSHER</span>${bar(d.mot[6]||1000,1000,2000,'#f93')}</div>`;}
setInterval(tick,200);tick();
</script></body></html>"""

if __name__=="__main__":
    print("dashboard on http://localhost:8088",flush=True)
    app.run(host="0.0.0.0",port=8088,threaded=True)
