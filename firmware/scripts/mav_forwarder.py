"""Tiny mavlink forwarder. TCP 5760 (AP) <-> UDP 14550 (QGC) and UDP 14551 (test).
Replaces mavproxy because mavproxy daemon mode is unstable in nohup."""
import socket, threading, sys, time

AP_HOST = "127.0.0.1"; AP_PORT = 5760
QGC_PORT  = 14550
TEST_PORT = 14551

def make_tcp():
    while True:
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.connect((AP_HOST, AP_PORT))
            s.setblocking(True)
            print(f"[fwd] connected to AP TCP {AP_PORT}", flush=True)
            return s
        except Exception as e:
            print(f"[fwd] AP not ready: {e}", flush=True)
            time.sleep(2)

tcp = make_tcp()
udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
udp.bind(("127.0.0.1", 14552))   # bind for replies from QGC

def heartbeat_loop():
    """Send GCS heartbeat to AP at 5Hz + request all data streams once."""
    from pymavlink.dialects.v20 import ardupilotmega as mavlink
    mav = mavlink.MAVLink(None)
    mav.srcSystem = 255
    mav.srcComponent = 190
    # Request all data streams at 10Hz
    req = mav.request_data_stream_encode(1, 1, 0, 10, 1)   # MAV_DATA_STREAM_ALL=0
    try:
        tcp.sendall(req.pack(mav))
        print("[fwd] requested data streams", flush=True)
    except Exception as e:
        print(f"[fwd] req-stream err: {e}", flush=True)
    while True:
        try:
            msg = mav.heartbeat_encode(6, 8, 0, 0, 0)
            buf = msg.pack(mav)
            tcp.sendall(buf)
        except Exception as e:
            print(f"[fwd] hb err: {e}", flush=True)
        time.sleep(0.2)   # 5Hz

threading.Thread(target=heartbeat_loop, daemon=True).start()
print("[fwd] heartbeat loop started", flush=True)

# Track addresses to forward replies
qgc_addr = ("127.0.0.1", QGC_PORT)
test_addr = ("127.0.0.1", TEST_PORT)

def tcp_to_udp():
    """Read from AP TCP, forward to QGC + test UDP."""
    global tcp
    n = 0
    while True:
        try:
            data = tcp.recv(4096)
            if not data:
                print("[fwd] AP TCP closed, reconnecting", flush=True)
                tcp.close(); tcp = make_tcp(); continue
            udp.sendto(data, qgc_addr)
            udp.sendto(data, test_addr)
            n += 1
            if n <= 3 or n % 200 == 0:
                print(f"[fwd] tcp->udp pkt #{n} len={len(data)}", flush=True)
        except Exception as e:
            print(f"[fwd] tcp_to_udp err: {e}", flush=True)
            tcp = make_tcp()

def udp_to_tcp():
    """Read replies from UDP, forward to AP."""
    while True:
        try:
            data, addr = udp.recvfrom(4096)
            tcp.sendall(data)
        except Exception as e:
            print(f"[fwd] udp_to_tcp err: {e}", flush=True)
            time.sleep(0.5)

threading.Thread(target=tcp_to_udp, daemon=True).start()
threading.Thread(target=udp_to_tcp, daemon=True).start()
print(f"[fwd] forwarding AP <-> QGC({QGC_PORT}) + test({TEST_PORT})", flush=True)
while True:
    time.sleep(60)
