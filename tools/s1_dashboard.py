#!/usr/bin/env python3
"""
s1_dashboard.py - Buddy 1 (subsystem 1) dashboard in the browser.

Runs a small web server on this computer and opens the dashboard in Chrome
(http://localhost:8080).  The page has START / STOP / E-STOP buttons (Esc
key = E-STOP), the barcode letter -> movement map, the robot's heartbeat and
decoded telemetry, NFR6 / NFR7 / NFR17 timing and an event log.  It talks
to the robot through the MQTT broker, like the terminal version did.

Only built-in Python modules plus paho-mqtt: no tkinter, no web framework,
nothing loaded from the internet (works on a phone hotspot).

  pip install paho-mqtt
  python3 s1_dashboard.py                         # broker on this computer
  python3 s1_dashboard.py --broker 172.20.10.2    # broker elsewhere
  python3 s1_dashboard.py --http-port 8081        # if 8080 is taken
  python3 s1_dashboard.py --no-browser            # do not open Chrome
  Ctrl+C in the terminal stops the dashboard.

Optional, without the page (as before):
  python3 s1_dashboard.py --broker 192.168.1.10 --send START   # STOP / E-STOP
  python3 s1_dashboard.py --broker 192.168.1.10 --map forward=A,uturn=F,left=Z,right=W

Bit layouts match app_program/subsystem1/s1_codec.h (MSB-first, 4-bit header:
2-bit message type, then 2-bit version).  Telemetry slots:
  B2  26 bits  Motion status [25:0]   (subsystem2.h g_motion_status)
        [6:0] left speed, [13:7] right speed (cm/s, signed), [25:14] distance
        (mm, wraps every 4096 mm)
  B3   8 bits  Line & Barcode          (subsystem3.h s3_tlm_slot)
        [7:6] line state, [5:4] barcode state, [3:2] barcodes decoded
        (mod 4; 4 Nov, was the unused last command), [1] barcode error
        (held 1 s), [0] edge overflow (sticky)
  B4  44 bits  IMU                     (subsystem4.h s4_tlm_slot, s4_pack.h)
        [43:13] word 0: [2:0] state, [11:3] pitch 0.1 deg signed,
                [20:12] heading deg, [29:21] turn rate deg/s signed,
                [30] calibrated
        [12:2]  word 1: [4:0] hump mm, [8:5] hump count, [10:9] I2C fails
        [1] lifted / overturned (12 Nov)
        [0] spare, 0
  B5  55 bits  Obstacle Scan           (not filled by the robot yet: shown raw)
Heartbeat, last 5 bits (4 Nov, were padding): last barcode letter decoded,
0 = none, 1..26 = 'A'..'Z'.  A new read is seen from B3's counter, the
letter from the next heartbeat (at most 1 s later).
"""
import argparse
import csv
import json
import random
import statistics
import threading
import time
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import paho.mqtt.client as mqtt

# ================================================================ protocol
TYPE_BITS, VERSION_BITS = 2, 2
VERSION = 1
# Telemetry slot widths (bits) -- must match S1_Bx_BITS in s1_codec.h
B2_BITS, B3_BITS, B4_BITS, B5_BITS = 26, 8, 44, 55
TLM_BITS = TYPE_BITS + VERSION_BITS + B2_BITS + B3_BITS + B4_BITS + B5_BITS + 1  # 138
TLM_LEN = (TLM_BITS + 7) // 8                                                    # 18
T_TELEMETRY, T_HEARTBEAT, T_COMMAND, T_ACK = 0, 1, 2, 3
CMD = {"START": 0b00, "STOP": 0b01, "E-STOP": 0b10}
STATES = ["INIT", "IDLE", "RUNNING", "E_STOPPED", "FAULT", "COMPLETE", "6", "7"]


class Bits:
    """MSB-first bit reader, same order as s1_get_bits()."""

    def __init__(self, data):
        self.v = int.from_bytes(data, "big")
        self.n = len(data) * 8
        self.pos = 0

    def get(self, nbits):
        shift = self.n - self.pos - nbits
        self.pos += nbits
        return (self.v >> shift) & ((1 << nbits) - 1)


def header(b):
    return b.get(TYPE_BITS), b.get(VERSION_BITS)


def decode_telemetry(data):
    if len(data) != TLM_LEN:
        return None
    b = Bits(data)
    t, v = header(b)
    if (t, v) != (T_TELEMETRY, VERSION):
        return None
    d = {"b2": b.get(B2_BITS), "b3": b.get(B3_BITS),
         "b4": b.get(B4_BITS), "b5": b.get(B5_BITS)}
    d["wifi_loss"] = b.get(1)          # last data bit (tribunal decision)
    return d


def decode_heartbeat(data):
    if len(data) != 5:
        return None
    b = Bits(data)
    t, v = header(b)
    if (t, v) != (T_HEARTBEAT, VERSION):
        return None
    d = {"seq": b.get(8), "state": STATES[b.get(3)],
         "uptime_s": b.get(16), "link_losses": b.get(4)}
    code = b.get(5)                    # 4 Nov: last barcode, 0 = none
    d["barcode"] = chr(ord("A") + code - 1) if 1 <= code <= 26 else None
    return d


def decode_ack(data):
    if len(data) != 2:
        return None
    b = Bits(data)
    t, v = header(b)
    if (t, v) != (T_ACK, VERSION):
        return None
    return {"result": "FAILED" if b.get(1) else "INVALID", "id": b.get(6)}


MAP_ORDER = ["forward", "uturn", "left", "right"]   # S1_MAP_xxx order in s1_codec.h
MAP_ALIASES = {"forward": "forward", "f": "forward", "straight": "forward",
               "uturn": "uturn", "u-turn": "uturn", "u": "uturn",
               "left": "left", "l": "left", "right": "right", "r": "right"}


def parse_map(text):
    """'forward=A,uturn=F,left=Z,right=W' or 'A,F,Z,W' -> {'forward': 'A', ...}"""
    parts = [s.strip() for s in text.replace(" ", ",").split(",") if s.strip()]
    if len(parts) != 4:
        raise SystemExit("--map needs 4 entries: forward, uturn, left, right")
    m = {}
    for i, part in enumerate(parts):
        if "=" in part:
            k, v = part.split("=", 1)
            k = MAP_ALIASES.get(k.strip().lower())
            if k is None:
                raise SystemExit(f"--map: unknown movement in '{part}' (use forward/uturn/left/right)")
        else:
            k, v = MAP_ORDER[i], part
        v = v.strip().upper()
        if len(v) != 1 or not ("A" <= v <= "Z"):
            raise SystemExit(f"--map: '{v}' is not a letter A-Z")
        m[k] = v
    if sorted(m) != sorted(MAP_ORDER):
        raise SystemExit("--map needs each of forward, uturn, left, right once")
    if len(set(m.values())) != 4:
        raise SystemExit("--map: the four letters must be different")
    return m


def encode_map(m, cmd_id):
    """4 bytes: type(2) version(2) 11(2) id(6) forward(5) uturn(5) left(5) right(5)"""
    v = (T_COMMAND << 30) | (VERSION << 28) | (0b11 << 26) | ((cmd_id & 0x3F) << 20)
    for i, k in enumerate(MAP_ORDER):
        v |= (ord(m[k]) - ord("A")) << (15 - 5 * i)
    return v.to_bytes(4, "big")


def encode_command(name, cmd_id):
    """2 bytes: type(2) version(2) command(2) id(6) pad(4)"""
    v = (T_COMMAND << 14) | (VERSION << 12) | (CMD[name] << 10) | ((cmd_id & 0x3F) << 4)
    return v.to_bytes(2, "big")


# ================================================================ slot decoders
LINE_STATES = ["ON_LINE", "DRIFTING", "JUNCTION", "LOST"]
BC_STATES = ["WAITING", "MEASURING", "DECODING", "SENT (cool-down)"]
MOVE_NAMES = {"forward": "STRAIGHT ON", "uturn": "U-TURN", "left": "LEFT",
              "right": "RIGHT"}
IMU_STATES = ["CALIBRATING", "LEVEL", "CLIMBING", "DESCENDING", "LIFTED",
              "FAULT", "6?", "7?"]


def sext(value, bits):
    """Sign-extend the low 'bits' bits of value."""
    value &= (1 << bits) - 1
    return value - (1 << bits) if value & (1 << (bits - 1)) else value


def decode_b2(b2):
    return {"left_cm_s": sext(b2, 7),
            "right_cm_s": sext(b2 >> 7, 7),
            "dist_mm": (b2 >> 14) & 0xFFF}


def decode_b3(b3):
    return {"line": LINE_STATES[(b3 >> 6) & 3],
            "barcode": BC_STATES[(b3 >> 4) & 3],
            "reads": (b3 >> 2) & 3,     # barcodes decoded, mod 4
            "bc_err": (b3 >> 1) & 1,
            "overflow": b3 & 1}


def decode_b4(b4):
    w0 = (b4 >> 13) & 0x7FFFFFFF
    w1 = (b4 >> 2) & 0x7FF
    return {"state": IMU_STATES[w0 & 7],
            "pitch_deg": sext(w0 >> 3, 9) / 10.0,
            "heading_deg": (w0 >> 12) & 0x1FF,
            "rate_dps": sext(w0 >> 21, 9),
            "calibrated": (w0 >> 30) & 1,
            "hump_mm": w1 & 0x1F,
            "hump_count": (w1 >> 5) & 0xF,
            "i2c_fails": (w1 >> 9) & 3,
            "lifted": (b4 >> 1) & 1}


def dist_diff(old_mm, new_mm):
    """Signed difference of two 12-bit wrapping distances, in mm."""
    return ((new_mm - old_mm + 2048) & 0xFFF) - 2048


def new_client():
    try:
        return mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    except AttributeError:                           # paho-mqtt < 2.0
        return mqtt.Client()


# ================================================================ robot link
TELEMETRY_MS, HEARTBEAT_MS, RECONNECT_S = 500, 1000, 10.0
LOG_KEEP = 200


class Robot:
    """MQTT side.  paho's thread writes, the web server's threads read; all
    under self.cv, which also wakes the page streams on every change."""

    def __init__(self, host, port, log_path):
        self.host, self.port = host, port
        self.cv = threading.Condition()
        self.version = 0
        self.broker = "connecting"
        self.robot = "unknown"
        self.hb = None
        self.hb_time = None
        self.tlm = None
        self.prev_dist = None
        self.odo_mm = 0
        self.gaps = {"telemetry": [], "heartbeat": []}
        self.last = {"telemetry": None, "heartbeat": None}
        self.offline_at = None
        self.reconnects = []
        self.last_state = None
        self.last_cmd = "No command sent yet."
        self.sent = {}                        # command id -> description
        # Command ids count up (4 Nov): the robot ignores a START / STOP
        # whose id is one of the last 4 it accepted (subsystem1.c,
        # RECENT_IDS), so random ids made ~1 press in 16 do nothing.
        self.next_id = random.randrange(64)
        # Barcodes (4 Nov): letter from the heartbeat, new reads from the
        # telemetry counter
        self.bc_letter = None                 # last letter the robot read
        self.bc_prev_count = None
        self.bc_pending = 0                   # reads seen, letter not yet
        self.bc_reads = 0                     # reads seen by this dashboard
        self.bc_time = None
        self.map_sent = None                  # last MAP sent from this page
        self.log = []
        self.log_id = 0
        self.csv_file = open(log_path, "a", newline="")
        self.csv = csv.writer(self.csv_file)

        self.client = new_client()
        self.client.on_connect = self._on_connect
        self.client.on_disconnect = self._on_disconnect
        self.client.on_message = self._on_message
        self.client.connect_async(host, port, keepalive=30)
        self.client.loop_start()            # reconnects by itself

    # -------------------------------------------------- helpers (hold cv)
    def _note(self, text, kind="info"):
        self.log_id += 1
        self.log.append({"id": self.log_id, "t": time.strftime("%H:%M:%S"),
                         "text": text, "kind": kind})
        del self.log[:-LOG_KEEP]

    def _changed(self):
        self.version += 1
        self.cv.notify_all()

    # -------------------------------------------------- paho callbacks
    def _on_connect(self, client, userdata, flags, rc, *args):
        ok = (rc == 0) if isinstance(rc, int) else not rc.is_failure
        if ok:
            client.subscribe("robot/#", qos=1)
        with self.cv:
            self.broker = "connected" if ok else "refused"
            self._note(f"connected to the broker {self.host}:{self.port}" if ok
                       else f"broker refused the connection ({rc})",
                       "info" if ok else "bad")
            self._changed()

    def _on_disconnect(self, client, userdata, *args):
        with self.cv:
            self.broker = "lost"
            self._note("lost the broker, retrying...", "bad")
            self._changed()

    def _on_message(self, client, userdata, msg):
        now = time.time()
        data = bytes(msg.payload)
        with self.cv:
            self.csv.writerow([f"{now:.3f}", msg.topic, len(data), data.hex()])
            self.csv_file.flush()
            if msg.topic == "robot/status":
                self._status(now, data.decode(errors="replace"))
            else:
                kind = msg.topic.split("/")[-1]
                if kind in self.gaps:
                    if self.last[kind] is not None:
                        self.gaps[kind].append(now - self.last[kind])
                    self.last[kind] = now
                if kind == "telemetry":
                    self._telemetry(data)
                elif kind == "heartbeat":
                    self._heartbeat(now, data)
                elif kind == "ack":
                    self._ack(data)
            self._changed()

    def _status(self, now, text):
        if text == "ONLINE":
            self.robot = "ONLINE"
            if self.offline_at:
                dt = now - self.offline_at
                self.reconnects.append(dt)
                self._note(f"robot back ONLINE after {dt:.2f} s (NFR7)")
                self.offline_at = None
            else:
                self._note("robot ONLINE")
        elif text == "OFFLINE":
            self.robot = "OFFLINE"
            self.offline_at = now
            self._note("robot OFFLINE", "bad")
        else:
            self._note(f"status: {text}")

    def _heartbeat(self, now, data):
        d = decode_heartbeat(data)
        if not d:
            self._note(f"heartbeat UNDECODABLE {data.hex()}", "bad")
            return
        self.hb, self.hb_time = d, now
        if d["barcode"]:
            self.bc_letter = d["barcode"]
        if self.bc_pending and self.bc_letter:
            more = (f" ({self.bc_pending} barcodes since the last update,"
                    " this is the latest)" if self.bc_pending > 1 else "")
            self._note(f"barcode read: '{self.bc_letter}' -> "
                       f"{self._meaning(self.bc_letter)}{more}", "barcode")
            self.bc_reads += self.bc_pending
            self.bc_pending = 0
            self.bc_time = now
        if d["state"] != self.last_state:
            self._note(f"mission state: {self.last_state or '-'} -> {d['state']}",
                       "bad" if d["state"] in ("E_STOPPED", "FAULT") else "info")
            self.last_state = d["state"]

    def _telemetry(self, data):
        d = decode_telemetry(data)
        if not d:
            self._note(f"telemetry UNDECODABLE {data.hex()}", "bad")
            return
        b2 = decode_b2(d["b2"])
        b3 = decode_b3(d["b3"])
        if self.bc_prev_count is not None and b3["reads"] != self.bc_prev_count:
            self.bc_pending += (b3["reads"] - self.bc_prev_count) % 4
        self.bc_prev_count = b3["reads"]
        if self.prev_dist is not None:
            self.odo_mm += dist_diff(self.prev_dist, b2["dist_mm"])
        self.prev_dist = b2["dist_mm"]
        self.tlm = {"b2": b2, "b3": b3, "b4": decode_b4(d["b4"]),
                    "b5": d["b5"], "wifi_loss": d["wifi_loss"],
                    "odo_cm": round(self.odo_mm / 10, 1)}

    def _meaning(self, letter):
        """What the robot will do for this letter, by the last map sent."""
        if not self.map_sent:
            return "no map sent from this dashboard yet"
        for k, v in self.map_sent.items():
            if v == letter:
                return MOVE_NAMES[k] + " at the next junction"
        return "not in the map: straight on at the next junction"

    def _ack(self, data):
        d = decode_ack(data)
        if not d:
            self._note(f"ack UNDECODABLE {data.hex()}", "bad")
            return
        what = self.sent.get(d["id"], "unknown command")
        self._note(f"robot REFUSED {what} (id {d['id']}): {d['result']}", "refused")

    # -------------------------------------------------- from the page
    def take_id(self):
        """Next command id, 0..63, never one of the last 63 sent."""
        with self.cv:
            cmd_id = self.next_id
            self.next_id = (self.next_id + 1) % 64
            return cmd_id

    def send(self, payload, what, map_letters=None):
        cmd_id = payload[1]
        with self.cv:
            ok = self.broker == "connected"
            if ok:
                info = self.client.publish("robot/cmd", payload[0], qos=1)
                ok = info.rc == mqtt.MQTT_ERR_SUCCESS
            if ok and map_letters:
                self.map_sent = dict(map_letters)
            self.sent[cmd_id] = what
            self.last_cmd = (f"Last sent: {what}  id={cmd_id}  bytes={payload[0].hex()}"
                             f"  at {time.strftime('%H:%M:%S')}" +
                             ("" if ok else "  (NOT sent: broker not connected)"))
            self._note(f"sent {what} id={cmd_id}" if ok else
                       f"could not send {what}: broker not connected", "info" if ok else "bad")
            self._changed()
        return ok

    def snapshot(self):
        """Everything the page shows, as one JSON-ready dict (hold cv)."""
        now = time.time()

        def timing(kind):
            g = self.gaps[kind]
            if len(g) < 2:
                return None
            return {"n": len(g), "mean": round(statistics.mean(g) * 1000),
                    "max": round(max(g) * 1000)}

        ok = sum(1 for x in self.reconnects if x <= RECONNECT_S)
        return {"version": self.version, "broker": self.broker,
                "broker_addr": f"{self.host}:{self.port}", "robot": self.robot,
                "hb": self.hb,
                "hb_age": None if self.hb_time is None else round(now - self.hb_time, 1),
                "tlm": self.tlm, "last_cmd": self.last_cmd,
                "bc": {"letter": self.bc_letter, "reads": self.bc_reads,
                       "meaning": self._meaning(self.bc_letter) if self.bc_letter else None,
                       "age": None if self.bc_time is None else round(now - self.bc_time),
                       "map": self.map_sent},
                "timing": {"tlm": timing("telemetry"), "hb": timing("heartbeat"),
                           "rec": {"ok": ok, "n": len(self.reconnects)}},
                "log": self.log[-60:]}

    def close(self):
        self.client.disconnect()
        self.client.loop_stop()
        self.csv_file.close()


# ================================================================ web server
class Handler(BaseHTTPRequestHandler):
    robot = None                              # set in serve()

    def log_message(self, fmt, *args):        # keep the terminal quiet
        pass

    def _reply(self, code, body, ctype="application/json"):
        data = body.encode() if isinstance(body, str) else body
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(data)

    def do_GET(self):
        if self.path in ("/", "/index.html"):
            self._reply(200, PAGE, "text/html; charset=utf-8")
        elif self.path == "/events":
            self._events()
        else:
            self._reply(404, '{"error": "not found"}')

    def _events(self):
        """Server-Sent Events: the whole snapshot on every change, and at
        least once a second (so 'last heard' keeps counting)."""
        self.send_response(200)
        self.send_header("Content-Type", "text/event-stream")
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        seen = -1
        try:
            while True:
                with self.robot.cv:
                    if self.robot.version == seen:
                        self.robot.cv.wait(timeout=1.0)
                    seen = self.robot.version
                    snap = self.robot.snapshot()
                self.wfile.write(b"data: " + json.dumps(snap).encode() + b"\n\n")
                self.wfile.flush()
                time.sleep(0.05)              # at most ~20 updates a second
        except (BrokenPipeError, ConnectionResetError, OSError):
            return                            # page closed

    def do_POST(self):
        try:
            n = int(self.headers.get("Content-Length", 0))
            body = json.loads(self.rfile.read(n) or b"{}")
        except (ValueError, json.JSONDecodeError):
            self._reply(400, '{"error": "bad request"}')
            return
        cmd_id = self.robot.take_id()
        if self.path == "/api/command":
            name = body.get("name")
            if name not in CMD:
                self._reply(400, json.dumps({"error": f"unknown command {name}"}))
                return
            ok = self.robot.send((encode_command(name, cmd_id), cmd_id), name)
        elif self.path == "/api/map":
            text = ",".join(f"{k}={body.get(k, '')}" for k in MAP_ORDER)
            try:
                m = parse_map(text)
            except SystemExit as e:
                self._reply(400, json.dumps({"error": str(e).replace("--map", "Map")}))
                return
            what = "MAP " + " ".join(f"{k}={m[k]}" for k in MAP_ORDER)
            ok = self.robot.send((encode_map(m, cmd_id), cmd_id), what, m)
        else:
            self._reply(404, '{"error": "not found"}')
            return
        self._reply(200 if ok else 503, json.dumps(
            {"ok": ok, "id": cmd_id,
             "error": None if ok else "Broker not connected: command not sent."}))


def open_chrome(url):
    for name in ('open -a "Google Chrome" %s',            # macOS
                 "google-chrome", "chrome", "chromium", "chromium-browser"):
        try:
            if webbrowser.get(name).open(url):
                return
        except webbrowser.Error:
            pass
    webbrowser.open(url)                                  # default browser


def serve(args):
    robot = Robot(args.broker, args.port, args.log)
    Handler.robot = robot
    try:
        httpd = ThreadingHTTPServer((args.listen, args.http_port), Handler)
    except OSError as e:
        robot.close()
        raise SystemExit(f"Cannot use port {args.http_port} ({e}). "
                         f"Try: --http-port {args.http_port + 1}")
    httpd.daemon_threads = True
    url = f"http://localhost:{args.http_port}"
    print(f"[dashboard] MQTT broker {args.broker}:{args.port}")
    print(f"[dashboard] open {url} in Chrome   (Ctrl+C here to stop)")
    if args.listen != "127.0.0.1":
        print(f"[dashboard] WARNING: listening on {args.listen}: anyone on this "
              "network can open the page and drive the robot")
    if not args.no_browser:
        threading.Timer(0.8, open_chrome, args=(url,)).start()
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("\n[dashboard] stopped")
    finally:
        httpd.server_close()
        robot.close()


def send_once(args):
    """--send / --map: publish one command and exit, no page (as before)."""
    client = new_client()
    client.connect(args.broker, args.port, keepalive=30)
    cmd_id = args.id if args.id is not None else random.randrange(64)
    if args.map:
        m = parse_map(args.map)
        payload = encode_map(m, cmd_id)
        what = "MAP " + " ".join(f"{k}={m[k]}" for k in MAP_ORDER)
    else:
        payload = encode_command(args.send, cmd_id)
        what = args.send
    client.loop_start()
    client.publish("robot/cmd", payload, qos=1).wait_for_publish()
    print(f"[dashboard] sent {what} id={cmd_id} bytes={payload.hex()}")
    time.sleep(1)
    client.loop_stop()


def main():
    ap = argparse.ArgumentParser(description="Buddy 1 robot dashboard (opens in Chrome)")
    ap.add_argument("--broker", default="127.0.0.1",
                    help="MQTT broker address (default: this computer)")
    ap.add_argument("--port", type=int, default=1883, help="MQTT broker port")
    ap.add_argument("--http-port", type=int, default=8080, help="dashboard page port")
    ap.add_argument("--listen", default="127.0.0.1",
                    help="0.0.0.0 = let other devices (e.g. a phone) open the page")
    ap.add_argument("--no-browser", action="store_true", help="do not open Chrome")
    ap.add_argument("--send", choices=list(CMD), help="send one command, no page")
    ap.add_argument("--map", help="send a barcode mapping, no page, "
                                  "e.g. forward=A,uturn=F,left=Z,right=W")
    ap.add_argument("--id", type=int, help="command id 0..63 for --send / --map")
    ap.add_argument("--log", default="s1_mqtt_log.csv")
    args = ap.parse_args()
    if args.send or args.map:
        send_once(args)
    else:
        serve(args)


# ================================================================ the page
PAGE = r"""<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Robot Dashboard</title>
<style>
:root {
  --bg: #f4f5f7; --card: #ffffff; --ink: #1f2328; --muted: #656d76;
  --line: #d8dee4; --green: #1a7f37; --amber: #d97706; --red: #cf222e;
  --blue: #0969da; --grey: #8c959f; --chip: #eef1f4;
}
@media (prefers-color-scheme: dark) {
  :root { --bg: #0d1117; --card: #161b22; --ink: #e6edf3; --muted: #8d96a0;
          --line: #30363d; --chip: #21262d; }
}
* { box-sizing: border-box; }
body { margin: 0; background: var(--bg); color: var(--ink);
       font: 15px/1.4 -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif; }
header { display: flex; flex-wrap: wrap; align-items: center; gap: 10px 16px;
         padding: 14px 20px; border-bottom: 1px solid var(--line); background: var(--card); }
h1 { font-size: 18px; margin: 0 auto 0 0; }
.pill { display: inline-flex; align-items: center; gap: 6px; padding: 4px 10px;
        border-radius: 999px; background: var(--chip); font-size: 13px; }
.dot { width: 9px; height: 9px; border-radius: 50%; background: var(--grey); }
.ok .dot { background: var(--green); } .warn .dot { background: var(--amber); }
.bad .dot { background: var(--red); }
main { max-width: 1200px; margin: 0 auto; padding: 16px 20px 32px; }
.card { background: var(--card); border: 1px solid var(--line); border-radius: 10px;
        padding: 14px 16px; }
.card h2 { font-size: 13px; text-transform: uppercase; letter-spacing: .04em;
           color: var(--muted); margin: 0 0 10px; }
.controls { display: grid; grid-template-columns: repeat(3, 1fr); gap: 12px; }
.big { border: 0; border-radius: 10px; padding: 22px 10px; color: #fff; cursor: pointer;
       font-size: 22px; font-weight: 700; letter-spacing: .02em; transition: filter .1s, transform .05s; }
.big:hover:not(:disabled) { filter: brightness(1.08); }
.big:active:not(:disabled) { transform: scale(.98); }
.big:disabled { background: var(--grey) !important; opacity: .5; cursor: not-allowed; }
#b-start { background: var(--green); } #b-stop { background: var(--amber); }
#b-estop { background: var(--red); }
.sub { margin-top: 10px; font-size: 13px; color: var(--muted); }
.mapline { display: flex; flex-wrap: wrap; align-items: center; gap: 8px 14px; margin-top: 12px; }
.mapline label { display: inline-flex; align-items: center; gap: 6px; font-size: 14px; }
select, .btn { font: inherit; padding: 5px 8px; border-radius: 6px; border: 1px solid var(--line);
               background: var(--chip); color: var(--ink); }
.btn { cursor: pointer; padding: 6px 14px; }
.btn:disabled { opacity: .5; cursor: not-allowed; }
.grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(340px, 1fr));
        gap: 14px; margin-top: 14px; }
.hb { display: flex; gap: 16px; align-items: stretch; }
.state { flex: 0 0 170px; display: flex; align-items: center; justify-content: center;
         border-radius: 8px; color: #fff; font-weight: 800; font-size: 22px;
         background: var(--grey); text-align: center; padding: 8px; }
dl { display: grid; grid-template-columns: auto 1fr; gap: 4px 14px; margin: 0; flex: 1; }
dt { color: var(--muted); } dd { margin: 0; font-variant-numeric: tabular-nums; font-weight: 600; }
.red { color: var(--red); }
#h-up, #h-age { white-space: nowrap; }
#log { height: 190px; overflow-y: auto; font: 13px/1.5 ui-monospace, Menlo, monospace;
       background: var(--chip); border-radius: 6px; padding: 8px 10px; }
#log .bad, #log .refused { color: var(--red); }
#log .barcode { color: var(--blue); font-weight: 700; }
.bcrow { display: flex; align-items: center; gap: 14px; margin-bottom: 10px; }
.letter { flex: 0 0 64px; height: 64px; border-radius: 10px; display: flex;
          align-items: center; justify-content: center; font-size: 38px; font-weight: 800;
          background: var(--chip); color: var(--muted); transition: background .3s, color .3s; }
.letter.on { background: var(--blue); color: #fff; }
.letter.flash { background: var(--amber); color: #fff; }
.bcinfo { font-size: 14px; } .bcinfo b { font-size: 15px; }
.wide { grid-column: 1 / -1; }
#toast { position: fixed; left: 50%; top: 16px; transform: translateX(-50%);
         background: var(--red); color: #fff; padding: 12px 18px; border-radius: 8px;
         font-weight: 600; box-shadow: 0 6px 20px rgba(0,0,0,.25); display: none; z-index: 10; }
#offline { display: none; background: var(--red); color: #fff; text-align: center; padding: 8px; }
@media (max-width: 620px) {
  .controls { grid-template-columns: 1fr; }
  .hb { flex-direction: column; } .state { flex-basis: auto; min-height: 60px; }
}
</style>
</head>
<body>
<div id="offline">The dashboard program has stopped. Start it again in the terminal, then reload this page.</div>
<header>
  <h1>Robot Dashboard</h1>
  <span class="pill" id="p-broker"><span class="dot"></span><span>broker: connecting</span></span>
  <span class="pill" id="p-robot"><span class="dot"></span><span>robot: unknown</span></span>
</header>
<div id="toast"></div>
<main>
  <section class="card">
    <h2>Control</h2>
    <div class="controls">
      <button class="big" id="b-start" data-cmd="START">&#9654;&nbsp; START</button>
      <button class="big" id="b-stop" data-cmd="STOP">&#9632;&nbsp; STOP</button>
      <button class="big" id="b-estop" data-cmd="E-STOP">E-STOP&nbsp; (Esc)</button>
    </div>
    <div class="sub" id="lastcmd">No command sent yet.</div>
    <div class="mapline">
      <strong>Barcode map</strong>
      <label>forward <select id="m-forward"></select></label>
      <label>U-turn <select id="m-uturn"></select></label>
      <label>left <select id="m-left"></select></label>
      <label>right <select id="m-right"></select></label>
      <button class="btn" id="b-map">Send map</button>
    </div>
  </section>

  <div class="grid">
    <section class="card">
      <h2>Heartbeat (every 1 s)</h2>
      <div class="hb">
        <div class="state" id="h-state">—</div>
        <dl>
          <dt>Sequence</dt><dd id="h-seq">—</dd>
          <dt>Uptime</dt><dd id="h-up">—</dd>
          <dt>Link losses</dt><dd id="h-loss">—</dd>
          <dt>Last heard</dt><dd id="h-age">—</dd>
        </dl>
      </div>
    </section>
    <section class="card">
      <h2>Line &amp; Barcode (B3)</h2>
      <div class="bcrow">
        <div class="letter" id="bc-letter">–</div>
        <div class="bcinfo"><div><b>Last barcode read</b></div>
          <div id="bc-mean">No barcode read yet.</div>
          <div id="bc-when" class="sub" style="margin-top:2px"></div></div>
      </div>
      <dl>
        <dt>Line</dt><dd id="l-line">—</dd>
        <dt>Barcode reader</dt><dd id="l-bc">—</dd>
        <dt>Barcodes read</dt><dd id="l-reads">—</dd>
        <dt>Barcode error</dt><dd id="l-err">—</dd>
        <dt>Edge overflow</dt><dd id="l-ovf">—</dd>
      </dl>
    </section>
    <section class="card">
      <h2>Motion (B2)</h2>
      <dl>
        <dt>Left wheel</dt><dd id="m-l">—</dd>
        <dt>Right wheel</dt><dd id="m-r">—</dd>
        <dt>Distance counter</dt><dd id="m-d">—</dd>
        <dt>Travelled (this session)</dt><dd id="m-o">—</dd>
      </dl>
    </section>
    <section class="card">
      <h2>IMU (B4)</h2>
      <dl>
        <dt>State</dt><dd id="i-state">—</dd>
        <dt>Pitch</dt><dd id="i-pitch">—</dd>
        <dt>Heading</dt><dd id="i-hdg">—</dd>
        <dt>Turn rate</dt><dd id="i-rate">—</dd>
        <dt>Last hump</dt><dd id="i-hump">—</dd>
        <dt>Calibrated</dt><dd id="i-cal">—</dd>
        <dt>Lifted / overturned</dt><dd id="i-lifted">—</dd>
        <dt>I2C fails</dt><dd id="i-i2c">—</dd>
      </dl>
    </section>
    <section class="card">
      <h2>Obstacle Scan (B5)</h2>
      <dl>
        <dt>Raw</dt><dd id="o-raw">—</dd>
        <dt>Wi-Fi-loss bit</dt><dd id="o-wifi">—</dd>
      </dl>
    </section>
    <section class="card">
      <h2>Timing (NFR6 / NFR7 / NFR17)</h2>
      <dl>
        <dt>Telemetry (500 ms)</dt><dd id="t-tlm">—</dd>
        <dt>Heartbeat (1000 ms)</dt><dd id="t-hb">—</dd>
        <dt>Reconnects within 10 s</dt><dd id="t-rec">—</dd>
      </dl>
    </section>
    <section class="card wide">
      <h2>Event log</h2>
      <div id="log"></div>
    </section>
  </div>
</main>
<script>
const $ = id => document.getElementById(id);
const COLOURS = { RUNNING: "var(--green)", IDLE: "var(--grey)", INIT: "var(--grey)",
                  E_STOPPED: "var(--red)", FAULT: "var(--red)", COMPLETE: "var(--blue)" };
let connected = false, lastLogId = 0, firstSnap = true, toastTimer = null;

function set(id, text, red) { const e = $(id); e.textContent = text; e.classList.toggle("red", !!red); }
function pill(id, cls, text) { const p = $(id); p.className = "pill " + cls; p.lastElementChild.textContent = text; }
function toast(text) {
  const t = $("toast"); t.textContent = text; t.style.display = "block";
  clearTimeout(toastTimer); toastTimer = setTimeout(() => t.style.display = "none", 6000);
}
const signed = v => (v > 0 ? "+" : "") + v;

// letter drop-downs, defaults A F Z W
["forward", "uturn", "left", "right"].forEach((k, i) => {
  const s = $("m-" + k);
  for (let c = 65; c <= 90; c++) s.add(new Option(String.fromCharCode(c)));
  s.value = "AFZW"[i];
});

async function post(url, body) {
  try {
    const r = await fetch(url, { method: "POST", headers: { "Content-Type": "application/json" },
                                 body: JSON.stringify(body) });
    const j = await r.json();
    if (!r.ok || !j.ok) toast(j.error || "Command not sent.");
  } catch (e) { toast("The dashboard program is not running."); }
}
document.querySelectorAll(".big").forEach(b =>
  b.addEventListener("click", () => post("/api/command", { name: b.dataset.cmd })));
$("b-map").addEventListener("click", () => {
  const m = {}; ["forward", "uturn", "left", "right"].forEach(k => m[k] = $("m-" + k).value);
  post("/api/map", m);
});
document.addEventListener("keydown", e => {
  if (e.key === "Escape" && connected) { e.preventDefault(); post("/api/command", { name: "E-STOP" }); }
});

function render(s) {
  connected = s.broker === "connected";
  document.querySelectorAll(".big, #b-map").forEach(b => b.disabled = !connected);
  pill("p-broker", connected ? "ok" : s.broker === "connecting" ? "warn" : "bad",
       "broker " + s.broker_addr + ": " + s.broker);
  pill("p-robot", s.robot === "ONLINE" ? "ok" : s.robot === "OFFLINE" ? "bad" : "",
       "robot: " + s.robot);
  $("lastcmd").textContent = s.last_cmd;

  const stale = s.hb_age !== null && s.hb_age > 3;
  if (s.hb) {
    const st = $("h-state"); st.textContent = s.hb.state;
    st.style.background = stale ? "var(--grey)" : (COLOURS[s.hb.state] || "var(--grey)");
    set("h-seq", s.hb.seq);
    set("h-up", Math.floor(s.hb.uptime_s / 60) + " min " + String(s.hb.uptime_s % 60).padStart(2, "0") + " s");
    set("h-loss", s.hb.link_losses + (s.hb.link_losses === 15 ? " (max shown)" : ""));
    set("h-age", stale ? s.hb_age.toFixed(0) + " s ago (no heartbeat!)"
                       : s.hb_age < 1.5 ? "just now" : s.hb_age.toFixed(0) + " s ago", stale);
  }
  const t = s.tlm;
  if (t) {
    set("m-l", signed(t.b2.left_cm_s) + " cm/s"); set("m-r", signed(t.b2.right_cm_s) + " cm/s");
    set("m-d", t.b2.dist_mm + " mm (wraps at 4096)"); set("m-o", t.odo_cm.toFixed(1) + " cm");
    set("l-line", t.b3.line, t.b3.line === "LOST"); set("l-bc", t.b3.barcode);
    set("l-err", t.b3.bc_err ? "YES" : "no", t.b3.bc_err); set("l-ovf", t.b3.overflow ? "YES" : "no", t.b3.overflow);
    set("i-state", t.b4.state); set("i-pitch", signed(t.b4.pitch_deg.toFixed(1)) + "°");
    set("i-hdg", t.b4.heading_deg + "°"); set("i-rate", signed(t.b4.rate_dps) + "°/s");
    set("i-hump", t.b4.hump_mm + " mm (count " + t.b4.hump_count + ")");
    set("i-cal", t.b4.calibrated ? "yes" : "NO", !t.b4.calibrated);
    set("i-lifted", t.b4.lifted ? "LIFTED - motors stopped" : "no", t.b4.lifted); set("i-i2c", t.b4.i2c_fails);
    set("o-raw", t.b5 === 0 ? "0 (not sent yet)" : "0x" + t.b5.toString(16));
    set("o-wifi", t.wifi_loss ? "1 (frame held during Wi-Fi loss)" : "0");
  }
  if (s.bc.letter) {
    const L = $("bc-letter"), fresh = s.bc.age !== null && s.bc.age < 3;
    if (L.textContent !== s.bc.letter || fresh) L.textContent = s.bc.letter;
    L.className = "letter " + (fresh ? "flash" : "on");
    $("bc-mean").textContent = s.bc.meaning;
    $("bc-when").textContent = s.bc.age === null ? "read before this dashboard started"
      : s.bc.age < 5 ? "just now" : s.bc.age + " s ago";
  }
  set("l-reads", s.bc.reads + " since the dashboard started");
  const tm = s.timing, fmt = x => x ? "n=" + x.n + "  mean " + x.mean + "  max " + x.max + " ms" : "—";
  set("t-tlm", fmt(tm.tlm)); set("t-hb", fmt(tm.hb));
  if (tm.rec.n) { const pct = Math.round(100 * tm.rec.ok / tm.rec.n);
    set("t-rec", tm.rec.ok + "/" + tm.rec.n + " (" + pct + " %, need ≥ 95 %)", pct < 95); }

  const log = $("log"), atBottom = log.scrollTop + log.clientHeight >= log.scrollHeight - 4;
  for (const e of s.log) {
    if (e.id <= lastLogId) continue;
    lastLogId = e.id;
    const div = document.createElement("div"); div.className = e.kind;
    div.textContent = e.t + "  " + e.text; log.appendChild(div);
    if (e.kind === "refused" && !firstSnap) toast(e.text);
  }
  while (log.childElementCount > 300) log.firstElementChild.remove();
  if (atBottom) log.scrollTop = log.scrollHeight;
  firstSnap = false;
}

function connect() {
  const es = new EventSource("/events");
  es.onmessage = ev => { $("offline").style.display = "none"; render(JSON.parse(ev.data)); };
  es.onerror = () => {
    $("offline").style.display = "block"; connected = false;
    document.querySelectorAll(".big, #b-map").forEach(b => b.disabled = true);
  };
}
connect();
</script>
</html>
"""

if __name__ == "__main__":
    main()