#!/usr/bin/env python3
"""
s1_dashboard.py - Buddy 1 (subsystem 1) dashboard: decodes the robot's binary
MQTT messages, sends binary commands, and measures NFR6 / NFR7 / NFR17.

Bit layouts match app_program/subsystem1/s1_codec.h (MSB-first, 4-bit header:
2-bit message type, then 2-bit version).

  pip install paho-mqtt
  python s1_dashboard.py --broker 192.168.1.10              # live view, Ctrl+C = summary
  python s1_dashboard.py --broker 192.168.1.10 --send START # START / STOP / E-STOP
"""
import argparse
import csv
import random
import statistics
import time

import paho.mqtt.client as mqtt

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
    return {"seq": b.get(8), "state": STATES[b.get(3)],
            "uptime_s": b.get(16), "link_losses": b.get(4)}


def decode_ack(data):
    if len(data) != 2:
        return None
    b = Bits(data)
    t, v = header(b)
    if (t, v) != (T_ACK, VERSION):
        return None
    return {"result": "FAILED" if b.get(1) else "INVALID", "id": b.get(6)}


def encode_command(name, cmd_id):
    """2 bytes: type(2) version(2) command(2) id(6) pad(4)"""
    v = (T_COMMAND << 14) | (VERSION << 12) | (CMD[name] << 10) | ((cmd_id & 0x3F) << 4)
    return v.to_bytes(2, "big")


# ---------------------------------------------------------------- live view
gaps = {"telemetry": [], "heartbeat": []}
last = {"telemetry": None, "heartbeat": None, "offline_at": None}
reconnects = []


def on_connect(client, userdata, flags, rc, *args):
    print(f"[dashboard] connected to broker (rc={rc})")
    client.subscribe("robot/#", qos=1)


def on_message(client, userdata, msg):
    now = time.time()
    data = msg.payload
    userdata["log"].writerow([f"{now:.3f}", msg.topic, len(data), data.hex()])
    userdata["file"].flush()

    if msg.topic == "robot/status":
        text = data.decode(errors="replace")
        print(f"[status] {text}")
        if text == "OFFLINE":
            last["offline_at"] = now
        elif text == "ONLINE" and last["offline_at"]:
            dt = now - last["offline_at"]
            reconnects.append(dt)
            print(f"[NFR7] back ONLINE {dt:.2f} s after the broker marked it OFFLINE")
            last["offline_at"] = None
        return

    kind = msg.topic.split("/")[-1]
    if kind in gaps:
        if last[kind] is not None:
            gaps[kind].append(now - last[kind])
        last[kind] = now

    if kind == "telemetry":
        d = decode_telemetry(data)
        print(f"[telemetry] {len(data)} B  " + (str(d) if d else f"UNDECODABLE {data.hex()}"))
    elif kind == "heartbeat":
        d = decode_heartbeat(data)
        print(f"[heartbeat] {len(data)} B  " + (str(d) if d else f"UNDECODABLE {data.hex()}"))
    elif kind == "ack":
        d = decode_ack(data)
        print(f"[ack] {d if d else data.hex()}")


def summary():
    print("\n===== summary =====")
    for kind, target in (("telemetry", 500), ("heartbeat", 1000)):
        g = gaps[kind]
        if len(g) > 1:
            print(f"{kind}: n={len(g)} mean={statistics.mean(g) * 1000:.0f} ms "
                  f"max={max(g) * 1000:.0f} ms (target {target} ms)")
    if reconnects:
        ok = sum(1 for r in reconnects if r <= 10.0)
        print(f"reconnects: {len(reconnects)}, within 10 s: {ok} "
              f"({100 * ok / len(reconnects):.0f} %, NFR7 needs >= 95 %)")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--broker", required=True)
    ap.add_argument("--port", type=int, default=1883)
    ap.add_argument("--send", choices=list(CMD))
    ap.add_argument("--id", type=int, help="command id 0..63 (default: random)")
    ap.add_argument("--log", default="s1_mqtt_log.csv")
    a = ap.parse_args()

    f = open(a.log, "a", newline="")
    userdata = {"log": csv.writer(f), "file": f}
    try:
        client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, userdata=userdata)
    except AttributeError:  # paho-mqtt < 2.0
        client = mqtt.Client(userdata=userdata)
    client.on_connect = on_connect
    client.on_message = on_message
    client.connect(a.broker, a.port, keepalive=30)

    if a.send:
        cmd_id = a.id if a.id is not None else random.randrange(64)
        payload = encode_command(a.send, cmd_id)
        client.loop_start()
        client.publish("robot/cmd", payload, qos=1).wait_for_publish()
        print(f"[dashboard] sent {a.send} id={cmd_id} bytes={payload.hex()}")
        time.sleep(1)
        client.loop_stop()
        f.close()
        return

    try:
        client.loop_forever()
    except KeyboardInterrupt:
        summary()
    finally:
        f.close()


if __name__ == "__main__":
    main()