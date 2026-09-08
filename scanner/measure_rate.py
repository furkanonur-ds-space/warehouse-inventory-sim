#!/usr/bin/env python3
"""
What the simulator actually delivers, without flying anything.

    python3 scanner/measure_rate.py            30 seconds
    python3 scanner/measure_rate.py 60         longer

Subscribes to the two scanning cameras on a simulator that is already running
and reports, per camera, the rate in simulated time (what the model asks for),
the rate on the wall clock (what the machine manages) and the ratio, which is
the real time factor.

It answers one question: if the camera is asked for more frames, does the
machine give them, and what does that cost in minutes. In lockstep a slower
machine slows simulated time as well, so the vehicle sees the same frames per
metre either way and only the waiting gets longer. That is worth knowing
before a change is made rather than after a flight.
"""
import json, os, sys, time, threading

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import gz.transport13 as trans
from gz.msgs10.image_pb2 import Image

HERE = os.path.dirname(os.path.abspath(__file__))
LAYOUT = json.load(open(os.path.join(HERE, "layout.json"), encoding="utf-8"))
BASE = "/world/%s/model/%s_0/link" % (LAYOUT["world"], LAYOUT["model"])
CAMS = {"hires": BASE + "/camera_hires_link/sensor/camera/image",
        "rear": BASE + "/camera_track_rear_link/sensor/camera/image"}

seconds = float(sys.argv[1]) if len(sys.argv) > 1 else 30.0
state = dict((n, {"sim": [], "wall": [], "last_sim": None, "last_wall": None})
             for n in CAMS)
lock = threading.Lock()


def make(name):
    def handler(msg):
        sim = msg.header.stamp.sec + msg.header.stamp.nsec / 1e9
        wall = time.time()
        with lock:
            s = state[name]
            if s["last_sim"] is not None:
                s["sim"].append(sim - s["last_sim"])
                s["wall"].append(wall - s["last_wall"])
            s["last_sim"], s["last_wall"] = sim, wall
    return handler


nodes = []
for name, topic in CAMS.items():
    node = trans.Node()
    if not node.subscribe(Image, topic, make(name)):
        print("could not subscribe to %s" % topic)
    nodes.append(node)

print("listening for %.0f s. Is the simulator up?" % seconds)
time.sleep(seconds)

print()
print("%-8s %8s %10s %10s %8s" % ("camera", "frames", "sim Hz", "wall Hz", "RTF"))
for name in CAMS:
    s = state[name]
    if not s["sim"]:
        print("%-8s %8s  nothing arrived" % (name, 0))
        continue
    sim = sorted(s["sim"])[len(s["sim"]) // 2]
    wall = sorted(s["wall"])[len(s["wall"]) // 2]
    print("%-8s %8d %10.2f %10.2f %8.2f"
          % (name, len(s["sim"]) + 1, 1 / sim, 1 / wall, sim / wall))

print()
print("sim Hz is what the model asks for and what the vehicle sees per metre.")
print("wall Hz is what the machine manages. RTF is the ratio: a 600 s flight")
print("takes 600 / RTF seconds to sit through.")
