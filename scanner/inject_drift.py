"""
Give the simulated VIO an error, so that correcting it means something.

In simulation the position estimate comes from Gazebo's OdometryPublisher,
which reports the model's true pose. The simulated VIO is therefore exact and
never drifts, the ArUco correction has nothing to correct, and "the drift
correction works" is a claim that cannot be checked either way. Testing a lane
keeping assist on a straight road in still air tells you nothing about the
assist.

This sits between the simulator and PX4 and makes the estimate wrong on
purpose:

    OdometryPublisher -> .../odometry_with_covariance_true
                              |
                          this script, adds an accumulating error
                              |
                         .../odometry_with_covariance -> PX4

PX4's gz bridge subscribes to exactly /model/<model>/odometry_with_covariance,
built from the model name in GZBridge.cpp, so moving the plugin off that topic
is what makes room here. Build the model with

    python3 build_c27_drone.py --drift

and nothing will fly until this is running, because PX4 then has no position
source at all. That is deliberate: a test harness the vehicle depends on
should be impossible to forget rather than quietly optional.

Why here and not in the scanner. Biasing what the scanner reads does move the
vehicle, through the lateral check in goto_waypoint's settle loop, but only as
far as that check's tolerance and timeout allow, and it leaves PX4's estimator
being fed the truth. On the vehicle it is the estimator that is wrong, and
every setpoint lands displaced because of it. Putting the error here also
means EKF2 is fusing a drifting vision source, which is the thing that will
actually be happening on the aircraft.

    python3 inject_drift.py                     one per cent of distance
    python3 inject_drift.py --rate 0.02         two per cent
    python3 inject_drift.py --rate 0            a pass through, to prove
                                                this script is not itself
                                                what breaks a run
    python3 inject_drift.py --seed 7            a different but repeatable
                                                random walk

Writes out/drift_injected.csv: the true position, what PX4 was told, and the
error at that moment, so a run can be scored against what it was actually
given rather than against what it was meant to be given.
"""
import os

os.environ["PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION"] = "python"

import argparse
import json
import math
import random
import signal
import sys
import threading
import time

import gz.transport13 as trans
from gz.msgs10.odometry_with_covariance_pb2 import OdometryWithCovariance

HERE = os.path.dirname(os.path.abspath(__file__))
LAYOUT_PATH = os.path.join(HERE, "layout.json")
with open(LAYOUT_PATH, encoding="utf-8") as handle:
    LAYOUT = json.load(handle)

# PX4 builds its topic from the name of the spawned entity, which carries the
# instance suffix: GZBridge.cpp asks for /model/<model_name>/odometry_with_
# covariance, and <model_name> is x500_c27_0 rather than x500_c27, the same
# suffix the camera topics carry. Getting this wrong is silent: the relay
# publishes to a topic nobody reads and PX4 waits forever for a position.
MODEL = LAYOUT["model"] + "_0"
PX4_TOPIC = "/model/%s/odometry_with_covariance" % MODEL

# What the plugin is moved onto by build_c27_drone.py --drift. A literal name
# rather than one built from the entity, because the model file is written
# before anything is spawned and does not know the suffix.
TRUE_TOPIC = "/model/%s/odometry_with_covariance_true" % LAYOUT["model"]


class Drift:
    """
    An error that grows with distance travelled, in a direction that turns
    slowly.

    Distance and not time, because that is how visual odometry fails. A
    vehicle sitting still accumulates nothing, having no motion to misjudge;
    one that has flown two hundred metres has had two hundred metres of small
    misjudgements to add up. Published VIO figures are quoted the same way, as
    a percentage of path length.

    The direction has to persist, and getting that wrong was the first attempt
    here. Drawing a fresh direction every sample makes the steps cancel: with
    odometry at 50 Hz the vehicle moves 0.02 m between messages, and a random
    walk of those spreads as rate times the square root of distance times step
    length. Over the 216 m of a scan that came to 0.018 m of error where one
    per cent of distance should be 2.16 m, a hundred times too small. It would
    have flown a whole test that proved nothing.

    So the error grows by rate times distance along a heading, and the heading
    itself wanders with a correlation length. Over anything shorter than that
    length the error accumulates coherently, which is the real thing: a scale
    or heading error in the estimator leans one way for a while. Over longer
    distances the direction decorrelates and the growth turns back into a
    square root, which is also the real thing.

    Vertical drift is deliberately much smaller. The vehicle has a downward
    facing camera and a floor to look at, and height is the axis VIO holds
    best; making it as loose as the horizontal ones would model a failure that
    does not happen.

    The walk advances in fixed steps of distance rather than once per message,
    so that a seed names one error and not one error per machine. The first
    version drew a number every time odometry arrived, and odometry does not
    arrive at a fixed rate: four runs over the same 256 m path logged 2870,
    1872, 2000 and 1978 messages. Same seed, four different errors, and no way
    to fly the same drift twice. Distance is the thing both runs share.
    """

    VERTICAL_SHARE = 0.2

    # How far the vehicle flies between two draws. Small enough that the walk
    # is smooth at the 18 m between floor markers, large enough not to matter:
    # a 256 m scan is five thousand of them.
    QUANTUM_M = 0.05

    def __init__(self, rate, seed, correlation_m=20.0):
        self.rate = rate
        # How far the vehicle flies before the direction of the error has
        # substantially changed. Twenty metres is a little longer than the
        # 18 m between one floor marker and the next, so the error between two
        # fixes is mostly coherent, which is the case the correction has to
        # handle.
        self.correlation_m = correlation_m
        self.random = random.Random(seed)
        self.heading = self.random.uniform(0, 2 * math.pi)
        self.error = [0.0, 0.0, 0.0]
        self.last = None
        self.travelled = 0.0
        self.unspent = 0.0

    def step(self, x, y, z):
        """Advance the error for a move to this true position."""
        if self.last is None:
            self.last = (x, y, z)
            return self.error
        moved = math.dist((x, y, z), self.last)
        self.last = (x, y, z)
        if moved <= 0.0:
            return self.error
        self.travelled += moved

        # Distance is banked and spent in whole quanta, so the draws depend on
        # how far the vehicle has flown and not on how often it said so. What
        # is left over waits for the next message.
        self.unspent += moved
        while self.unspent >= self.QUANTUM_M:
            self.unspent -= self.QUANTUM_M
            # The heading wanders as the square root of distance, which is
            # what keeps a long path from being a straight lean.
            self.heading += self.random.gauss(0, 1) * math.sqrt(
                self.QUANTUM_M / self.correlation_m)
            grown = self.rate * self.QUANTUM_M
            self.error[0] += grown * math.cos(self.heading)
            self.error[1] += grown * math.sin(self.heading)
            self.error[2] += (grown * self.VERTICAL_SHARE
                              * self.random.gauss(0, 1))
        return self.error


def main():
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--rate", type=float, default=0.01,
                        help="metres of error added per metre flown, "
                             "0.01 being one per cent (default)")
    parser.add_argument("--seed", type=int, default=1,
                        help="so a run can be repeated exactly")
    parser.add_argument("--out", default=os.path.join(HERE, "..", "out",
                                                      "drift_injected.csv"))
    args = parser.parse_args()

    drift = Drift(args.rate, args.seed)
    node = trans.Node()
    publisher = node.advertise(PX4_TOPIC, OdometryWithCovariance)
    if not publisher.valid():
        raise SystemExit("could not advertise %s" % PX4_TOPIC)

    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    log = open(args.out, "w", encoding="utf-8")
    log.write("sim_time,true_x,true_y,true_z,told_x,told_y,told_z,"
              "error_m,travelled_m\n")

    counted = {"n": 0, "logged": 0}
    started = time.time()

    def relay(msg):
        pose = msg.pose_with_covariance.pose.position
        error = drift.step(pose.x, pose.y, pose.z)

        # The message is passed on with its own header and covariance intact,
        # so what reaches PX4 differs from the truth in the position and in
        # nothing else.
        out = OdometryWithCovariance()
        out.CopyFrom(msg)
        moved = out.pose_with_covariance.pose.position
        moved.x = pose.x + error[0]
        moved.y = pose.y + error[1]
        moved.z = pose.z + error[2]
        publisher.publish(out)

        counted["n"] += 1
        # One line in ten. The topic runs at 50 Hz and a flight is ten
        # minutes; the error moves slowly enough that five a second says
        # everything about it.
        if counted["n"] % 10 == 0:
            stamp = msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9
            log.write("%.3f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.2f\n"
                      % (stamp, pose.x, pose.y, pose.z,
                         moved.x, moved.y, moved.z,
                         math.hypot(error[0], error[1]), drift.travelled))
            counted["logged"] += 1
            if counted["logged"] % 250 == 0:
                log.flush()

    if not node.subscribe(OdometryWithCovariance, TRUE_TOPIC, relay):
        raise SystemExit("could not subscribe to %s" % TRUE_TOPIC)

    # Subscribing succeeds whether or not anything is publishing, so it says
    # nothing about whether the model was built with --drift. Wait for a
    # message and say so plainly if none comes: the failure to catch here is a
    # relay that runs perfectly against a topic nobody writes to, while PX4
    # sits on the other side with no position and the flight never arms.
    for _ in range(100):
        if counted["n"]:
            break
        time.sleep(0.1)
    if not counted["n"]:
        print("\nnothing arrived on %s in ten seconds." % TRUE_TOPIC)
        print("Either the simulator is not up yet, or the model was built")
        print("without --drift and is publishing straight to PX4. Check with")
        print("  gz topic -l | grep odometry")
        print("Waiting anyway; this is a warning and not an exit.")

    print("relaying %s" % TRUE_TOPIC)
    print("      to %s" % PX4_TOPIC)
    if args.rate == 0:
        print("rate 0: passing the truth through unchanged, which is the "
              "control for this script itself")
    else:
        print("adding %.1f per cent of distance flown as error, seed %d"
              % (args.rate * 100, args.seed))
    print("logging to %s" % os.path.normpath(args.out))
    print("PX4 has no other position source while the model is built with "
          "--drift, so leave this running for the whole flight.")

    stop = threading.Event()

    def finish(signum, frame):
        stop.set()

    signal.signal(signal.SIGINT, finish)
    signal.signal(signal.SIGTERM, finish)

    last_report = 0.0
    while not stop.is_set():
        time.sleep(0.5)
        if time.time() - last_report > 30 and counted["n"]:
            last_report = time.time()
            print("  %.0f s: %d messages, %.1f m flown, error %.3f m"
                  % (time.time() - started, counted["n"], drift.travelled,
                     math.hypot(drift.error[0], drift.error[1])))

    log.close()
    print("\n%d messages relayed, %.1f m flown, final error %.3f m"
          % (counted["n"], drift.travelled,
             math.hypot(drift.error[0], drift.error[1])))
    print("wrote %s" % os.path.normpath(args.out))


if __name__ == "__main__":
    main()
