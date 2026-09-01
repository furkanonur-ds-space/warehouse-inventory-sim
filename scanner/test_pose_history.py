"""
Check that a frame is placed against the pose it was taken at.

This is the fix for a scan that put two per cent of its codes on the right
shelf when the machine was fast enough to outrun the decoder. The frames were
fine and the poses were fine; they were simply paired with each other wrongly.

So the thing worth testing is the pairing, and the cases that break a lookup
like this one: a time between two samples, a time before the history starts, a
heading that wraps through north, and a queue deep enough that the vehicle has
travelled metres since the frame was taken.

No simulator, no flight.
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import scanner as s

failures = []


def check(label, got, want, tol=None):
    if tol is None:
        ok = got == want
    else:
        ok = got is not None and abs(got - want) <= tol
    print("  %-52s %s" % (label, "ok" if ok else
                          "FAILED  got %r want %r" % (got, want)))
    if not ok:
        failures.append(label)


def fill(samples):
    s.pose_history[:] = list(samples)


print("1. a time that falls between two samples is interpolated")
fill([(100.0, 0.0, 0.0, -1.0, 0.0),
      (101.0, 1.0, 2.0, -3.0, 0.0)])
got = s.pose_at(100.5)
check("north halfway", got[0], 0.5, 1e-9)
check("east halfway", got[1], 1.0, 1e-9)
check("down halfway", got[2], -2.0, 1e-9)

print("\n2. the ends behave")
check("exactly on a sample", s.pose_at(100.0)[0], 0.0, 1e-9)
check("after the last sample, holds the last", s.pose_at(200.0)[0], 1.0, 1e-9)
check("before the history starts, refuses", s.pose_at(50.0), None)
fill([])
check("empty history refuses", s.pose_at(100.0), None)

print("\n3. heading interpolates the short way round north")
fill([(100.0, 0.0, 0.0, 0.0, 170.0),
      (101.0, 0.0, 0.0, 0.0, -170.0)])
# 170 to -170 is 20 degrees through 180, not 340 the other way.
got = s.pose_at(100.5)[3]
wrapped = (got + 180.0) % 360.0 - 180.0
check("halfway between 170 and -170 is 180", abs(wrapped), 180.0, 1e-6)

fill([(100.0, 0.0, 0.0, 0.0, -90.0),
      (101.0, 0.0, 0.0, 0.0, -90.0)])
check("a steady heading stays put", s.pose_at(100.5)[3], -90.0, 1e-9)

print("\n4. a deep queue is a wait, not an error")
# The vehicle flies north up an aisle at the cruise speed for ten seconds,
# sampled at 26 Hz the way PX4 reports it.
fill([(100.0 + i / 26.0, -9.0 + s.CRUISE_SPEED * i / 26.0, -8.4, -0.5, -90.0)
      for i in range(260)])
taken_at = 101.0                      # a frame from one second in
true_north = -9.0 + s.CRUISE_SPEED * 1.0
got = s.pose_at(taken_at)
check("the pose of a one second old frame", got[0], true_north, 0.02)

# Now the same frame handled two minutes later, which is what the failing run
# was doing. The answer must not move.
s.sim_now["s"] = 100.0 + 259 / 26.0
check("same frame, handled much later, same answer",
      s.pose_at(taken_at)[0], true_north, 0.02)

# And what the old code did, for contrast: read the position now.
drifted = s.pose_history[-1][1]
print("     reading the current pose instead would have said %.2f m,"
      % drifted)
print("     which is %.2f m from where the frame was taken"
      % abs(drifted - true_north))

print("\n5. the clock the age check uses cannot come from the camera queue")
import inspect
src = inspect.getsource(s.on_tof_scan)
check("the TOF no longer sets the clock", 'sim_now["s"]' in src, False)
src = inspect.getsource(s.track_odometry)
check("odometry sets the clock", 'sim_now["s"]' in src, True)
check("odometry fills the history", "pose_history.append" in src, True)

print("\n%s" % ("all checks passed" if not failures
                else "%d FAILED: %s" % (len(failures), ", ".join(failures))))
sys.exit(1 if failures else 0)
