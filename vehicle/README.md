# vehicle

The part of the scan that is meant to run on the drone, in C.

The vehicle runs ModalAI's VOXL2 stack: services written in C, talking over
pipes, with `meshine` as the mission service. Python is not part of that
world, so what travels there is this folder. `scanner/` stays where it is and
keeps doing what it is good at: being the test bench, in a language where a
warehouse can be changed and reflown in an afternoon.

## What is here

    route.h / route.c     where the lanes are and in what order they are flown
    flight.h / flight.c   how a leg is flown: setpoints and yaw sweeps
    main.c                prints the route, for the comparison test
    flight_tool.c         prints the setpoints, for the same
    tests/legs.json       the legs and turns both languages are asked to fly
    third_party/cJSON.*   JSON parsing, MIT, vendored

cJSON is vendored rather than installed because `meshine` reads its own
mission files through ModalAI's `modal_json`, which is a wrapper around
cJSON. Using the same object model means the code that moves to the drone
does not have to be rewritten around a different parser, and it builds here
with no sudo and no package.

## Building and testing

    ./scripts/build.sh
    ./tests/compare_with_python.sh

The test is the reason this port can be trusted at all: the same
`layout.json` has to give the same waypoints in both languages, compared
value by value.

    PASS both languages give the same 24 waypoints

## What the test compares, and why it compares three things

    ok   the same 24 waypoints
    ok   the same 4 remarks about the route
    ok   the same 8 camera standoffs and frame limits
    ok   the same 671 setpoints and yaw steps

The waypoints are the point. The other two are there because the waypoints
alone were not enough, which was found rather than assumed.

Five deliberate mistakes were introduced one at a time, each a plausible
slip when porting. Four were caught by the waypoints alone: rounding to two
decimals instead of three, taking the bottom of a code band instead of its
middle, and forgetting either of the two alternations that make the flight a
boustrophedon.

The fifth was not. Moving `HIRES_MOUNT_X` from 0.06 to 0.07 changed nothing
visible: the mount offsets and the lens reach only the warning about a band
of codes too tall to frame, and in this warehouse every band fits, so no
warning appears either way. Comparing the remarks did not help for the same
reason. What closed it was printing the numbers themselves, which is what
`--limits` is for. With that comparison in place the same centimetre of
error now fails, and names the lane and the value.

A note for whoever changes this next: the first attempt at that check also
passed, because the constant had been moved to route.h and the test was
still editing route.c. A negative test that edits the wrong file proves
nothing. Check that the file really changed before believing the result.

The flight comparison was checked the same way, against four slips, and
caught all four: the climb speed, the timeout margin, rounding the number of
yaw steps instead of truncating it, and using plain `fmod` where Python's
`%` was meant.

That last one is the reason the file says so much about a modulo. Python's
`%` takes the sign of the divisor and C's `fmod` takes the sign of the
dividend, so a turn from 170 to -170 degrees comes out as 20 degrees in
Python and -340 in C. The vehicle would have turned the long way round, in
an aisle, at 30 degrees a second.

## What is not covered

None of this has flown since the split. The comparisons cover what can be
worked out in advance: the same route, the same setpoints, the same turns.
They say nothing about whether the vehicle settles, what the markers
correct, or whether 432 codes still come back. That needs a scan, and a scan
takes seventeen minutes.

## What is not here yet

The flight itself: streaming position and heading setpoints to PX4, which is
what the warehouse scan needs and what `voxl-vision-hub`'s own offboard
modes cannot do, since they turn the nose along the direction of travel. See
`~/not_yaw_hitl.md`.
