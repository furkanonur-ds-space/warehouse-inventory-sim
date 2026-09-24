# vehicle

The part of the scan that is meant to run on the drone, in C.

The vehicle runs ModalAI's VOXL2 stack: services written in C, talking over
pipes, with `meshine` as the mission service. Python is not part of that
world, so what travels there is this folder. `scanner/` stays where it is and
keeps doing what it is good at: being the test bench, in a language where a
warehouse can be changed and reflown in an afternoon.

## What is here

    route.h / route.c     where the lanes are and in what order they are flown
    main.c                prints the route, for the comparison test
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

## What the test does and does not cover

It was checked against four deliberate mistakes, each a plausible slip when
porting, and it caught all four: rounding to two decimals instead of three,
taking the bottom of a code band instead of its middle, and forgetting either
of the two alternations that make the flight a boustrophedon.

It also missed one. Changing `HIRES_MOUNT_X` from 0.06 to 0.07 left the
comparison passing, because the mount offsets only reach the warning about a
band that cannot fit the frame, never the waypoints themselves. So the
warnings are not compared, and anything that only affects them is outside
this test. Worth knowing before trusting a green result too far: it says the
route agrees, not that every number in the file does.

## What is not here yet

The flight itself: streaming position and heading setpoints to PX4, which is
what the warehouse scan needs and what `voxl-vision-hub`'s own offboard
modes cannot do, since they turn the nose along the direction of travel. See
`~/not_yaw_hitl.md`.
