# Checked against the VOXL2 side before connecting it

Two things decide whether the first HITL flight rises or flips, and neither
can be seen until a real board is on the other end: which motor is which,
and what a motor command means. Both were checked on 2026-09-29 against
ModalAI's own HITL sources, branch `hitl-muorb-bridge` of
`github.com/modalai/px4-firmware`.

## Motor order and spin direction

PX4 on the board allocates thrust with the geometry in
`boards/modalai/voxl2/target/voxl-px4-hitl-set-default-parameters.config`.
The model allocates it with the rotor links and `motorNumber` in
`models/x500_voxl/model.sdf`. The two frames disagree about which way y
points, FRD to the right and FLU to the left, which is exactly where a
mismatch would hide.

| motor | PX4 CA_ROTOR (FRD) | position | KM | model rotor (FLU) | position | direction |
|---|---|---|---|---|---|---|
| 0 | +0.174, +0.174 | front right | +0.05, CCW | rotor_0 +0.174, -0.174 | front right | ccw |
| 1 | -0.174, -0.174 | rear left | +0.05, CCW | rotor_1 -0.174, +0.174 | rear left | ccw |
| 2 | +0.174, -0.174 | front left | -0.05, CW | rotor_2 +0.174, +0.174 | front left | cw |
| 3 | -0.174, +0.174 | rear right | -0.05, CW | rotor_3 -0.174, -0.174 | rear right | cw |

A positive KM is CCW, from PX4's own parameter description:
"Use a positive value for a rotor with CCW rotation."

`HIL_ACT_FUNC1..4` are 101..104, Motor1 to Motor4, so `controls[0..3]` in
HIL_ACTUATOR_CONTROLS are CA rotors 0 to 3. The bridge puts `controls[i]`
into velocity `i` of the Actuators message, and the motor model reads the
index it is given by `motorNumber`. All four agree.

## What a motor command means

HIL_ACTUATOR_CONTROLS is filled straight from `actuator_outputs_sim`
(`src/modules/mavlink/streams/HIL_ACTUATOR_CONTROLS.hpp`), which
`pwm_out_sim` publishes (`src/modules/simulation/pwm_out_sim/PWMSim.cpp`):

- a non-reversible motor is scaled to **0 to 1**: `(pwm - 1000) / 1000`
- anything else is scaled to -1 to 1; a quad has nothing else on 0 to 3
- a disarmed output is left at **0**, the zero of a freshly initialised
  message, rather than set to anything

The bridge clamps each command to 0 to 1 and turns anything that is not a
finite number into 0, then multiplies by the model's `maxRotVelocity`. That
is the same range, and a disarmed vehicle gets stopped rotors.

`flags` is `0x0F` when armed and the vehicle is a quad, as ModalAI's
document says; the bridge does not rely on it.

## Safety note for the bench

`voxl-px4-hitl-start` starts `pwm_out_sim -m hil`, not the ESC driver, so
the board should not drive the real motors. The parameter file does map
`PWM_MAIN_FUNC1..4` to the motors as well, and a board that was not started
through that script, or a later version of it, could behave differently.
Propellers stay off for every HITL session regardless.

## Sources

- `boards/modalai/voxl2/target/voxl-px4-hitl-set-default-parameters.config`
- `boards/modalai/voxl2/target/voxl-px4-hitl-start`
- `src/modules/mavlink/streams/HIL_ACTUATOR_CONTROLS.hpp`
- `src/modules/simulation/pwm_out_sim/PWMSim.cpp` and `.hpp`
- `src/modules/control_allocator/module.yaml` in PX4, for the sign of KM

All on `modalai/px4-firmware`, branch `hitl-muorb-bridge`, except the last,
which is upstream PX4 on this machine.
