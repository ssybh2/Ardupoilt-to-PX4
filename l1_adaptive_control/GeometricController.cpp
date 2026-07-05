#include "GeometricController.hpp"

bool GeometricController::update(const Input &input, Output &output)
{
_last_input = input;

output.timestamp_us = input.timestamp_us;

// Stage 3 placeholder:
// No real geometric control calculation is performed yet.
// This file only defines the future controller interface.
output.thrust_newton = 0.f;
output.moment_newton_meter[0] = 0.f;
output.moment_newton_meter[1] = 0.f;
output.moment_newton_meter[2] = 0.f;

// For safety, the empty controller never produces a valid control output.
output.valid = false;

_last_output = output;

// Return true only means the function executed.
// It does NOT mean output is safe for actuator control.
return input.state_valid_for_control && !input.failsafe;
}
