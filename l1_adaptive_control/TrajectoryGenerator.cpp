#include "TrajectoryGenerator.hpp"

#include <math.h>
#include <mathlib/mathlib.h>

namespace
{

static constexpr float PI_F = 3.14159265358979323846f;
static constexpr float TWO_PI_F = 2.f * PI_F;

void copy3(const float in[3], float out[3])
{
out[0] = in[0];
out[1] = in[1];
out[2] = in[2];
}

}

bool TrajectoryGenerator::update(const Input &input, Output &output)
{
_last_input = input;

output.timestamp_us = input.timestamp_us;

set_zero_derivatives(output);

// Safety gate:
// If the PX4 state is not valid for control or the vehicle is disarmed, only
// mirror current position for debug and reset the trajectory start point.
// Do not generate a valid trajectory.
if (!input.state_valid_for_control || !input.armed || input.failsafe) {
reset();

output.position_ned[0] = input.current_position_ned[0];
output.position_ned[1] = input.current_position_ned[1];
output.position_ned[2] = input.current_position_ned[2];

output.yaw = input.current_yaw;
output.elapsed_time_s = 0.f;
output.mode = Mode::WaitForValidState;
output.valid = false;

_last_output = output;
return true;
}

// Initialize trajectory start point on the first valid state.
if (!_initialized) {
_start_time_us = input.timestamp_us;
_last_update_us = input.timestamp_us;

_start_position_ned[0] = input.current_position_ned[0];
_start_position_ned[1] = input.current_position_ned[1];
_start_position_ned[2] = input.current_position_ned[2];

_takeoff_target_position_ned[0] = _start_position_ned[0];
_takeoff_target_position_ned[1] = _start_position_ned[1];

// NED convention:
// z becomes more negative when the vehicle moves upward.
_takeoff_target_position_ned[2] = _start_position_ned[2] - TAKEOFF_HEIGHT_M;
copy3(_takeoff_target_position_ned, _hover_position_ned);
copy3(_hover_position_ned, _circle_center_position_ned);

_start_yaw = input.current_yaw;

_initialized = true;
}

const float elapsed_s = math::max((input.timestamp_us - _start_time_us) * 1e-6f, 0.f);
output.elapsed_time_s = elapsed_s;

output.yaw = _start_yaw;
output.yaw_rate = 0.f;
output.yaw_accel = 0.f;

if (elapsed_s < TAKEOFF_DURATION_S) {
output.mode = Mode::Takeoff;
_manual_hold_initialized = false;

const float s = math::constrain(elapsed_s / TAKEOFF_DURATION_S, 0.f, 1.f);

// Smooth cubic trajectory:
// h(s) = 3s^2 - 2s^3
// h_dot = (6s - 6s^2) / T
// h_ddot = (6 - 12s) / T^2
// h_jerk = -12 / T^3
const float h = 3.f * s * s - 2.f * s * s * s;
const float h_dot = (6.f * s - 6.f * s * s) / TAKEOFF_DURATION_S;
const float h_ddot = (6.f - 12.f * s) / (TAKEOFF_DURATION_S * TAKEOFF_DURATION_S);
const float h_jerk = -12.f / (TAKEOFF_DURATION_S * TAKEOFF_DURATION_S * TAKEOFF_DURATION_S);

for (int i = 0; i < 3; i++) {
const float delta = _takeoff_target_position_ned[i] - _start_position_ned[i];

output.position_ned[i] = _start_position_ned[i] + delta * h;
output.velocity_ned[i] = delta * h_dot;
output.acceleration_ned[i] = delta * h_ddot;
output.jerk_ned[i] = delta * h_jerk;
output.snap_ned[i] = 0.f;
}

} else {
if (_commanded_mode == CommandedMode::Circle) {
update_circle_target(input, output);

} else {
update_hover_target(input, output);
}
}

_last_update_us = input.timestamp_us;
output.valid = true;

_last_output = output;

return true;
}

void TrajectoryGenerator::reset()
{
_initialized = false;
_start_time_us = 0;
_last_update_us = 0;
_manual_hold_initialized = false;
reset_circle_state();
}

void TrajectoryGenerator::set_commanded_mode(CommandedMode mode)
{
if (_commanded_mode == mode) {
return;
}

_commanded_mode = mode;
reset_circle_state();
}

float TrajectoryGenerator::circle_period_s() const
{
if (CIRCLE_SPEED_M_S < 1e-5f || CIRCLE_RADIUS_M < 1e-5f) {
return 0.f;
}

return TWO_PI_F * CIRCLE_RADIUS_M / CIRCLE_SPEED_M_S;
}

void TrajectoryGenerator::set_zero_derivatives(Output &output)
{
for (int i = 0; i < 3; i++) {
output.velocity_ned[i] = 0.f;
output.acceleration_ned[i] = 0.f;
output.jerk_ned[i] = 0.f;
output.snap_ned[i] = 0.f;
}

output.yaw_rate = 0.f;
output.yaw_accel = 0.f;
}

void TrajectoryGenerator::set_hold_position(Output &output, const float position_ned[3])
{
output.position_ned[0] = position_ned[0];
output.position_ned[1] = position_ned[1];
output.position_ned[2] = position_ned[2];

set_zero_derivatives(output);
}

void TrajectoryGenerator::update_manual_hold_target(const Input &input, Output &output)
{
const float target_vz_ned = update_manual_height_reference(input);

output.position_ned[0] = _manual_hold_position_ned[0];
output.position_ned[1] = _manual_hold_position_ned[1];
output.position_ned[2] = _manual_hold_position_ned[2];

set_zero_derivatives(output);
output.velocity_ned[2] = target_vz_ned;
sync_hover_reference_from_output(output);
}

float TrajectoryGenerator::update_manual_height_reference(const Input &input)
{
if (!_manual_hold_initialized) {
_manual_hold_position_ned[0] = _hover_position_ned[0];
_manual_hold_position_ned[1] = _hover_position_ned[1];
_manual_hold_position_ned[2] = _hover_position_ned[2];
_manual_hold_initialized = true;
_last_update_us = input.timestamp_us;
}

const float dt = math::constrain((input.timestamp_us - _last_update_us) * 1e-6f, 0.f, 0.1f);
float stick = input.manual_height_control_valid ? math::constrain(input.manual_height_stick, -1.f, 1.f) : 0.f;

if (fabsf(stick) < MANUAL_HEIGHT_DEADZONE) {
stick = 0.f;
}

const float target_vz_ned = -stick * MANUAL_MAX_CLIMB_RATE_M_S;
_manual_hold_position_ned[2] += target_vz_ned * dt;

const float min_z_ned = _start_position_ned[2] - MANUAL_MAX_HEIGHT_M;
const float max_z_ned = _start_position_ned[2] - MANUAL_MIN_HEIGHT_M;
_manual_hold_position_ned[2] = math::constrain(_manual_hold_position_ned[2], min_z_ned, max_z_ned);

_hover_position_ned[2] = _manual_hold_position_ned[2];

return target_vz_ned;
}

void TrajectoryGenerator::update_hover_target(const Input &input, Output &output)
{
output.mode = Mode::Hover;
reset_circle_state();

if (input.manual_height_control_enabled) {
update_manual_hold_target(input, output);

} else {
_manual_hold_initialized = false;
set_hold_position(output, _takeoff_target_position_ned);
sync_hover_reference_from_output(output);
}
}

void TrajectoryGenerator::update_circle_target(const Input &input, Output &output)
{
output.mode = Mode::Circle;

const float radius = CIRCLE_RADIUS_M;
const float speed_m_s = CIRCLE_SPEED_M_S;
float target_vz_ned = 0.f;

if (radius < 1e-5f || speed_m_s < 1e-5f) {
set_hold_position(output, _hover_position_ned);
return;
}

if (!_circle_initialized) {
copy3(_hover_position_ned, _circle_center_position_ned);
_circle_current_speed_rad_s = speed_m_s / radius;
_circle_time_offset_s = output.elapsed_time_s;
_circle_current_loop_time_s = TWO_PI_F / _circle_current_speed_rad_s;
_circle_acc_complete = true;
_circle_initialized = true;
}

if (input.manual_height_control_enabled) {
target_vz_ned = update_manual_height_reference(input);
_circle_center_position_ned[2] = _hover_position_ned[2];

} else {
_manual_hold_initialized = false;
_hover_position_ned[2] = _takeoff_target_position_ned[2];
_circle_center_position_ned[2] = _hover_position_ned[2];
}

const float t = math::max(input.timestamp_us >= _start_time_us ?
			 (input.timestamp_us - _start_time_us) * 1e-6f - _circle_time_offset_s : 0.f, 0.f);
const float w = _circle_current_speed_rad_s;
const float theta = w * t;
const float s = sinf(theta);
const float c = cosf(theta);

output.position_ned[0] = _circle_center_position_ned[0] + radius * c;
output.position_ned[1] = _circle_center_position_ned[1] + radius * s;
output.position_ned[2] = _circle_center_position_ned[2];

output.velocity_ned[0] = -radius * w * s;
output.velocity_ned[1] = radius * w * c;
output.velocity_ned[2] = target_vz_ned;

output.acceleration_ned[0] = -radius * w * w * c;
output.acceleration_ned[1] = -radius * w * w * s;
output.acceleration_ned[2] = 0.f;

output.jerk_ned[0] = radius * w * w * w * s;
output.jerk_ned[1] = -radius * w * w * w * c;
output.jerk_ned[2] = 0.f;

output.snap_ned[0] = radius * w * w * w * w * c;
output.snap_ned[1] = radius * w * w * w * w * s;
output.snap_ned[2] = 0.f;

if (_circle_yaw_mode == CircleYawMode::Fixed) {
output.yaw = _start_yaw;
output.yaw_rate = 0.f;
output.yaw_accel = 0.f;
}
}

void TrajectoryGenerator::reset_circle_state()
{
_circle_initialized = false;
_circle_current_speed_rad_s = 0.f;
_circle_time_offset_s = 0.f;
_circle_current_loop_time_s = 0.f;
_circle_acc_complete = false;
}

void TrajectoryGenerator::sync_hover_reference_from_output(const Output &output)
{
_hover_position_ned[0] = output.position_ned[0];
_hover_position_ned[1] = output.position_ned[1];
_hover_position_ned[2] = output.position_ned[2];
}
