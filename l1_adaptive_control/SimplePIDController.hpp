#pragma once

#include <drivers/drv_hrt.h>

#include <stdint.h>

// Simple PID baseline controller for the L1 adaptive PX4 module.
//
// The controller intentionally avoids SO(3)/SE(3) geometric attitude errors.
// It uses:
//   position/velocity PD -> desired NED acceleration
//   small-angle acceleration -> desired roll/pitch
//   Euler attitude PID (+ body-rate damping) -> body moments
//
// The L1 adaptive layer remains downstream and can still augment the resulting
// thrust/moment command.
class SimplePIDController
{
public:
struct Input {
hrt_abstime timestamp_us{0};

float position_ned[3]{0.f, 0.f, 0.f};
float velocity_ned[3]{0.f, 0.f, 0.f};
float quat_body_to_ned[4]{1.f, 0.f, 0.f, 0.f};
float angular_velocity_body[3]{0.f, 0.f, 0.f};

float target_position_ned[3]{0.f, 0.f, 0.f};
float target_velocity_ned[3]{0.f, 0.f, 0.f};
float target_acceleration_ned[3]{0.f, 0.f, 0.f};
float target_jerk_ned[3]{0.f, 0.f, 0.f};
float target_snap_ned[3]{0.f, 0.f, 0.f};

float target_yaw{0.f};
float target_yaw_rate{0.f};
float target_yaw_accel{0.f};

bool state_valid_for_control{false};
bool armed{false};
bool failsafe{false};
uint8_t nav_state{0};
};

struct Output {
hrt_abstime timestamp_us{0};

// Positive errors mean target - current.
float position_error_ned[3]{0.f, 0.f, 0.f};
float velocity_error_ned[3]{0.f, 0.f, 0.f};
float desired_acceleration_ned[3]{0.f, 0.f, 0.f};

// Euler angles are [roll, pitch, yaw] in radians.
float current_euler_rpy[3]{0.f, 0.f, 0.f};
float desired_euler_rpy[3]{0.f, 0.f, 0.f};
float attitude_error_rpy[3]{0.f, 0.f, 0.f};
float attitude_integral_rpy[3]{0.f, 0.f, 0.f};
float rate_error_body[3]{0.f, 0.f, 0.f};

float thrust_newton{0.f};
float moment_newton_meter[3]{0.f, 0.f, 0.f};

bool valid{false};
};

SimplePIDController() = default;
~SimplePIDController() = default;

bool update(const Input &input, Output &output);
void reset();

const Input &last_input() const { return _last_input; }
const Output &last_output() const { return _last_output; }

private:
Input _last_input{};
Output _last_output{};
float _attitude_integral_rpy[3]{0.f, 0.f, 0.f};
hrt_abstime _last_update_us{0};
bool _was_active{false};
};
