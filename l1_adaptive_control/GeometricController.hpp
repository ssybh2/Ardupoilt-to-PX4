#pragma once

#include <drivers/drv_hrt.h>

#include <stdint.h>

class GeometricController
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

float position_error_ned[3]{0.f, 0.f, 0.f};
float velocity_error_ned[3]{0.f, 0.f, 0.f};

float acceleration_error_ned[3]{0.f, 0.f, 0.f};
float jerk_error_ned[3]{0.f, 0.f, 0.f};

float target_force_ned[3]{0.f, 0.f, 0.f};
float target_force_dot_ned[3]{0.f, 0.f, 0.f};
float target_force_ddot_ned[3]{0.f, 0.f, 0.f};

float body_z_axis_ned[3]{0.f, 0.f, 1.f};
float body_z_axis_dot_ned[3]{0.f, 0.f, 0.f};
float target_thrust_dot_newton_s{0.f};

float desired_body_x_axis_ned[3]{1.f, 0.f, 0.f};
float desired_body_y_axis_ned[3]{0.f, 1.f, 0.f};
float desired_body_z_axis_ned[3]{0.f, 0.f, 1.f};
float desired_body_x_axis_dot_ned[3]{0.f, 0.f, 0.f};
float desired_body_y_axis_dot_ned[3]{0.f, 0.f, 0.f};
float desired_body_z_axis_dot_ned[3]{0.f, 0.f, 0.f};
float desired_body_x_axis_ddot_ned[3]{0.f, 0.f, 0.f};
float desired_body_y_axis_ddot_ned[3]{0.f, 0.f, 0.f};
float desired_body_z_axis_ddot_ned[3]{0.f, 0.f, 0.f};

float desired_rotation_matrix[3][3]{{1.f, 0.f, 0.f}, {0.f, 1.f, 0.f}, {0.f, 0.f, 1.f}};
float desired_rotation_matrix_dot[3][3]{{0.f, 0.f, 0.f}, {0.f, 0.f, 0.f}, {0.f, 0.f, 0.f}};
float desired_rotation_matrix_ddot[3][3]{{0.f, 0.f, 0.f}, {0.f, 0.f, 0.f}, {0.f, 0.f, 0.f}};

// Stage: unit_vec(-target_force, -target_force_dot, -target_force_ddot)
float b3c_ned[3]{0.f, 0.f, 1.f};
float b3c_dot_ned[3]{0.f, 0.f, 0.f};
float b3c_ddot_ned[3]{0.f, 0.f, 0.f};

// Stage: A2 and unit_vec(A2, A2_dot, A2_ddot)
float a2_ned[3]{0.f, 1.f, 0.f};
float a2_dot_ned[3]{0.f, 0.f, 0.f};
float a2_ddot_ned[3]{0.f, 0.f, 0.f};

float b2c_ned[3]{0.f, 1.f, 0.f};
float b2c_dot_ned[3]{0.f, 0.f, 0.f};
float b2c_ddot_ned[3]{0.f, 0.f, 0.f};

float rotation_error[3]{0.f, 0.f, 0.f};

float desired_angular_velocity_body[3]{0.f, 0.f, 0.f};
float desired_angular_acceleration_body[3]{0.f, 0.f, 0.f};
float angular_velocity_error[3]{0.f, 0.f, 0.f};

float pd_moment_newton_meter[3]{0.f, 0.f, 0.f};
float feedforward_moment_newton_meter[3]{0.f, 0.f, 0.f};
float j_omega_body[3]{0.f, 0.f, 0.f};
float gyro_moment_newton_meter[3]{0.f, 0.f, 0.f};

float thrust_newton{0.f};
float moment_newton_meter[3]{0.f, 0.f, 0.f};

bool valid{false};
};

GeometricController() = default;
~GeometricController() = default;

bool update(const Input &input, Output &output);

const Input &last_input() const { return _last_input; }
const Output &last_output() const { return _last_output; }

private:
Input _last_input{};
Output _last_output{};
};
