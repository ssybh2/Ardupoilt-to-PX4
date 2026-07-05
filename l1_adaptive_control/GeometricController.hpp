#pragma once

#include <drivers/drv_hrt.h>

#include <stdint.h>

class GeometricController
{
public:
struct Input {
hrt_abstime timestamp_us{0};

// Current vehicle state
float position_ned[3]{0.f, 0.f, 0.f};              // x, y, z in NED [m]
float velocity_ned[3]{0.f, 0.f, 0.f};              // vx, vy, vz in NED [m/s]
float quat_body_to_ned[4]{1.f, 0.f, 0.f, 0.f};     // q(w, x, y, z), body FRD -> earth NED
float angular_velocity_body[3]{0.f, 0.f, 0.f};     // p, q, r in body FRD [rad/s]

// Target trajectory
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

// Physical control command placeholder.
// Later:
// thrust_newton = F
// moment_newton_meter = [Mx, My, Mz]
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
