#include "GeometricController.hpp"

#include <math.h>

namespace
{

// Temporary controller constants.
// Later these should become PX4 parameters, matching the original g.GeoCtrl_* style parameters.
static constexpr float VEHICLE_MASS_KG = 1.0f;
static constexpr float GRAVITY_MSS = 9.80665f;

static constexpr float KP_X = 4.0f;
static constexpr float KP_Y = 4.0f;
static constexpr float KP_Z = 6.0f;

static constexpr float KV_X = 3.0f;
static constexpr float KV_Y = 3.0f;
static constexpr float KV_Z = 4.0f;

void quat_to_body_z_axis_ned(const float q[4], float z_axis_ned[3])
{
// q = [w, x, y, z]
// Rotation is body FRD -> earth NED.
const float w = q[0];
const float x = q[1];
const float y = q[2];
const float z = q[3];

// Third column of the rotation matrix.
// This is the body z-axis expressed in NED.
z_axis_ned[0] = 2.0f * (x * z + w * y);
z_axis_ned[1] = 2.0f * (y * z - w * x);
z_axis_ned[2] = 1.0f - 2.0f * (x * x + y * y);
}

float dot3(const float a[3], const float b[3])
{
return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

}

bool GeometricController::update(const Input &input, Output &output)
{
_last_input = input;

output.timestamp_us = input.timestamp_us;

// ------------------------------------------------------------
// 1. Position error and velocity error
// Original L1Quad logic:
// r_error = statePos - targetPos
// v_error = stateVel - targetVel
// ------------------------------------------------------------
for (int i = 0; i < 3; i++) {
output.position_error_ned[i] = input.position_ned[i] - input.target_position_ned[i];
output.velocity_error_ned[i] = input.velocity_ned[i] - input.target_velocity_ned[i];
}

// ------------------------------------------------------------
// 2. Target force in NED
// Original L1Quad logic:
// target_force.x = m * targetAcc.x - Kp_x * ex - Kv_x * evx
// target_force.y = m * targetAcc.y - Kp_y * ey - Kv_y * evy
// target_force.z = m * (targetAcc.z - g) - Kp_z * ez - Kv_z * evz
//
// NED convention:
// z is positive downward.
// Hover-like force is therefore negative in z direction.
// ------------------------------------------------------------
output.target_force_ned[0] =
VEHICLE_MASS_KG * input.target_acceleration_ned[0]
- KP_X * output.position_error_ned[0]
- KV_X * output.velocity_error_ned[0];

output.target_force_ned[1] =
VEHICLE_MASS_KG * input.target_acceleration_ned[1]
- KP_Y * output.position_error_ned[1]
- KV_Y * output.velocity_error_ned[1];

output.target_force_ned[2] =
VEHICLE_MASS_KG * (input.target_acceleration_ned[2] - GRAVITY_MSS)
- KP_Z * output.position_error_ned[2]
- KV_Z * output.velocity_error_ned[2];

// ------------------------------------------------------------
// 3. Current body z-axis in NED
// Original L1Quad uses:
// q.rotation_matrix(R)
// z_axis = R.colz()
// ------------------------------------------------------------
quat_to_body_z_axis_ned(input.quat_body_to_ned, output.body_z_axis_ned);

// ------------------------------------------------------------
// 4. Target thrust
// Original L1Quad logic:
// target_thrust = -target_force · z_axis
// ------------------------------------------------------------
output.thrust_newton = -dot3(output.target_force_ned, output.body_z_axis_ned);

// Moment calculation is not migrated yet.
output.moment_newton_meter[0] = 0.f;
output.moment_newton_meter[1] = 0.f;
output.moment_newton_meter[2] = 0.f;

// This means the geometric calculation has meaningful input.
// It still does NOT publish actuator outputs anywhere.
output.valid = input.state_valid_for_control && !input.failsafe;

_last_output = output;

// Return true means this update function executed.
// Safety is still represented by output.valid.
return true;
}
