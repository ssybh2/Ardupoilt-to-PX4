#include "GeometricController.hpp"

#include <math.h>

namespace
{

static constexpr float VEHICLE_MASS_KG = 1.0f;
static constexpr float GRAVITY_MSS = 9.80665f;

static constexpr float KP_X = 4.0f;
static constexpr float KP_Y = 4.0f;
static constexpr float KP_Z = 6.0f;

static constexpr float KV_X = 3.0f;
static constexpr float KV_Y = 3.0f;
static constexpr float KV_Z = 4.0f;

float dot3(const float a[3], const float b[3])
{
return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

void cross3(const float a[3], const float b[3], float out[3])
{
out[0] = a[1] * b[2] - a[2] * b[1];
out[1] = a[2] * b[0] - a[0] * b[2];
out[2] = a[0] * b[1] - a[1] * b[0];
}

bool normalize3(float v[3])
{
const float n = sqrtf(dot3(v, v));

if (n < 1e-6f) {
return false;
}

v[0] /= n;
v[1] /= n;
v[2] /= n;

return true;
}

void quat_to_rotation_matrix_body_to_ned(const float q[4], float R[3][3])
{
const float w = q[0];
const float x = q[1];
const float y = q[2];
const float z = q[3];

R[0][0] = 1.0f - 2.0f * (y * y + z * z);
R[0][1] = 2.0f * (x * y - w * z);
R[0][2] = 2.0f * (x * z + w * y);

R[1][0] = 2.0f * (x * y + w * z);
R[1][1] = 1.0f - 2.0f * (x * x + z * z);
R[1][2] = 2.0f * (y * z - w * x);

R[2][0] = 2.0f * (x * z - w * y);
R[2][1] = 2.0f * (y * z + w * x);
R[2][2] = 1.0f - 2.0f * (x * x + y * y);
}

void get_matrix_column(const float R[3][3], int col, float out[3])
{
out[0] = R[0][col];
out[1] = R[1][col];
out[2] = R[2][col];
}

void set_matrix_column(float R[3][3], int col, const float v[3])
{
R[0][col] = v[0];
R[1][col] = v[1];
R[2][col] = v[2];
}

void compute_rotation_error(const float R[3][3], const float Rd[3][3], float eR[3])
{
float A[3][3]{};

for (int i = 0; i < 3; i++) {
for (int j = 0; j < 3; j++) {
float RdT_R = 0.f;
float RT_Rd = 0.f;

for (int k = 0; k < 3; k++) {
RdT_R += Rd[k][i] * R[k][j];
RT_Rd += R[k][i] * Rd[k][j];
}

A[i][j] = 0.5f * (RdT_R - RT_Rd);
}
}

// Same vee convention as the original L1Quad code:
// vee(A) = [A32, A13, A21] using 1-based matrix notation.
eR[0] = A[1][2];
eR[1] = A[2][0];
eR[2] = A[0][1];
}

}

bool GeometricController::update(const Input &input, Output &output)
{
_last_input = input;

output.timestamp_us = input.timestamp_us;

for (int i = 0; i < 3; i++) {
output.position_error_ned[i] = input.position_ned[i] - input.target_position_ned[i];
output.velocity_error_ned[i] = input.velocity_ned[i] - input.target_velocity_ned[i];
}

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

float R[3][3]{};
quat_to_rotation_matrix_body_to_ned(input.quat_body_to_ned, R);

get_matrix_column(R, 2, output.body_z_axis_ned);

output.thrust_newton = -dot3(output.target_force_ned, output.body_z_axis_ned);

// Desired body z-axis:
// Original logic:
// z_axis_desired = -target_force
output.desired_body_z_axis_ned[0] = -output.target_force_ned[0];
output.desired_body_z_axis_ned[1] = -output.target_force_ned[1];
output.desired_body_z_axis_ned[2] = -output.target_force_ned[2];

if (!normalize3(output.desired_body_z_axis_ned)) {
output.desired_body_z_axis_ned[0] = 0.f;
output.desired_body_z_axis_ned[1] = 0.f;
output.desired_body_z_axis_ned[2] = 1.f;
}

// Desired heading vector from target yaw:
// x_c_des = [cos(yaw), sin(yaw), 0]
const float x_c_des[3] = {
cosf(input.target_yaw),
sinf(input.target_yaw),
0.f
};

// Original logic:
// y_axis_desired = z_axis_desired cross x_c_des
cross3(output.desired_body_z_axis_ned, x_c_des, output.desired_body_y_axis_ned);

if (!normalize3(output.desired_body_y_axis_ned)) {
output.desired_body_y_axis_ned[0] = 0.f;
output.desired_body_y_axis_ned[1] = 1.f;
output.desired_body_y_axis_ned[2] = 0.f;
}

// Original logic:
// x_axis_desired = y_axis_desired cross z_axis_desired
cross3(output.desired_body_y_axis_ned,
       output.desired_body_z_axis_ned,
       output.desired_body_x_axis_ned);

if (!normalize3(output.desired_body_x_axis_ned)) {
output.desired_body_x_axis_ned[0] = 1.f;
output.desired_body_x_axis_ned[1] = 0.f;
output.desired_body_x_axis_ned[2] = 0.f;
}

float Rd[3][3]{};
set_matrix_column(Rd, 0, output.desired_body_x_axis_ned);
set_matrix_column(Rd, 1, output.desired_body_y_axis_ned);
set_matrix_column(Rd, 2, output.desired_body_z_axis_ned);

compute_rotation_error(R, Rd, output.rotation_error);

// Moment calculation is still not migrated in this stage.
output.moment_newton_meter[0] = 0.f;
output.moment_newton_meter[1] = 0.f;
output.moment_newton_meter[2] = 0.f;

output.valid = input.state_valid_for_control && !input.failsafe;

_last_output = output;

return true;
}
