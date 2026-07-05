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

static constexpr float KR_X = 0.08f;
static constexpr float KR_Y = 0.08f;
static constexpr float KR_Z = 0.04f;

static constexpr float KO_X = 0.015f;
static constexpr float KO_Y = 0.015f;
static constexpr float KO_Z = 0.010f;

static constexpr float JXX_KGM2 = 0.005f;
static constexpr float JYY_KGM2 = 0.005f;
static constexpr float JZZ_KGM2 = 0.009f;

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

void set_matrix_column(float R[3][3], int col, const float v[3])
{
R[0][col] = v[0];
R[1][col] = v[1];
R[2][col] = v[2];
}

void get_matrix_column(const float R[3][3], int col, float out[3])
{
out[0] = R[0][col];
out[1] = R[1][col];
out[2] = R[2][col];
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

eR[0] = A[1][2];
eR[1] = A[2][0];
eR[2] = A[0][1];
}

bool unit_vec_with_derivatives(const float q[3],
       const float q_dot[3],
       const float q_ddot[3],
       float u[3],
       float u_dot[3],
       float u_ddot[3])
{
const float nq2 = dot3(q, q);
const float nq = sqrtf(nq2);

if (nq < 1e-6f) {
u[0] = 0.f;
u[1] = 0.f;
u[2] = 1.f;

u_dot[0] = 0.f;
u_dot[1] = 0.f;
u_dot[2] = 0.f;

u_ddot[0] = 0.f;
u_ddot[1] = 0.f;
u_ddot[2] = 0.f;

return false;
}

const float nq3 = nq2 * nq;
const float nq5 = nq3 * nq2;

const float q_qdot = dot3(q, q_dot);
const float qdot_qdot = dot3(q_dot, q_dot);
const float q_qddot = dot3(q, q_ddot);

for (int i = 0; i < 3; i++) {
u[i] = q[i] / nq;

// Original L1Quad:
// u_dot = q_dot/nq - q*(q*q_dot)/nq^3
u_dot[i] = q_dot[i] / nq
   - q[i] * q_qdot / nq3;

// Original L1Quad:
// u_ddot = q_ddot/nq
//          - q_dot/nq^3 * 2*(q*q_dot)
//          - q/nq^3 * (q_dot*q_dot + q*q_ddot)
//          + q*3/nq^5 * (q*q_dot)^2
u_ddot[i] = q_ddot[i] / nq
    - q_dot[i] * 2.f * q_qdot / nq3
    - q[i] * (qdot_qdot + q_qddot) / nq3
    + q[i] * 3.f * q_qdot * q_qdot / nq5;
}

return true;
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

output.acceleration_error_ned[0] =
-output.body_z_axis_ned[0] * output.thrust_newton / VEHICLE_MASS_KG
- input.target_acceleration_ned[0];

output.acceleration_error_ned[1] =
-output.body_z_axis_ned[1] * output.thrust_newton / VEHICLE_MASS_KG
- input.target_acceleration_ned[1];

output.acceleration_error_ned[2] =
GRAVITY_MSS
- output.body_z_axis_ned[2] * output.thrust_newton / VEHICLE_MASS_KG
- input.target_acceleration_ned[2];

output.target_force_dot_ned[0] =
-KP_X * output.velocity_error_ned[0]
-KV_X * output.acceleration_error_ned[0]
+ VEHICLE_MASS_KG * input.target_jerk_ned[0];

output.target_force_dot_ned[1] =
-KP_Y * output.velocity_error_ned[1]
-KV_Y * output.acceleration_error_ned[1]
+ VEHICLE_MASS_KG * input.target_jerk_ned[1];

output.target_force_dot_ned[2] =
-KP_Z * output.velocity_error_ned[2]
-KV_Z * output.acceleration_error_ned[2]
+ VEHICLE_MASS_KG * input.target_jerk_ned[2];

output.jerk_error_ned[0] = -input.target_jerk_ned[0];
output.jerk_error_ned[1] = -input.target_jerk_ned[1];
output.jerk_error_ned[2] = -input.target_jerk_ned[2];

output.target_force_ddot_ned[0] =
-KP_X * output.acceleration_error_ned[0]
-KV_X * output.jerk_error_ned[0]
+ VEHICLE_MASS_KG * input.target_snap_ned[0];

output.target_force_ddot_ned[1] =
-KP_Y * output.acceleration_error_ned[1]
-KV_Y * output.jerk_error_ned[1]
+ VEHICLE_MASS_KG * input.target_snap_ned[1];

output.target_force_ddot_ned[2] =
-KP_Z * output.acceleration_error_ned[2]
-KV_Z * output.jerk_error_ned[2]
+ VEHICLE_MASS_KG * input.target_snap_ned[2];

const float minus_target_force[3] = {
-output.target_force_ned[0],
-output.target_force_ned[1],
-output.target_force_ned[2]
};

const float minus_target_force_dot[3] = {
-output.target_force_dot_ned[0],
-output.target_force_dot_ned[1],
-output.target_force_dot_ned[2]
};

const float minus_target_force_ddot[3] = {
-output.target_force_ddot_ned[0],
-output.target_force_ddot_ned[1],
-output.target_force_ddot_ned[2]
};

unit_vec_with_derivatives(minus_target_force,
  minus_target_force_dot,
  minus_target_force_ddot,
  output.b3c_ned,
  output.b3c_dot_ned,
  output.b3c_ddot_ned);

for (int i = 0; i < 3; i++) {
output.desired_body_z_axis_ned[i] = output.b3c_ned[i];
}

const float yaw = input.target_yaw;
const float yaw_dot = input.target_yaw_rate;
const float yaw_ddot = input.target_yaw_accel;

const float x_c_des[3] = {
cosf(yaw),
sinf(yaw),
0.f
};

const float x_c_des_dot[3] = {
-sinf(yaw) * yaw_dot,
 cosf(yaw) * yaw_dot,
0.f
};

const float x_c_des_ddot[3] = {
-cosf(yaw) * yaw_dot * yaw_dot - sinf(yaw) * yaw_ddot,
-sinf(yaw) * yaw_dot * yaw_dot + cosf(yaw) * yaw_ddot,
0.f
};

// Original L1Quad:
// A2 = -hatOperator(x_c_des) * b3c
// Since hat(a)*b = a cross b:
// A2 = -(x_c_des cross b3c) = b3c cross x_c_des
cross3(output.b3c_ned, x_c_des, output.a2_ned);

// Original L1Quad:
// A2_dot = -hat(x_c_dot)*b3c - hat(x_c)*b3c_dot
//        = b3c cross x_c_dot + b3c_dot cross x_c
float a2_dot_part1[3]{};
float a2_dot_part2[3]{};
cross3(output.b3c_ned, x_c_des_dot, a2_dot_part1);
cross3(output.b3c_dot_ned, x_c_des, a2_dot_part2);

for (int i = 0; i < 3; i++) {
output.a2_dot_ned[i] = a2_dot_part1[i] + a2_dot_part2[i];
}

// Original L1Quad:
// A2_ddot = -hat(x_c_ddot)*b3c
//           -2*hat(x_c_dot)*b3c_dot
//           -hat(x_c)*b3c_ddot
//         = b3c cross x_c_ddot
//           +2*b3c_dot cross x_c_dot
//           +b3c_ddot cross x_c
float a2_ddot_part1[3]{};
float a2_ddot_part2[3]{};
float a2_ddot_part3[3]{};
cross3(output.b3c_ned, x_c_des_ddot, a2_ddot_part1);
cross3(output.b3c_dot_ned, x_c_des_dot, a2_ddot_part2);
cross3(output.b3c_ddot_ned, x_c_des, a2_ddot_part3);

for (int i = 0; i < 3; i++) {
output.a2_ddot_ned[i] =
a2_ddot_part1[i]
+ 2.f * a2_ddot_part2[i]
+ a2_ddot_part3[i];
}

// Original L1Quad:
// b2cCollection = unit_vec(A2, A2_dot, A2_ddot)
unit_vec_with_derivatives(output.a2_ned,
  output.a2_dot_ned,
  output.a2_ddot_ned,
  output.b2c_ned,
  output.b2c_dot_ned,
  output.b2c_ddot_ned);

for (int i = 0; i < 3; i++) {
output.desired_body_y_axis_ned[i] = output.b2c_ned[i];
}

// Original L1Quad:
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

output.desired_angular_velocity_body[0] = 0.f;
output.desired_angular_velocity_body[1] = 0.f;
output.desired_angular_velocity_body[2] = 0.f;

for (int i = 0; i < 3; i++) {
output.angular_velocity_error[i] =
input.angular_velocity_body[i] - output.desired_angular_velocity_body[i];
}

output.pd_moment_newton_meter[0] =
-KR_X * output.rotation_error[0]
-KO_X * output.angular_velocity_error[0];

output.pd_moment_newton_meter[1] =
-KR_Y * output.rotation_error[1]
-KO_Y * output.angular_velocity_error[1];

output.pd_moment_newton_meter[2] =
-KR_Z * output.rotation_error[2]
-KO_Z * output.angular_velocity_error[2];

output.j_omega_body[0] = JXX_KGM2 * input.angular_velocity_body[0];
output.j_omega_body[1] = JYY_KGM2 * input.angular_velocity_body[1];
output.j_omega_body[2] = JZZ_KGM2 * input.angular_velocity_body[2];

cross3(input.angular_velocity_body,
       output.j_omega_body,
       output.gyro_moment_newton_meter);

output.moment_newton_meter[0] =
output.pd_moment_newton_meter[0] + output.gyro_moment_newton_meter[0];

output.moment_newton_meter[1] =
output.pd_moment_newton_meter[1] + output.gyro_moment_newton_meter[1];

output.moment_newton_meter[2] =
output.pd_moment_newton_meter[2] + output.gyro_moment_newton_meter[2];

output.valid = input.state_valid_for_control && !input.failsafe;

_last_output = output;

return true;
}
