#include "GeometricController.hpp"

#include <math.h>

namespace
{

static constexpr float VEHICLE_MASS_KG = 3.0f;
static constexpr float GRAVITY_MSS = 9.80665f;

static constexpr float KP_X = 18.0f;
static constexpr float KP_Y = 18.0f;
static constexpr float KP_Z = 27.6f;

static constexpr float KV_X = 4.0f;
static constexpr float KV_Y = 4.0f;
static constexpr float KV_Z = 6.0f;

static constexpr float KR_X = 5.4f;
static constexpr float KR_Y = 5.4f;
static constexpr float KR_Z = 0.092f;

static constexpr float KO_X = 0.6f;
static constexpr float KO_Y = 0.6f;
static constexpr float KO_Z = 0.023f;

static constexpr float JXX_KGM2 = 0.023f;
static constexpr float JYY_KGM2 = 0.023f;
static constexpr float JZZ_KGM2 = 0.0459f;

float dot3(const float a[3], const float b[3])
{
return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

void copy3(const float in[3], float out[3])
{
out[0] = in[0];
out[1] = in[1];
out[2] = in[2];
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

if (n < 1e-6f || !isfinite(n)) {
return false;
}

v[0] /= n;
v[1] /= n;
v[2] /= n;

return true;
}

bool is_finite3(const float v[3])
{
return isfinite(v[0]) && isfinite(v[1]) && isfinite(v[2]);
}

bool is_finite_matrix3(const float M[3][3])
{
for (int row = 0; row < 3; row++) {
for (int col = 0; col < 3; col++) {
if (!isfinite(M[row][col])) {
return false;
}
}
}

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

void mat_transpose_mat(const float A[3][3], const float B[3][3], float out[3][3])
{
for (int row = 0; row < 3; row++) {
for (int col = 0; col < 3; col++) {
out[row][col] = 0.f;

for (int k = 0; k < 3; k++) {
out[row][col] += A[k][row] * B[k][col];
}
}
}
}

void mat_vec_mul(const float A[3][3], const float v[3], float out[3])
{
for (int row = 0; row < 3; row++) {
out[row] = A[row][0] * v[0] + A[row][1] * v[1] + A[row][2] * v[2];
}
}

void mat_sub(const float A[3][3], const float B[3][3], float out[3][3])
{
for (int row = 0; row < 3; row++) {
for (int col = 0; col < 3; col++) {
out[row][col] = A[row][col] - B[row][col];
}
}
}

void hat3(const float v[3], float out[3][3])
{
out[0][0] = 0.f;
out[0][1] = -v[2];
out[0][2] = v[1];
out[1][0] = v[2];
out[1][1] = 0.f;
out[1][2] = -v[0];
out[2][0] = -v[1];
out[2][1] = v[0];
out[2][2] = 0.f;
}

void mat_mul(const float A[3][3], const float B[3][3], float out[3][3])
{
for (int row = 0; row < 3; row++) {
for (int col = 0; col < 3; col++) {
out[row][col] = 0.f;

for (int k = 0; k < 3; k++) {
out[row][col] += A[row][k] * B[k][col];
}
}
}
}

void vee3(const float M[3][3], float out[3])
{
out[0] = M[2][1];
out[1] = M[0][2];
out[2] = M[1][0];
}

void inertia_mul(const float v[3], float out[3])
{
out[0] = JXX_KGM2 * v[0];
out[1] = JYY_KGM2 * v[1];
out[2] = JZZ_KGM2 * v[2];
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

eR[0] = A[2][1];
eR[1] = A[0][2];
eR[2] = A[1][0];
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

if (nq < 1e-6f || !isfinite(nq)) {
return false;
}

const float nq3 = nq2 * nq;
const float nq5 = nq3 * nq2;

const float q_qdot = dot3(q, q_dot);
const float qdot_qdot = dot3(q_dot, q_dot);
const float q_qddot = dot3(q, q_ddot);

for (int i = 0; i < 3; i++) {
u[i] = q[i] / nq;
u_dot[i] = q_dot[i] / nq - q[i] * q_qdot / nq3;
u_ddot[i] = q_ddot[i] / nq
    - q_dot[i] * 2.f * q_qdot / nq3
    - q[i] * (qdot_qdot + q_qddot) / nq3
    + q[i] * 3.f * q_qdot * q_qdot / nq5;
}

return is_finite3(u) && is_finite3(u_dot) && is_finite3(u_ddot);
}

bool output_is_finite(const GeometricController::Output &output)
{
return isfinite(output.thrust_newton)
       && isfinite(output.target_thrust_dot_newton_s)
       && is_finite3(output.position_error_ned)
       && is_finite3(output.velocity_error_ned)
       && is_finite3(output.acceleration_error_ned)
       && is_finite3(output.jerk_error_ned)
       && is_finite3(output.target_force_ned)
       && is_finite3(output.target_force_dot_ned)
       && is_finite3(output.target_force_ddot_ned)
       && is_finite3(output.body_z_axis_ned)
       && is_finite3(output.body_z_axis_dot_ned)
       && is_finite3(output.desired_body_x_axis_ned)
       && is_finite3(output.desired_body_y_axis_ned)
       && is_finite3(output.desired_body_z_axis_ned)
       && is_finite3(output.desired_body_x_axis_dot_ned)
       && is_finite3(output.desired_body_y_axis_dot_ned)
       && is_finite3(output.desired_body_z_axis_dot_ned)
       && is_finite3(output.desired_body_x_axis_ddot_ned)
       && is_finite3(output.desired_body_y_axis_ddot_ned)
       && is_finite3(output.desired_body_z_axis_ddot_ned)
       && is_finite_matrix3(output.desired_rotation_matrix)
       && is_finite_matrix3(output.desired_rotation_matrix_dot)
       && is_finite_matrix3(output.desired_rotation_matrix_ddot)
       && is_finite3(output.b3c_ned)
       && is_finite3(output.b3c_dot_ned)
       && is_finite3(output.b3c_ddot_ned)
       && is_finite3(output.b2c_ned)
       && is_finite3(output.b2c_dot_ned)
       && is_finite3(output.b2c_ddot_ned)
       && is_finite3(output.rotation_error)
       && is_finite3(output.desired_angular_velocity_body)
       && is_finite3(output.desired_angular_acceleration_body)
       && is_finite3(output.angular_velocity_error)
       && is_finite3(output.pd_moment_newton_meter)
       && is_finite3(output.feedforward_moment_newton_meter)
       && is_finite3(output.gyro_moment_newton_meter)
       && is_finite3(output.moment_newton_meter);
}

}

bool GeometricController::update(const Input &input, Output &output)
{
_last_input = input;
output = Output{};
output.timestamp_us = input.timestamp_us;

if (!input.state_valid_for_control || !input.armed || input.failsafe) {
output.valid = false;
_last_output = output;
return true;
}

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

if (!is_finite_matrix3(R) || !is_finite3(output.body_z_axis_ned)) {
output.valid = false;
_last_output = output;
return true;
}

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

const float omega_cross_e3[3] = {
input.angular_velocity_body[1],
-input.angular_velocity_body[0],
0.f
};
mat_vec_mul(R, omega_cross_e3, output.body_z_axis_dot_ned);

output.target_thrust_dot_newton_s =
-dot3(output.target_force_dot_ned, output.body_z_axis_ned)
-dot3(output.target_force_ned, output.body_z_axis_dot_ned);

output.jerk_error_ned[0] =
-output.body_z_axis_ned[0] * output.target_thrust_dot_newton_s / VEHICLE_MASS_KG
-output.body_z_axis_dot_ned[0] * output.thrust_newton / VEHICLE_MASS_KG
-input.target_jerk_ned[0];

output.jerk_error_ned[1] =
-output.body_z_axis_ned[1] * output.target_thrust_dot_newton_s / VEHICLE_MASS_KG
-output.body_z_axis_dot_ned[1] * output.thrust_newton / VEHICLE_MASS_KG
-input.target_jerk_ned[1];

output.jerk_error_ned[2] =
-output.body_z_axis_ned[2] * output.target_thrust_dot_newton_s / VEHICLE_MASS_KG
-output.body_z_axis_dot_ned[2] * output.thrust_newton / VEHICLE_MASS_KG
-input.target_jerk_ned[2];

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

if (!unit_vec_with_derivatives(minus_target_force,
			       minus_target_force_dot,
			       minus_target_force_ddot,
			       output.b3c_ned,
			       output.b3c_dot_ned,
			       output.b3c_ddot_ned)) {
output.valid = false;
_last_output = output;
return true;
}

copy3(output.b3c_ned, output.desired_body_z_axis_ned);
copy3(output.b3c_dot_ned, output.desired_body_z_axis_dot_ned);
copy3(output.b3c_ddot_ned, output.desired_body_z_axis_ddot_ned);

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

cross3(output.b3c_ned, x_c_des, output.a2_ned);

float a2_dot_part1[3]{};
float a2_dot_part2[3]{};
cross3(output.b3c_ned, x_c_des_dot, a2_dot_part1);
cross3(output.b3c_dot_ned, x_c_des, a2_dot_part2);

for (int i = 0; i < 3; i++) {
output.a2_dot_ned[i] = a2_dot_part1[i] + a2_dot_part2[i];
}

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

if (!unit_vec_with_derivatives(output.a2_ned,
			       output.a2_dot_ned,
			       output.a2_ddot_ned,
			       output.b2c_ned,
			       output.b2c_dot_ned,
			       output.b2c_ddot_ned)) {
output.valid = false;
_last_output = output;
return true;
}

copy3(output.b2c_ned, output.desired_body_y_axis_ned);
copy3(output.b2c_dot_ned, output.desired_body_y_axis_dot_ned);
copy3(output.b2c_ddot_ned, output.desired_body_y_axis_ddot_ned);

cross3(output.desired_body_y_axis_ned,
       output.desired_body_z_axis_ned,
       output.desired_body_x_axis_ned);

float b1c_dot_part1[3]{};
float b1c_dot_part2[3]{};
cross3(output.desired_body_y_axis_dot_ned,
       output.desired_body_z_axis_ned,
       b1c_dot_part1);
cross3(output.desired_body_y_axis_ned,
       output.desired_body_z_axis_dot_ned,
       b1c_dot_part2);

for (int i = 0; i < 3; i++) {
output.desired_body_x_axis_dot_ned[i] = b1c_dot_part1[i] + b1c_dot_part2[i];
}

float b1c_ddot_part1[3]{};
float b1c_ddot_part2[3]{};
float b1c_ddot_part3[3]{};
cross3(output.desired_body_y_axis_ddot_ned,
       output.desired_body_z_axis_ned,
       b1c_ddot_part1);
cross3(output.desired_body_y_axis_dot_ned,
       output.desired_body_z_axis_dot_ned,
       b1c_ddot_part2);
cross3(output.desired_body_y_axis_ned,
       output.desired_body_z_axis_ddot_ned,
       b1c_ddot_part3);

for (int i = 0; i < 3; i++) {
output.desired_body_x_axis_ddot_ned[i] =
b1c_ddot_part1[i]
+ 2.f * b1c_ddot_part2[i]
+ b1c_ddot_part3[i];
}

if (!normalize3(output.desired_body_x_axis_ned)) {
output.valid = false;
_last_output = output;
return true;
}

set_matrix_column(output.desired_rotation_matrix, 0, output.desired_body_x_axis_ned);
set_matrix_column(output.desired_rotation_matrix, 1, output.desired_body_y_axis_ned);
set_matrix_column(output.desired_rotation_matrix, 2, output.desired_body_z_axis_ned);

set_matrix_column(output.desired_rotation_matrix_dot, 0, output.desired_body_x_axis_dot_ned);
set_matrix_column(output.desired_rotation_matrix_dot, 1, output.desired_body_y_axis_dot_ned);
set_matrix_column(output.desired_rotation_matrix_dot, 2, output.desired_body_z_axis_dot_ned);

set_matrix_column(output.desired_rotation_matrix_ddot, 0, output.desired_body_x_axis_ddot_ned);
set_matrix_column(output.desired_rotation_matrix_ddot, 1, output.desired_body_y_axis_ddot_ned);
set_matrix_column(output.desired_rotation_matrix_ddot, 2, output.desired_body_z_axis_ddot_ned);

compute_rotation_error(R, output.desired_rotation_matrix, output.rotation_error);

float RdT_Rd_dot[3][3]{};
mat_transpose_mat(output.desired_rotation_matrix, output.desired_rotation_matrix_dot, RdT_Rd_dot);
vee3(RdT_Rd_dot, output.desired_angular_velocity_body);

float RdT_Rd_ddot[3][3]{};
float omega_d_hat[3][3]{};
float omega_d_hat_sq[3][3]{};
float omega_d_dot_matrix[3][3]{};
mat_transpose_mat(output.desired_rotation_matrix, output.desired_rotation_matrix_ddot, RdT_Rd_ddot);
hat3(output.desired_angular_velocity_body, omega_d_hat);
mat_mul(omega_d_hat, omega_d_hat, omega_d_hat_sq);
mat_sub(RdT_Rd_ddot, omega_d_hat_sq, omega_d_dot_matrix);
vee3(omega_d_dot_matrix, output.desired_angular_acceleration_body);

float RT_Rd[3][3]{};
mat_transpose_mat(R, output.desired_rotation_matrix, RT_Rd);

float RT_Rd_omega_d[3]{};
float RT_Rd_omega_d_dot[3]{};
mat_vec_mul(RT_Rd, output.desired_angular_velocity_body, RT_Rd_omega_d);
mat_vec_mul(RT_Rd, output.desired_angular_acceleration_body, RT_Rd_omega_d_dot);

for (int i = 0; i < 3; i++) {
output.angular_velocity_error[i] = input.angular_velocity_body[i] - RT_Rd_omega_d[i];
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

float omega_hat[3][3]{};
float omega_hat_rt_rd_omega_d[3]{};
hat3(input.angular_velocity_body, omega_hat);
mat_vec_mul(omega_hat, RT_Rd_omega_d, omega_hat_rt_rd_omega_d);

const float feedforward_argument[3] = {
omega_hat_rt_rd_omega_d[0] - RT_Rd_omega_d_dot[0],
omega_hat_rt_rd_omega_d[1] - RT_Rd_omega_d_dot[1],
omega_hat_rt_rd_omega_d[2] - RT_Rd_omega_d_dot[2]
};

float j_feedforward_argument[3]{};
inertia_mul(feedforward_argument, j_feedforward_argument);

for (int i = 0; i < 3; i++) {
output.feedforward_moment_newton_meter[i] = -j_feedforward_argument[i];
}

inertia_mul(input.angular_velocity_body, output.j_omega_body);
cross3(input.angular_velocity_body,
       output.j_omega_body,
       output.gyro_moment_newton_meter);

for (int i = 0; i < 3; i++) {
output.moment_newton_meter[i] =
output.pd_moment_newton_meter[i]
+ output.feedforward_moment_newton_meter[i]
+ output.gyro_moment_newton_meter[i];
}

output.valid = output_is_finite(output) && output.thrust_newton > 0.f;

if (!output.valid) {
output.moment_newton_meter[0] = 0.f;
output.moment_newton_meter[1] = 0.f;
output.moment_newton_meter[2] = 0.f;
output.thrust_newton = 0.f;
}

_last_output = output;

return true;
}
