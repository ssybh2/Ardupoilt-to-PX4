#include "GeometricController.hpp"

#include <matrix/matrix/math.hpp>

#include <math.h>

using matrix::Dcmf;
using matrix::Matrix3f;
using matrix::Quatf;
using matrix::Vector;
using matrix::Vector3f;

using Vector9f = Vector<float, 9>;

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

bool is_finite(const Vector3f &v)
{
	return isfinite(v(0)) && isfinite(v(1)) && isfinite(v(2));
}

bool is_normalizable(const Vector3f &v)
{
	const float norm = v.norm();
	return norm >= 1e-6f && isfinite(norm);
}

bool unit_vec_with_derivatives(const Vector3f &q,
			       const Vector3f &q_dot,
			       const Vector3f &q_ddot,
			       Vector9f &output)
{
	const float nq = q.norm();

	if (nq < 1e-6f || !isfinite(nq)) {
		return false;
	}

	const float nq3 = nq * nq * nq;
	const float nq5 = nq3 * nq * nq;
	const float q_qdot = q.dot(q_dot);

	const Vector3f u = q / nq;
	const Vector3f u_dot = q_dot / nq - q * q_qdot / nq3;
	const Vector3f u_ddot = q_ddot / nq
				- q_dot * (2.f * q_qdot / nq3)
				- q * ((q_dot.dot(q_dot) + q.dot(q_ddot)) / nq3)
				+ q * (3.f * q_qdot * q_qdot / nq5);

	for (int i = 0; i < 3; i++) {
		output(i) = u(i);
		output(i + 3) = u_dot(i);
		output(i + 6) = u_ddot(i);
	}

	return is_finite(u) && is_finite(u_dot) && is_finite(u_ddot);
}

void copy_vector(const Vector3f &input, float output[3])
{
	for (int i = 0; i < 3; i++) {
		output[i] = input(i);
	}
}

void copy_matrix(const Matrix3f &input, float output[3][3])
{
	for (int row = 0; row < 3; row++) {
		for (int col = 0; col < 3; col++) {
			output[row][col] = input(row, col);
		}
	}
}

bool is_finite3(const float v[3])
{
	return isfinite(v[0]) && isfinite(v[1]) && isfinite(v[2]);
}

bool is_finite_matrix3(const float matrix[3][3])
{
	for (int row = 0; row < 3; row++) {
		for (int col = 0; col < 3; col++) {
			if (!isfinite(matrix[row][col])) {
				return false;
			}
		}

	return true;
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
	       && is_finite3(output.a2_ned)
	       && is_finite3(output.a2_dot_ned)
	       && is_finite3(output.a2_ddot_ned)
	       && is_finite3(output.b2c_ned)
	       && is_finite3(output.b2c_dot_ned)
	       && is_finite3(output.b2c_ddot_ned)
	       && is_finite3(output.rotation_error)
	       && is_finite3(output.desired_angular_velocity_body)
	       && is_finite3(output.desired_angular_acceleration_body)
	       && is_finite3(output.angular_velocity_error)
	       && is_finite3(output.pd_moment_newton_meter)
	       && is_finite3(output.feedforward_moment_newton_meter)
	       && is_finite3(output.j_omega_body)
	       && is_finite3(output.gyro_moment_newton_meter)
	       && is_finite3(output.moment_newton_meter);
}

} // namespace

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

	const Vector3f state_position{input.position_ned[0], input.position_ned[1], input.position_ned[2]};
	const Vector3f state_velocity{input.velocity_ned[0], input.velocity_ned[1], input.velocity_ned[2]};
	const Vector3f omega{input.angular_velocity_body[0], input.angular_velocity_body[1], input.angular_velocity_body[2]};

	const Vector3f target_position{input.target_position_ned[0], input.target_position_ned[1], input.target_position_ned[2]};
	const Vector3f target_velocity{input.target_velocity_ned[0], input.target_velocity_ned[1], input.target_velocity_ned[2]};
	const Vector3f target_acceleration{input.target_acceleration_ned[0], input.target_acceleration_ned[1],
					  input.target_acceleration_ned[2]};
	const Vector3f target_jerk{input.target_jerk_ned[0], input.target_jerk_ned[1], input.target_jerk_ned[2]};
	const Vector3f target_snap{input.target_snap_ned[0], input.target_snap_ned[1], input.target_snap_ned[2]};
	const Vector3f e3{0.f, 0.f, 1.f};

	const Vector3f position_error = state_position - target_position;
	const Vector3f velocity_error = state_velocity - target_velocity;

	Vector3f target_force;
	target_force(0) = VEHICLE_MASS_KG * target_acceleration(0) - KP_X * position_error(0) - KV_X * velocity_error(0);
	target_force(1) = VEHICLE_MASS_KG * target_acceleration(1) - KP_Y * position_error(1) - KV_Y * velocity_error(1);
	target_force(2) = VEHICLE_MASS_KG * (target_acceleration(2) - GRAVITY_MSS)
			  - KP_Z * position_error(2) - KV_Z * velocity_error(2);

	const Quatf attitude{input.quat_body_to_ned};
	const Dcmf rotation{attitude};
	const Vector3f body_z_axis{rotation(0, 2), rotation(1, 2), rotation(2, 2)};

	if (!is_finite(body_z_axis)) {
		output.valid = false;
		_last_output = output;
		return true;
	}

	const float thrust_newton = -target_force.dot(body_z_axis);

	const Vector3f acceleration_error =
		e3 * GRAVITY_MSS - body_z_axis * (thrust_newton / VEHICLE_MASS_KG) - target_acceleration;

	Vector3f target_force_dot;
	target_force_dot(0) = -KP_X * velocity_error(0) - KV_X * acceleration_error(0)
			      + VEHICLE_MASS_KG * target_jerk(0);
	target_force_dot(1) = -KP_Y * velocity_error(1) - KV_Y * acceleration_error(1)
			      + VEHICLE_MASS_KG * target_jerk(1);
	target_force_dot(2) = -KP_Z * velocity_error(2) - KV_Z * acceleration_error(2)
			      + VEHICLE_MASS_KG * target_jerk(2);

	const Vector3f body_z_axis_dot = rotation * omega.hat() * e3;
	const float thrust_dot_newton_s = -target_force_dot.dot(body_z_axis) - target_force.dot(body_z_axis_dot);

	const Vector3f jerk_error =
		-body_z_axis * (thrust_dot_newton_s / VEHICLE_MASS_KG)
		- body_z_axis_dot * (thrust_newton / VEHICLE_MASS_KG)
		- target_jerk;

	Vector3f target_force_ddot;
	target_force_ddot(0) = -KP_X * acceleration_error(0) - KV_X * jerk_error(0)
			       + VEHICLE_MASS_KG * target_snap(0);
	target_force_ddot(1) = -KP_Y * acceleration_error(1) - KV_Y * jerk_error(1)
			       + VEHICLE_MASS_KG * target_snap(1);
	target_force_ddot(2) = -KP_Z * acceleration_error(2) - KV_Z * jerk_error(2)
			       + VEHICLE_MASS_KG * target_snap(2);

	Vector9f b3c_collection;

	if (!unit_vec_with_derivatives(-target_force, -target_force_dot, -target_force_ddot, b3c_collection)) {
		output.valid = false;
		_last_output = output;
		return true;
	}

	const Vector3f b3c{b3c_collection(0), b3c_collection(1), b3c_collection(2)};
	const Vector3f b3c_dot{b3c_collection(3), b3c_collection(4), b3c_collection(5)};
	const Vector3f b3c_ddot{b3c_collection(6), b3c_collection(7), b3c_collection(8)};

	const float yaw = input.target_yaw;
	const float yaw_rate = input.target_yaw_rate;
	const float yaw_accel = input.target_yaw_accel;
	const Vector3f x_c_des{cosf(yaw), sinf(yaw), 0.f};
	const Vector3f x_c_des_dot{-sinf(yaw) * yaw_rate, cosf(yaw) * yaw_rate, 0.f};
	const Vector3f x_c_des_ddot{
		-cosf(yaw) * yaw_rate * yaw_rate - sinf(yaw) * yaw_accel,
		-sinf(yaw) * yaw_rate * yaw_rate + cosf(yaw) * yaw_accel,
		0.f
	};

	const Vector3f a2 = b3c.cross(x_c_des);
	const Vector3f a2_dot = b3c.cross(x_c_des_dot) + b3c_dot.cross(x_c_des);
	const Vector3f a2_ddot = b3c.cross(x_c_des_ddot)
				 + b3c_dot.cross(x_c_des_dot) * 2.f
				 + b3c_ddot.cross(x_c_des);

	Vector9f b2c_collection;

	if (!unit_vec_with_derivatives(a2, a2_dot, a2_ddot, b2c_collection)) {
		output.valid = false;
		_last_output = output;
		return true;
	}

	const Vector3f b2c{b2c_collection(0), b2c_collection(1), b2c_collection(2)};
	const Vector3f b2c_dot{b2c_collection(3), b2c_collection(4), b2c_collection(5)};
	const Vector3f b2c_ddot{b2c_collection(6), b2c_collection(7), b2c_collection(8)};

	Vector3f b1c = b2c.cross(b3c);

	if (!is_normalizable(b1c)) {
		output.valid = false;
		_last_output = output;
		return true;
	}

	b1c.normalize();

	const Vector3f b1c_dot = b2c_dot.cross(b3c) + b2c.cross(b3c_dot);
	const Vector3f b1c_ddot = b2c_ddot.cross(b3c)
				  + b2c_dot.cross(b3c_dot) * 2.f
				  + b2c.cross(b3c_ddot);

	Matrix3f desired_rotation{};
	desired_rotation.setCol(0, b1c);
	desired_rotation.setCol(1, b2c);
	desired_rotation.setCol(2, b3c);

	Matrix3f desired_rotation_dot{};
	desired_rotation_dot.setCol(0, b1c_dot);
	desired_rotation_dot.setCol(1, b2c_dot);
	desired_rotation_dot.setCol(2, b3c_dot);

	Matrix3f desired_rotation_ddot{};
	desired_rotation_ddot.setCol(0, b1c_ddot);
	desired_rotation_ddot.setCol(1, b2c_ddot);
	desired_rotation_ddot.setCol(2, b3c_ddot);

	const Matrix3f rotation_error_matrix =
		(desired_rotation.transpose() * rotation - rotation.transpose() * desired_rotation) * 0.5f;
	const Vector3f rotation_error = Dcmf{rotation_error_matrix}.vee();

	const Vector3f desired_angular_velocity = Dcmf{desired_rotation.transpose() * desired_rotation_dot}.vee();
	const Dcmf desired_angular_velocity_hat = desired_angular_velocity.hat();
	const Vector3f desired_angular_acceleration =
		Dcmf{desired_rotation.transpose() * desired_rotation_ddot
		     - desired_angular_velocity_hat * desired_angular_velocity_hat}.vee();

	const Matrix3f rotation_transpose_desired = rotation.transpose() * desired_rotation;
	const Vector3f desired_angular_velocity_in_body = rotation_transpose_desired * desired_angular_velocity;
	const Vector3f desired_angular_acceleration_in_body = rotation_transpose_desired * desired_angular_acceleration;
	const Vector3f angular_velocity_error = omega - desired_angular_velocity_in_body;

	const Vector3f pd_moment{
		-KR_X * rotation_error(0) - KO_X * angular_velocity_error(0),
		-KR_Y * rotation_error(1) - KO_Y * angular_velocity_error(1),
		-KR_Z * rotation_error(2) - KO_Z * angular_velocity_error(2)
	};

	Matrix3f inertia{};
	inertia(0, 0) = JXX_KGM2;
	inertia(1, 1) = JYY_KGM2;
	inertia(2, 2) = JZZ_KGM2;

	const Vector3f feedforward_argument =
		omega.hat() * desired_angular_velocity_in_body - desired_angular_acceleration_in_body;
	const Vector3f feedforward_moment = -(inertia * feedforward_argument);
	const Vector3f j_omega = inertia * omega;
	const Vector3f gyro_moment = omega.cross(j_omega);
	const Vector3f moment = pd_moment + feedforward_moment + gyro_moment;

	copy_vector(position_error, output.position_error_ned);
	copy_vector(velocity_error, output.velocity_error_ned);
	copy_vector(acceleration_error, output.acceleration_error_ned);
	copy_vector(jerk_error, output.jerk_error_ned);
	copy_vector(target_force, output.target_force_ned);
	copy_vector(target_force_dot, output.target_force_dot_ned);
	copy_vector(target_force_ddot, output.target_force_ddot_ned);
	copy_vector(body_z_axis, output.body_z_axis_ned);
	copy_vector(body_z_axis_dot, output.body_z_axis_dot_ned);
	output.target_thrust_dot_newton_s = thrust_dot_newton_s;

	copy_vector(b1c, output.desired_body_x_axis_ned);
	copy_vector(b2c, output.desired_body_y_axis_ned);
	copy_vector(b3c, output.desired_body_z_axis_ned);
	copy_vector(b1c_dot, output.desired_body_x_axis_dot_ned);
	copy_vector(b2c_dot, output.desired_body_y_axis_dot_ned);
	copy_vector(b3c_dot, output.desired_body_z_axis_dot_ned);
	copy_vector(b1c_ddot, output.desired_body_x_axis_ddot_ned);
	copy_vector(b2c_ddot, output.desired_body_y_axis_ddot_ned);
	copy_vector(b3c_ddot, output.desired_body_z_axis_ddot_ned);

	copy_matrix(desired_rotation, output.desired_rotation_matrix);
	copy_matrix(desired_rotation_dot, output.desired_rotation_matrix_dot);
	copy_matrix(desired_rotation_ddot, output.desired_rotation_matrix_ddot);

	copy_vector(b3c, output.b3c_ned);
	copy_vector(b3c_dot, output.b3c_dot_ned);
	copy_vector(b3c_ddot, output.b3c_ddot_ned);
	copy_vector(a2, output.a2_ned);
	copy_vector(a2_dot, output.a2_dot_ned);
	copy_vector(a2_ddot, output.a2_ddot_ned);
	copy_vector(b2c, output.b2c_ned);
	copy_vector(b2c_dot, output.b2c_dot_ned);
	copy_vector(b2c_ddot, output.b2c_ddot_ned);

	copy_vector(rotation_error, output.rotation_error);
	copy_vector(desired_angular_velocity, output.desired_angular_velocity_body);
	copy_vector(desired_angular_acceleration, output.desired_angular_acceleration_body);
	copy_vector(angular_velocity_error, output.angular_velocity_error);
	copy_vector(pd_moment, output.pd_moment_newton_meter);
	copy_vector(feedforward_moment, output.feedforward_moment_newton_meter);
	copy_vector(j_omega, output.j_omega_body);
	copy_vector(gyro_moment, output.gyro_moment_newton_meter);
	copy_vector(moment, output.moment_newton_meter);
	output.thrust_newton = thrust_newton;

	output.valid = output_is_finite(output) && output.thrust_newton > 0.f;

	if (!output.valid) {
		for (int i = 0; i < 3; i++) {
			output.moment_newton_meter[i] = 0.f;
		}

		output.thrust_newton = 0.f;
	}

	_last_output = output;
	return true;
}
