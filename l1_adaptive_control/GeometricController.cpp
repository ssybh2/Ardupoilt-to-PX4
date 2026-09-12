#include "GeometricController.hpp"

#include <matrix/matrix/math.hpp>

#include <math.h>

using matrix::Dcmf;
using matrix::Matrix3f;
using matrix::Quatf;
using matrix::Vector;
using matrix::Vector2f;
using matrix::Vector3f;

using Vector9f = Vector<float, 9>;

namespace
{

static constexpr float GRAVITY_MAGNITUDE = 9.80665f;

bool is_finite(const Vector3f &v)
{
	return std::isfinite(v(0)) && std::isfinite(v(1)) && std::isfinite(v(2));
}

bool is_normalizable(const Vector3f &v)
{
	const float norm = v.norm();
	return norm >= 1e-6f && std::isfinite(norm);
}

Matrix3f hatOperator(const Vector3f &input)
{
	Matrix3f output{};
	output(0, 1) = -input(2);
	output(0, 2) = input(1);
	output(1, 0) = input(2);
	output(1, 2) = -input(0);
	output(2, 0) = -input(1);
	output(2, 1) = input(0);
	return output;
}

Vector3f veeOperator(const Matrix3f &input)
{
	return Vector3f{input(2, 1), input(0, 2), input(1, 0)};
}

bool unit_vec(const Vector3f &q, const Vector3f &q_dot, const Vector3f &q_ddot, Vector9f &output)
{
	const float nq = q.norm();

	if (nq < 1e-6f || !std::isfinite(nq)) {
		return false;
	}

	const Vector3f u = q / nq;
	const Vector3f u_dot = q_dot / nq - q * q.dot(q_dot) / powf(nq, 3.f);
	const Vector3f u_ddot = q_ddot / nq
				- q_dot / powf(nq, 3.f) * 2.f * q.dot(q_dot)
				- q / powf(nq, 3.f) * (q_dot.dot(q_dot) + q.dot(q_ddot))
				+ q * 3.f / powf(nq, 5.f) * powf(q.dot(q_dot), 2.f);

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

bool is_finite3(const float v[3])
{
	return std::isfinite(v[0]) && std::isfinite(v[1]) && std::isfinite(v[2]);
}

bool output_is_finite(const GeometricController::Output &output)
{
	return std::isfinite(output.target_thrust)
	       && is_finite3(output.r_error)
	       && is_finite3(output.v_error)
	       && is_finite3(output.M);
}

} // namespace

bool GeometricController::update(const Input &input, Output &output)
{
	_last_input = input;
	output = Output{};
	output.timestamp_us = input.timestamp_us;

	if (!input.state_valid_for_control || !input.armed || input.failsafe) {
		_last_output = output;
		return true;
	}

	const float kg_vehicleMass = _parameters.mass_kg;

	if (!(kg_vehicleMass > 0.f) || !std::isfinite(kg_vehicleMass)) {
		_last_output = output;
		return true;
	}

	const float GeoCtrl_Kpx = _parameters.position_gain[0];
	const float GeoCtrl_Kpy = _parameters.position_gain[1];
	const float GeoCtrl_Kpz = _parameters.position_gain[2];
	const float GeoCtrl_Kvx = _parameters.velocity_gain[0];
	const float GeoCtrl_Kvy = _parameters.velocity_gain[1];
	const float GeoCtrl_Kvz = _parameters.velocity_gain[2];
	const float GeoCtrl_KRx = _parameters.rotation_gain[0];
	const float GeoCtrl_KRy = _parameters.rotation_gain[1];
	const float GeoCtrl_KRz = _parameters.rotation_gain[2];
	const float GeoCtrl_KOx = _parameters.angular_velocity_gain[0];
	const float GeoCtrl_KOy = _parameters.angular_velocity_gain[1];
	const float GeoCtrl_KOz = _parameters.angular_velocity_gain[2];

	Matrix3f J{};
	J(0, 0) = _parameters.inertia_kg_m2[0];
	J(1, 1) = _parameters.inertia_kg_m2[1];
	J(2, 2) = _parameters.inertia_kg_m2[2];

	const Vector3f targetPos{input.target_position_ned[0], input.target_position_ned[1], input.target_position_ned[2]};
	const Vector3f targetVel{input.target_velocity_ned[0], input.target_velocity_ned[1], input.target_velocity_ned[2]};
	const Vector3f targetAcc{input.target_acceleration_ned[0], input.target_acceleration_ned[1],
				 input.target_acceleration_ned[2]};
	const Vector3f targetJerk{input.target_jerk_ned[0], input.target_jerk_ned[1], input.target_jerk_ned[2]};
	const Vector3f targetSnap{input.target_snap_ned[0], input.target_snap_ned[1], input.target_snap_ned[2]};

	const Vector2f targetYaw{input.target_yaw[0], input.target_yaw[1]};
	const Vector2f targetYaw_dot{input.target_yaw_dot[0], input.target_yaw_dot[1]};
	const Vector2f targetYaw_ddot{input.target_yaw_ddot[0], input.target_yaw_ddot[1]};

	const Vector3f statePos{input.position_ned[0], input.position_ned[1], input.position_ned[2]};
	const Vector3f stateVel{input.velocity_ned[0], input.velocity_ned[1], input.velocity_ned[2]};
	const Vector3f Omega{input.angular_velocity_body[0], input.angular_velocity_body[1], input.angular_velocity_body[2]};
	const Vector3f e3{0.f, 0.f, 1.f};

	Vector3f r_error = statePos - targetPos;
	Vector3f v_error = stateVel - targetVel;

	Vector3f target_force;
	target_force(0) = kg_vehicleMass * targetAcc(0) - GeoCtrl_Kpx * r_error(0) - GeoCtrl_Kvx * v_error(0);
	target_force(1) = kg_vehicleMass * targetAcc(1) - GeoCtrl_Kpy * r_error(1) - GeoCtrl_Kvy * v_error(1);
	target_force(2) = kg_vehicleMass * (targetAcc(2) - GRAVITY_MAGNITUDE) - GeoCtrl_Kpz * r_error(2)
			  - GeoCtrl_Kvz * v_error(2);

	const Quatf q{input.quat_body_to_ned};
	const Matrix3f R{Dcmf{q}};
	const Vector3f z_axis{R(0, 2), R(1, 2), R(2, 2)};

	if (!is_finite(z_axis)) {
		_last_output = output;
		return true;
	}

	const float target_thrust = -target_force * z_axis;
	Vector3f z_axis_desired = -target_force;

	if (!is_normalizable(z_axis_desired)) {
		_last_output = output;
		return true;
	}

	z_axis_desired.normalize();

	Vector3f x_c_des{targetYaw(0), targetYaw(1), 0.f};
	Vector3f x_c_des_dot{targetYaw_dot(0), targetYaw_dot(1), 0.f};
	Vector3f x_c_des_ddot{targetYaw_ddot(0), targetYaw_ddot(1), 0.f};

	if (!is_normalizable(x_c_des)) {
		_last_output = output;
		return true;
	}

	x_c_des.normalize();

	Vector3f y_axis_desired = z_axis_desired % x_c_des;

	if (!is_normalizable(y_axis_desired)) {
		_last_output = output;
		return true;
	}

	y_axis_desired.normalize();
	Vector3f x_axis_desired = y_axis_desired % z_axis_desired;

	Matrix3f Rdes{};
	Rdes.setCol(0, x_axis_desired);
	Rdes.setCol(1, y_axis_desired);
	Rdes.setCol(2, z_axis_desired);

	Matrix3f eRM = (Rdes.transpose() * R - R.transpose() * Rdes) * 0.5f;
	Vector3f eR = veeOperator(eRM);

	Vector3f a_error = e3 * GRAVITY_MAGNITUDE - z_axis * target_thrust / kg_vehicleMass - targetAcc;

	Vector3f target_force_dot;
	target_force_dot(0) = -GeoCtrl_Kpx * v_error(0) - GeoCtrl_Kvx * a_error(0)
			      + kg_vehicleMass * targetJerk(0);
	target_force_dot(1) = -GeoCtrl_Kpy * v_error(1) - GeoCtrl_Kvy * a_error(1)
			      + kg_vehicleMass * targetJerk(1);
	target_force_dot(2) = -GeoCtrl_Kpz * v_error(2) - GeoCtrl_Kvz * a_error(2)
			      + kg_vehicleMass * targetJerk(2);

	Vector3f b3_dot = R * hatOperator(Omega) * e3;
	const float target_thrust_dot = -target_force_dot * z_axis - target_force * b3_dot;

	Vector3f j_error = -z_axis * target_thrust_dot / kg_vehicleMass
			       - b3_dot * target_thrust / kg_vehicleMass - targetJerk;

	Vector3f target_force_ddot;
	target_force_ddot(0) = -GeoCtrl_Kpx * a_error(0) - GeoCtrl_Kvx * j_error(0)
				+ kg_vehicleMass * targetSnap(0);
	target_force_ddot(1) = -GeoCtrl_Kpy * a_error(1) - GeoCtrl_Kvy * j_error(1)
				+ kg_vehicleMass * targetSnap(1);
	target_force_ddot(2) = -GeoCtrl_Kpz * a_error(2) - GeoCtrl_Kvz * j_error(2)
				+ kg_vehicleMass * targetSnap(2);

	Vector9f b3cCollection;

	if (!unit_vec(-target_force, -target_force_dot, -target_force_ddot, b3cCollection)) {
		_last_output = output;
		return true;
	}

	Vector3f b3c{b3cCollection(0), b3cCollection(1), b3cCollection(2)};
	Vector3f b3c_dot{b3cCollection(3), b3cCollection(4), b3cCollection(5)};
	Vector3f b3c_ddot{b3cCollection(6), b3cCollection(7), b3cCollection(8)};

	Vector3f A2 = -hatOperator(x_c_des) * b3c;
	Vector3f A2_dot = -hatOperator(x_c_des_dot) * b3c - hatOperator(x_c_des) * b3c_dot;
	Vector3f A2_ddot = -hatOperator(x_c_des_ddot) * b3c - hatOperator(x_c_des_dot) * b3c_dot * 2.f
			      - hatOperator(x_c_des) * b3c_ddot;

	Vector9f b2cCollection;

	if (!unit_vec(A2, A2_dot, A2_ddot, b2cCollection)) {
		_last_output = output;
		return true;
	}

	Vector3f b2c{b2cCollection(0), b2cCollection(1), b2cCollection(2)};
	Vector3f b2c_dot{b2cCollection(3), b2cCollection(4), b2cCollection(5)};
	Vector3f b2c_ddot{b2cCollection(6), b2cCollection(7), b2cCollection(8)};

	Vector3f b1c_dot = hatOperator(b2c_dot) * b3c + hatOperator(b2c) * b3c_dot;
	Vector3f b1c_ddot = hatOperator(b2c_ddot) * b3c + hatOperator(b2c_dot) * b3c_dot * 2.f
				+ hatOperator(b2c) * b3c_ddot;

	Matrix3f Rd_dot{};
	Rd_dot.setCol(0, b1c_dot);
	Rd_dot.setCol(1, b2c_dot);
	Rd_dot.setCol(2, b3c_dot);

	Matrix3f Rd_ddot{};
	Rd_ddot.setCol(0, b1c_ddot);
	Rd_ddot.setCol(1, b2c_ddot);
	Rd_ddot.setCol(2, b3c_ddot);

	Vector3f Omegad = veeOperator(Rdes.transpose() * Rd_dot);
	Vector3f Omegad_dot = veeOperator(Rdes.transpose() * Rd_ddot - hatOperator(Omegad) * hatOperator(Omegad));

	Vector3f ew = Omega - R.transpose() * Rdes * Omegad;

	// Single-motor-out spin hover cannot control absolute yaw. In this mode,
	// control only the thrust-vector direction and let yaw spin freely.
	if (!input.yaw_control_enabled) {
		const Vector3f reduced_rotation_error_ned = z_axis_desired % z_axis;
		eR = R.transpose() * reduced_rotation_error_ned;
		ew = Vector3f{Omega(0), Omega(1), 0.f};
	}

	Vector3f M;
	M(0) = -GeoCtrl_KRx * eR(0) - GeoCtrl_KOx * ew(0);
	M(1) = -GeoCtrl_KRy * eR(1) - GeoCtrl_KOy * ew(1);
	M(2) = -GeoCtrl_KRz * eR(2) - GeoCtrl_KOz * ew(2);

	if (input.yaw_control_enabled) {
		M = M - J * (hatOperator(Omega) * R.transpose() * Rdes * Omegad - R.transpose() * Rdes * Omegad_dot);
	}

	// Keep rigid-body gyroscopic compensation. During spin-hover this term is
	// important because the uncontrolled yaw rate couples into roll/pitch.
	M = M + Omega % (J * Omega);

	if (!input.yaw_control_enabled) {
		M(2) = 0.f;
	}

	copy_vector(r_error, output.r_error);
	copy_vector(v_error, output.v_error);
	copy_vector(M, output.M);
	output.target_thrust = target_thrust;
	output.valid = output_is_finite(output) && output.target_thrust > 0.f;

	if (!output.valid) {
		output.M[0] = 0.f;
		output.M[1] = 0.f;
		output.M[2] = 0.f;
		output.target_thrust = 0.f;
	}

	_last_output = output;
	return true;
}
