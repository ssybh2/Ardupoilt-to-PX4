#include "MotorMixer.hpp"

#include <math.h>

bool MotorMixer::mix(const ThrustMoment &thrustMomentCmd, MotorCommand &motor_command) const
{
	MotorCommand w{};

	float L = 0.f;
	float D = 0.f;
	float a_F = 0.f;
	float b_F = 0.f;
	float a_M = 0.f;
	float b_M = 0.f;

	if (!_parameters.real_vehicle) {
		// Original !REAL_OR_SITL constants.
		L = 0.25f;
		D = 0.25f;
		a_F = 0.0014597f;
		b_F = 0.043693f;
		a_M = 0.000011667f;
		b_M = 0.0059137f;

	} else {
		// Original REAL_OR_SITL constants.
		L = 0.175f;
		D = 0.131f;
		a_F = 0.0009251f;
		b_F = 0.021145f;
		a_M = 0.00001211f;
		b_M = 0.0009864f;
	}

	// Direct port of ModeAdaptive::motorMixing().
	const float w0 = (-b_F + sqrtf(b_F * b_F + a_F * thrustMomentCmd(0))) / 2.f / a_F;
	const float c_F = 2.f * a_F * w0 + b_F;
	const float c_M = 2.f * a_M * w0 + b_M;
	const float thrust_biased = 2.f * thrustMomentCmd(0) - 4.f * b_F * w0;
	const float M1 = thrustMomentCmd(1);
	const float M2 = thrustMomentCmd(2);
	const float M3 = thrustMomentCmd(3);

	const float c_F4_inv = 1.f / (4.f * c_F);
	const float c_FL_inv = 1.f / (2.f * L * c_F);
	const float c_FD_inv = 1.f / (2.f * D * c_F);
	const float c_M4_inv = 1.f / (4.f * c_M);

	w(0) = c_F4_inv * thrust_biased - c_FL_inv * M1 + c_FD_inv * M2 + c_M4_inv * M3;
	w(1) = c_F4_inv * thrust_biased + c_FL_inv * M1 - c_FD_inv * M2 + c_M4_inv * M3;
	w(2) = c_F4_inv * thrust_biased + c_FL_inv * M1 + c_FD_inv * M2 - c_M4_inv * M3;
	w(3) = c_F4_inv * thrust_biased - c_FL_inv * M1 - c_FD_inv * M2 - c_M4_inv * M3;

	const MotorCommand w2 = iterativeMotorMixing(w, thrustMomentCmd, a_F, b_F, a_M, b_M, L, D);
	motor_command = iterativeMotorMixing(w2, thrustMomentCmd, a_F, b_F, a_M, b_M, L, D);
	return true;
}

MotorMixer::MotorCommand MotorMixer::iterativeMotorMixing(const MotorCommand &w_input,
		const ThrustMoment &thrustMomentCmd, float a_F, float b_F, float a_M, float b_M,
		float L, float D) const
{
	MotorCommand w_new{};

	const float w1_square = w_input(0) * w_input(0);
	const float w2_square = w_input(1) * w_input(1);
	const float w3_square = w_input(2) * w_input(2);
	const float w4_square = w_input(3) * w_input(3);

	const float c_F1 = -a_F * w1_square;
	const float c_F2 = -a_F * w2_square;
	const float c_F3 = -a_F * w3_square;
	const float c_F4 = -a_F * w4_square;

	const float c_M1 = -a_M * w1_square;
	const float c_M2 = -a_M * w2_square;
	const float c_M3 = -a_M * w3_square;
	const float c_M4 = -a_M * w4_square;

	const float d_F1 = 2.f * a_F * w_input(0) + b_F;
	const float d_F2 = 2.f * a_F * w_input(1) + b_F;
	const float d_F3 = 2.f * a_F * w_input(2) + b_F;
	const float d_F4 = 2.f * a_F * w_input(3) + b_F;

	const float d_M1 = 2.f * a_M * w_input(0) + b_M;
	const float d_M2 = 2.f * a_M * w_input(1) + b_M;
	const float d_M3 = 2.f * a_M * w_input(2) + b_M;
	const float d_M4 = 2.f * a_M * w_input(3) + b_M;

	MotorCommand coefficientRow1{};
	MotorCommand coefficientRow2{};
	MotorCommand coefficientRow3{};
	MotorCommand coefficientRow4{};

	coefficientRow1(0) = d_F1;
	coefficientRow1(1) = d_F2;
	coefficientRow1(2) = d_F3;
	coefficientRow1(3) = d_F4;

	coefficientRow2(0) = -d_F1;
	coefficientRow2(1) = d_F2;
	coefficientRow2(2) = d_F3;
	coefficientRow2(3) = -d_F4;

	coefficientRow3(0) = d_F1;
	coefficientRow3(1) = -d_F2;
	coefficientRow3(2) = d_F3;
	coefficientRow3(3) = -d_F4;

	coefficientRow4(0) = d_M1;
	coefficientRow4(1) = d_M2;
	coefficientRow4(2) = -d_M3;
	coefficientRow4(3) = -d_M4;

	const Vector16f coefficientMatrixInv = mat4Inv(coefficientRow1, coefficientRow2,
			coefficientRow3, coefficientRow4);

	MotorCommand coefficientInvRow1{};
	MotorCommand coefficientInvRow2{};
	MotorCommand coefficientInvRow3{};
	MotorCommand coefficientInvRow4{};

	for (int i = 0; i < 4; i++) {
		coefficientInvRow1(i) = coefficientMatrixInv(i);
		coefficientInvRow2(i) = coefficientMatrixInv(i + 4);
		coefficientInvRow3(i) = coefficientMatrixInv(i + 8);
		coefficientInvRow4(i) = coefficientMatrixInv(i + 12);
	}

	MotorCommand shiftedCmd{};
	shiftedCmd(0) = thrustMomentCmd(0) - c_F1 - c_F2 - c_F3 - c_F4;
	shiftedCmd(1) = 2.f * thrustMomentCmd(1) / L + c_F1 - c_F2 - c_F3 + c_F4;
	shiftedCmd(2) = 2.f * thrustMomentCmd(2) / D - c_F1 + c_F2 - c_F3 + c_F4;
	shiftedCmd(3) = thrustMomentCmd(3) - c_M1 - c_M2 + c_M3 + c_M4;

	w_new(0) = coefficientInvRow1.dot(shiftedCmd);
	w_new(1) = coefficientInvRow2.dot(shiftedCmd);
	w_new(2) = coefficientInvRow3.dot(shiftedCmd);
	w_new(3) = coefficientInvRow4.dot(shiftedCmd);

	return w_new;
}

MotorMixer::Vector16f MotorMixer::mat4Inv(const MotorCommand &coefficientRow1,
		const MotorCommand &coefficientRow2, const MotorCommand &coefficientRow3,
		const MotorCommand &coefficientRow4) const
{
	// Direct port of ModeAdaptive::mat4Inv().
	const float A2323 = coefficientRow3(2) * coefficientRow4(3) - coefficientRow3(3) * coefficientRow4(2);
	const float A1323 = coefficientRow3(1) * coefficientRow4(3) - coefficientRow3(3) * coefficientRow4(1);
	const float A1223 = coefficientRow3(1) * coefficientRow4(2) - coefficientRow3(2) * coefficientRow4(1);
	const float A0323 = coefficientRow3(0) * coefficientRow4(3) - coefficientRow3(3) * coefficientRow4(0);
	const float A0223 = coefficientRow3(0) * coefficientRow4(2) - coefficientRow3(2) * coefficientRow4(0);
	const float A0123 = coefficientRow3(0) * coefficientRow4(1) - coefficientRow3(1) * coefficientRow4(0);
	const float A2313 = coefficientRow2(2) * coefficientRow4(3) - coefficientRow2(3) * coefficientRow4(2);
	const float A1313 = coefficientRow2(1) * coefficientRow4(3) - coefficientRow2(3) * coefficientRow4(1);
	const float A1213 = coefficientRow2(1) * coefficientRow4(2) - coefficientRow2(2) * coefficientRow4(1);
	const float A2312 = coefficientRow2(2) * coefficientRow3(3) - coefficientRow2(3) * coefficientRow3(2);
	const float A1312 = coefficientRow2(1) * coefficientRow3(3) - coefficientRow2(3) * coefficientRow3(1);
	const float A1212 = coefficientRow2(1) * coefficientRow3(2) - coefficientRow2(2) * coefficientRow3(1);
	const float A0313 = coefficientRow2(0) * coefficientRow4(3) - coefficientRow2(3) * coefficientRow4(0);
	const float A0213 = coefficientRow2(0) * coefficientRow4(2) - coefficientRow2(2) * coefficientRow4(0);
	const float A0312 = coefficientRow2(0) * coefficientRow3(3) - coefficientRow2(3) * coefficientRow3(0);
	const float A0212 = coefficientRow2(0) * coefficientRow3(2) - coefficientRow2(2) * coefficientRow3(0);
	const float A0113 = coefficientRow2(0) * coefficientRow4(1) - coefficientRow2(1) * coefficientRow4(0);
	const float A0112 = coefficientRow2(0) * coefficientRow3(1) - coefficientRow2(1) * coefficientRow3(0);

	float det = coefficientRow1(0) * (coefficientRow2(1) * A2323 - coefficientRow2(2) * A1323 + coefficientRow2(3) * A1223)
		- coefficientRow1(1) * (coefficientRow2(0) * A2323 - coefficientRow2(2) * A0323 + coefficientRow2(3) * A0223)
		+ coefficientRow1(2) * (coefficientRow2(0) * A1323 - coefficientRow2(1) * A0323 + coefficientRow2(3) * A0123)
		- coefficientRow1(3) * (coefficientRow2(0) * A1223 - coefficientRow2(1) * A0223 + coefficientRow2(2) * A0123);
	det = 1.f / det;

	Vector16f inv{};
	inv(0) = det * (coefficientRow2(1) * A2323 - coefficientRow2(2) * A1323 + coefficientRow2(3) * A1223);
	inv(1) = det * -(coefficientRow1(1) * A2323 - coefficientRow1(2) * A1323 + coefficientRow1(3) * A1223);
	inv(2) = det * (coefficientRow1(1) * A2313 - coefficientRow1(2) * A1313 + coefficientRow1(3) * A1213);
	inv(3) = det * -(coefficientRow1(1) * A2312 - coefficientRow1(2) * A1312 + coefficientRow1(3) * A1212);
	inv(4) = det * -(coefficientRow2(0) * A2323 - coefficientRow2(2) * A0323 + coefficientRow2(3) * A0223);
	inv(5) = det * (coefficientRow1(0) * A2323 - coefficientRow1(2) * A0323 + coefficientRow1(3) * A0223);
	inv(6) = det * -(coefficientRow1(0) * A2313 - coefficientRow1(2) * A0313 + coefficientRow1(3) * A0213);
	inv(7) = det * (coefficientRow1(0) * A2312 - coefficientRow1(2) * A0312 + coefficientRow1(3) * A0212);
	inv(8) = det * (coefficientRow2(0) * A1323 - coefficientRow2(1) * A0323 + coefficientRow2(3) * A0123);
	inv(9) = det * -(coefficientRow1(0) * A1323 - coefficientRow1(1) * A0323 + coefficientRow1(3) * A0123);
	inv(10) = det * (coefficientRow1(0) * A1313 - coefficientRow1(1) * A0313 + coefficientRow1(3) * A0113);
	inv(11) = det * -(coefficientRow1(0) * A1312 - coefficientRow1(1) * A0312 + coefficientRow1(3) * A0112);
	inv(12) = det * -(coefficientRow2(0) * A1223 - coefficientRow2(1) * A0223 + coefficientRow2(2) * A0123);
	inv(13) = det * (coefficientRow1(0) * A1223 - coefficientRow1(1) * A0223 + coefficientRow1(2) * A0123);
	inv(14) = det * -(coefficientRow1(0) * A1213 - coefficientRow1(1) * A0213 + coefficientRow1(2) * A0113);
	inv(15) = det * (coefficientRow1(0) * A1212 - coefficientRow1(1) * A0212 + coefficientRow1(2) * A0112);

	return inv;
}
