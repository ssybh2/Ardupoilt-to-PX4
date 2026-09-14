#pragma once

#include "L1SourceConfig.hpp"

#include <matrix/matrix/math.hpp>

class MotorMixer
{
public:
	using ThrustMoment = matrix::Vector4f;
	using MotorCommand = matrix::Vector4f;

	struct Parameters {
		bool real_vehicle{REAL_OR_SITL != 0};
	};

	bool mix(const ThrustMoment &thrust_moment_cmd, MotorCommand &motor_command) const;
	void set_parameters(const Parameters &parameters) { _parameters = parameters; }
	const Parameters &parameters() const { return _parameters; }

private:
	using Vector16f = matrix::Vector<float, 16>;

	MotorCommand iterativeMotorMixing(const MotorCommand &w_input,
					   const ThrustMoment &thrust_moment_cmd,
					   float a_F, float b_F, float a_M, float b_M,
					   float L, float D) const;

	Vector16f mat4Inv(const MotorCommand &coefficient_row1,
			  const MotorCommand &coefficient_row2,
			  const MotorCommand &coefficient_row3,
			  const MotorCommand &coefficient_row4) const;

	Parameters _parameters{};
};
