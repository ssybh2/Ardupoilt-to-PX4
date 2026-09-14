#include "GeometricController.hpp"

#include <gtest/gtest.h>

#include <math.h>

namespace
{

static constexpr float kGravityMss = 9.80665f;

GeometricController::Input make_hover_input()
{
	GeometricController::Input input{};
	input.timestamp_us = 1'000'000;
	input.position_ned[2] = -1.f;
	input.target_position_ned[2] = -1.f;
	input.quat_body_to_ned[0] = 1.f;
	input.target_yaw[0] = 1.f;
	input.target_yaw[1] = 0.f;
	input.target_yaw_dot[0] = 0.f;
	input.target_yaw_dot[1] = 0.f;
	input.target_yaw_ddot[0] = 0.f;
	input.target_yaw_ddot[1] = 0.f;
	input.state_valid_for_control = true;
	input.armed = true;
	input.failsafe = false;
	return input;
}

float norm3(const float v[3])
{
	return sqrtf(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

} // namespace

TEST(GeometricController, UploadedDSunHoverMatchesOriginalControllerEquation)
{
	GeometricController controller;
	GeometricController::Parameters parameters{}; // uploaded DSun real-aircraft source values
	controller.set_parameters(parameters);

	GeometricController::Output output{};
	EXPECT_TRUE(controller.update(make_hover_input(), output));
	ASSERT_TRUE(output.valid);

	EXPECT_NEAR(output.r_error[0], 0.f, 1e-6f);
	EXPECT_NEAR(output.r_error[1], 0.f, 1e-6f);
	EXPECT_NEAR(output.r_error[2], 0.f, 1e-6f);
	EXPECT_NEAR(output.v_error[0], 0.f, 1e-6f);
	EXPECT_NEAR(output.v_error[1], 0.f, 1e-6f);
	EXPECT_NEAR(output.v_error[2], 0.f, 1e-6f);
	EXPECT_NEAR(output.target_thrust, parameters.mass_kg * kGravityMss, 1e-4f);
	EXPECT_NEAR(norm3(output.M), 0.f, 1e-5f);
}

TEST(GeometricController, TargetYawIsOriginalCosSinVectorNotScalarYaw)
{
	GeometricController controller;
	GeometricController::Input input = make_hover_input();

	const float yaw = 0.4f;
	input.target_yaw[0] = cosf(yaw);
	input.target_yaw[1] = sinf(yaw);

	GeometricController::Output output{};
	EXPECT_TRUE(controller.update(input, output));
	ASSERT_TRUE(output.valid);

	// The source controller accepts [cos(yaw), sin(yaw)] directly.  With a level
	// vehicle at yaw=0 and a nonzero desired yaw it must request a yaw moment.
	EXPECT_GT(fabsf(output.M[2]), 1e-5f);
}

TEST(GeometricController, InvalidPX4StateIsRejectedOutsideNumericalCore)
{
	GeometricController controller;
	GeometricController::Input input = make_hover_input();
	input.failsafe = true;

	GeometricController::Output output{};
	EXPECT_TRUE(controller.update(input, output));
	EXPECT_FALSE(output.valid);
	EXPECT_FLOAT_EQ(output.target_thrust, 0.f);
	EXPECT_FLOAT_EQ(output.M[0], 0.f);
	EXPECT_FLOAT_EQ(output.M[1], 0.f);
	EXPECT_FLOAT_EQ(output.M[2], 0.f);
}
