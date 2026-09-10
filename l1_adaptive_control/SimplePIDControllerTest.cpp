#include "SimplePIDController.hpp"

#include <gtest/gtest.h>

#include <math.h>

namespace
{

static constexpr float kMassKg = 3.0f;
static constexpr float kGravityMss = 9.80665f;

SimplePIDController::Input make_hover_input(float yaw_rad = 0.f)
{
	SimplePIDController::Input input{};
	input.timestamp_us = 1'000'000;
	input.position_ned[0] = 0.f;
	input.position_ned[1] = 0.f;
	input.position_ned[2] = -1.f;
	input.target_position_ned[0] = 0.f;
	input.target_position_ned[1] = 0.f;
	input.target_position_ned[2] = -1.f;

	const float half_yaw = 0.5f * yaw_rad;
	input.quat_body_to_ned[0] = cosf(half_yaw);
	input.quat_body_to_ned[1] = 0.f;
	input.quat_body_to_ned[2] = 0.f;
	input.quat_body_to_ned[3] = sinf(half_yaw);

	input.state_valid_for_control = true;
	input.armed = true;
	input.failsafe = false;
	return input;
}

} // namespace

TEST(SimplePIDController, HoverProducesWeightThrustAndNoMoment)
{
	SimplePIDController controller;
	SimplePIDController::Input input = make_hover_input();
	SimplePIDController::Output output{};

	EXPECT_TRUE(controller.update(input, output));
	ASSERT_TRUE(output.valid);
	EXPECT_NEAR(output.thrust_newton, kMassKg * kGravityMss, 1e-3f);
	EXPECT_NEAR(output.moment_newton_meter[0], 0.f, 1e-6f);
	EXPECT_NEAR(output.moment_newton_meter[1], 0.f, 1e-6f);
	EXPECT_NEAR(output.moment_newton_meter[2], 0.f, 1e-6f);
}

TEST(SimplePIDController, YawSpinIsIntentionallyUncontrolled)
{
	SimplePIDController controller;
	SimplePIDController::Input input = make_hover_input(1.2f);
	input.angular_velocity_body[2] = 4.0f;
	input.target_yaw = -2.0f; // Deliberately different: spin-hover must ignore it.
	input.target_yaw_rate = 0.f;
	SimplePIDController::Output output{};

	EXPECT_TRUE(controller.update(input, output));
	ASSERT_TRUE(output.valid);
	EXPECT_NEAR(output.desired_euler_rpy[2], output.current_euler_rpy[2], 1e-5f);
	EXPECT_NEAR(output.attitude_error_rpy[2], 0.f, 1e-6f);
	EXPECT_NEAR(output.rate_error_body[2], 0.f, 1e-6f);
	EXPECT_NEAR(output.moment_newton_meter[2], 0.f, 1e-6f);
}

TEST(SimplePIDController, HorizontalCorrectionRotatesWithCurrentYaw)
{
	SimplePIDController controller;
	SimplePIDController::Input input = make_hover_input(1.57079632679f); // +90 deg yaw

	// Vehicle is 0.2 m north of the target. The controller should command a
	// body-frame tilt using the current yaw rather than a fixed yaw reference.
	input.position_ned[0] = 0.2f;
	SimplePIDController::Output output{};

	EXPECT_TRUE(controller.update(input, output));
	ASSERT_TRUE(output.valid);
	EXPECT_TRUE(isfinite(output.desired_euler_rpy[0]));
	EXPECT_TRUE(isfinite(output.desired_euler_rpy[1]));
	EXPECT_GT(fabsf(output.desired_euler_rpy[0]) + fabsf(output.desired_euler_rpy[1]), 1e-4f);
	EXPECT_NEAR(output.moment_newton_meter[2], 0.f, 1e-6f);
}
