#include "SimplePIDController.hpp"

#include <gtest/gtest.h>

#include <math.h>

namespace
{

static constexpr float kMassKg = 3.0f;
static constexpr float kGravityMss = 9.80665f;
static constexpr float kPi = 3.14159265358979323846f;

SimplePIDController::Input make_hover_input()
{
	SimplePIDController::Input input{};
	input.timestamp_us = 1'000'000;
	input.position_ned[0] = 0.f;
	input.position_ned[1] = 0.f;
	input.position_ned[2] = -1.f;
	input.velocity_ned[0] = 0.f;
	input.velocity_ned[1] = 0.f;
	input.velocity_ned[2] = 0.f;
	input.quat_body_to_ned[0] = 1.f;
	input.quat_body_to_ned[1] = 0.f;
	input.quat_body_to_ned[2] = 0.f;
	input.quat_body_to_ned[3] = 0.f;
	input.angular_velocity_body[0] = 0.f;
	input.angular_velocity_body[1] = 0.f;
	input.angular_velocity_body[2] = 0.f;
	input.target_position_ned[0] = 0.f;
	input.target_position_ned[1] = 0.f;
	input.target_position_ned[2] = -1.f;
	input.target_yaw = 0.f;
	input.state_valid_for_control = true;
	input.armed = true;
	input.failsafe = false;
	return input;
}

} // namespace

TEST(SimplePIDController, HoverProducesWeightThrustAndZeroMoment)
{
	SimplePIDController controller;
	SimplePIDController::Input input = make_hover_input();
	SimplePIDController::Output output{};

	EXPECT_TRUE(controller.update(input, output));
	ASSERT_TRUE(output.valid);

	EXPECT_NEAR(output.thrust_newton, kMassKg * kGravityMss, 1e-3f);
	EXPECT_NEAR(output.desired_euler_rpy[0], 0.f, 1e-5f);
	EXPECT_NEAR(output.desired_euler_rpy[1], 0.f, 1e-5f);
	EXPECT_NEAR(output.desired_euler_rpy[2], 0.f, 1e-5f);
	EXPECT_NEAR(output.moment_newton_meter[0], 0.f, 1e-5f);
	EXPECT_NEAR(output.moment_newton_meter[1], 0.f, 1e-5f);
	EXPECT_NEAR(output.moment_newton_meter[2], 0.f, 1e-5f);
}

TEST(SimplePIDController, PositiveRollErrorCommandsNegativeRollMoment)
{
	SimplePIDController controller;
	SimplePIDController::Input input = make_hover_input();

	const float roll = 10.0f * kPi / 180.0f;
	input.quat_body_to_ned[0] = cosf(roll * 0.5f);
	input.quat_body_to_ned[1] = sinf(roll * 0.5f);
	input.quat_body_to_ned[2] = 0.f;
	input.quat_body_to_ned[3] = 0.f;

	SimplePIDController::Output output{};
	EXPECT_TRUE(controller.update(input, output));
	ASSERT_TRUE(output.valid);

	EXPECT_LT(output.attitude_error_rpy[0], 0.f);
	EXPECT_LT(output.moment_newton_meter[0], 0.f);
}

TEST(SimplePIDController, NorthPositionErrorCommandsNegativePitch)
{
	SimplePIDController controller;
	SimplePIDController::Input input = make_hover_input();
	input.target_position_ned[0] = 1.0f;

	SimplePIDController::Output output{};
	EXPECT_TRUE(controller.update(input, output));
	ASSERT_TRUE(output.valid);

	EXPECT_GT(output.desired_acceleration_ned[0], 0.f);
	EXPECT_LT(output.desired_euler_rpy[1], 0.f);
	EXPECT_LT(output.moment_newton_meter[1], 0.f);
}

TEST(SimplePIDController, IntegralAccumulatesForPersistentAttitudeError)
{
	SimplePIDController controller;
	SimplePIDController::Input input = make_hover_input();

	const float roll = 5.0f * kPi / 180.0f;
	input.quat_body_to_ned[0] = cosf(roll * 0.5f);
	input.quat_body_to_ned[1] = sinf(roll * 0.5f);

	SimplePIDController::Output first{};
	ASSERT_TRUE(controller.update(input, first));
	ASSERT_TRUE(first.valid);

	input.timestamp_us += 4'000;
	SimplePIDController::Output second{};
	ASSERT_TRUE(controller.update(input, second));
	ASSERT_TRUE(second.valid);

	EXPECT_LT(second.attitude_integral_rpy[0], first.attitude_integral_rpy[0]);
	EXPECT_LT(second.moment_newton_meter[0], first.moment_newton_meter[0]);
}

TEST(SimplePIDController, DisarmedInputResetsAndSuppressesOutput)
{
	SimplePIDController controller;
	SimplePIDController::Input input = make_hover_input();
	input.armed = false;

	SimplePIDController::Output output{};
	EXPECT_TRUE(controller.update(input, output));
	EXPECT_FALSE(output.valid);
	EXPECT_FLOAT_EQ(output.thrust_newton, 0.f);
	EXPECT_FLOAT_EQ(output.moment_newton_meter[0], 0.f);
	EXPECT_FLOAT_EQ(output.moment_newton_meter[1], 0.f);
	EXPECT_FLOAT_EQ(output.moment_newton_meter[2], 0.f);
}
