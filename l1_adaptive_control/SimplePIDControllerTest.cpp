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
	input.position_ned[2] = -1.f;
	input.target_position_ned[2] = -1.f;

	const float half_yaw = 0.5f * yaw_rad;
	input.quat_body_to_ned[0] = cosf(half_yaw);
	input.quat_body_to_ned[1] = 0.f;
	input.quat_body_to_ned[2] = 0.f;
	input.quat_body_to_ned[3] = sinf(half_yaw);

	input.target_yaw = yaw_rad;
	input.state_valid_for_control = true;
	input.armed = true;
	input.failsafe = false;
	return input;
}

} // namespace

TEST(DSunGeometricAdapter, HoverProducesWeightThrustAndNoMoment)
{
	SimplePIDController controller;
	SimplePIDController::Input input = make_hover_input();
	SimplePIDController::Output output{};

	EXPECT_TRUE(controller.update(input, output));
	ASSERT_TRUE(output.valid);
	EXPECT_NEAR(output.thrust_newton, kMassKg * kGravityMss, 1e-3f);
	EXPECT_NEAR(output.moment_newton_meter[0], 0.f, 1e-5f);
	EXPECT_NEAR(output.moment_newton_meter[1], 0.f, 1e-5f);
	EXPECT_NEAR(output.moment_newton_meter[2], 0.f, 1e-6f);
}

TEST(DSunGeometricAdapter, YawSpinIsIntentionallyUncontrolled)
{
	SimplePIDController controller;
	SimplePIDController::Input input = make_hover_input(1.2f);
	input.angular_velocity_body[2] = 4.0f;
	input.target_yaw = -2.0f;
	input.target_yaw_rate = 0.f;
	SimplePIDController::Output output{};

	EXPECT_TRUE(controller.update(input, output));
	ASSERT_TRUE(output.valid);
	EXPECT_TRUE(isfinite(output.thrust_newton));
	EXPECT_NEAR(output.moment_newton_meter[2], 0.f, 1e-6f);
}

TEST(DSunGeometricAdapter, HorizontalPositionErrorProducesCorrectiveMoment)
{
	SimplePIDController controller;
	SimplePIDController::Input input = make_hover_input(1.57079632679f);
	input.position_ned[0] = 0.2f;
	SimplePIDController::Output output{};

	EXPECT_TRUE(controller.update(input, output));
	ASSERT_TRUE(output.valid);
	EXPECT_TRUE(isfinite(output.moment_newton_meter[0]));
	EXPECT_TRUE(isfinite(output.moment_newton_meter[1]));
	EXPECT_GT(fabsf(output.moment_newton_meter[0]) + fabsf(output.moment_newton_meter[1]), 1e-5f);
	EXPECT_NEAR(output.moment_newton_meter[2], 0.f, 1e-6f);
}

TEST(DSunGeometricAdapter, InvalidStateProducesZeroOutput)
{
	SimplePIDController controller;
	SimplePIDController::Input input = make_hover_input();
	input.state_valid_for_control = false;
	SimplePIDController::Output output{};

	EXPECT_TRUE(controller.update(input, output));
	EXPECT_FALSE(output.valid);
	EXPECT_NEAR(output.thrust_newton, 0.f, 1e-6f);
	EXPECT_NEAR(output.moment_newton_meter[0], 0.f, 1e-6f);
	EXPECT_NEAR(output.moment_newton_meter[1], 0.f, 1e-6f);
	EXPECT_NEAR(output.moment_newton_meter[2], 0.f, 1e-6f);
}
