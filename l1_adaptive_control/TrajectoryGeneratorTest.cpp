#include "TrajectoryGenerator.hpp"

#include <gtest/gtest.h>

namespace
{

static constexpr float kTolerance = 1e-4f;

TrajectoryGenerator::Input make_valid_input(hrt_abstime timestamp_us)
{
	TrajectoryGenerator::Input input{};
	input.timestamp_us = timestamp_us;
	input.current_position_ned[0] = 0.f;
	input.current_position_ned[1] = 0.f;
	input.current_position_ned[2] = 0.f;
	input.current_velocity_ned[0] = 0.f;
	input.current_velocity_ned[1] = 0.f;
	input.current_velocity_ned[2] = 0.f;
	input.current_yaw = 0.f;
	input.state_valid_for_control = true;
	input.armed = true;
	input.failsafe = false;
	return input;
}

TrajectoryGenerator::Output update_at(TrajectoryGenerator &generator, hrt_abstime timestamp_us)
{
	TrajectoryGenerator::Output output{};
	EXPECT_TRUE(generator.update(make_valid_input(timestamp_us), output));
	EXPECT_TRUE(output.valid);
	return output;
}

} // namespace

TEST(TrajectoryGenerator, TakeoffUsesOriginalACRLPolynomial)
{
	TrajectoryGenerator generator;

	// First valid sample corresponds to entering ModeAdaptive and timeInThisRun = 0.
	update_at(generator, 1'000'000);
	const TrajectoryGenerator::Output output = update_at(generator, 2'000'000);

	// Original ACRL polynomial at t=1 s:
	// p(t) = -0.1563 t^7 + 1.0938 t^6 - 2.6250 t^5 + 2.1875 t^4.
	EXPECT_NEAR(output.time_in_this_run_s, 1.f, kTolerance);
	EXPECT_NEAR(output.position_ned[0], 0.f, kTolerance);
	EXPECT_NEAR(output.position_ned[1], 0.f, kTolerance);
	EXPECT_NEAR(output.position_ned[2], -0.5f, kTolerance);
	EXPECT_NEAR(output.velocity_ned[2], -1.0937f, 2e-4f);
	EXPECT_NEAR(output.yaw[0], 1.f, kTolerance);
	EXPECT_NEAR(output.yaw[1], 0.f, kTolerance);
	EXPECT_NEAR(output.yaw_dot[0], 0.f, kTolerance);
	EXPECT_NEAR(output.yaw_dot[1], 0.f, kTolerance);
}

TEST(TrajectoryGenerator, FixedYawCircleUsesOriginalACRLInitialState)
{
	TrajectoryGenerator generator;
	TrajectoryGenerator::Parameters parameters{};
	parameters.trajectory_index = 2;
	parameters.radius_x = 1.f;
	parameters.radius_y = 1.f;
	parameters.target_speed = 0.5f;
	generator.set_parameters(parameters);

	update_at(generator, 1'000'000);
	const TrajectoryGenerator::Output circle = update_at(generator, 3'000'000);

	// Original SITL circle_fixed_yaw at timeInThisRun=2, radius=1, speed=0.5.
	EXPECT_NEAR(circle.time_in_this_run_s, 2.f, kTolerance);
	EXPECT_NEAR(circle.position_ned[0], 0.f, kTolerance);
	EXPECT_NEAR(circle.position_ned[1], 0.f, kTolerance);
	EXPECT_NEAR(circle.position_ned[2], -1.f, kTolerance);
	EXPECT_NEAR(circle.velocity_ned[0], 0.5f, kTolerance);
	EXPECT_NEAR(circle.velocity_ned[1], 0.f, kTolerance);
	EXPECT_NEAR(circle.acceleration_ned[0], 0.f, kTolerance);
	EXPECT_NEAR(circle.acceleration_ned[1], 0.25f, kTolerance);
	EXPECT_NEAR(circle.jerk_ned[0], -0.125f, kTolerance);
	EXPECT_NEAR(circle.snap_ned[1], -0.0625f, kTolerance);
	EXPECT_NEAR(circle.yaw[0], 1.f, kTolerance);
	EXPECT_NEAR(circle.yaw[1], 0.f, kTolerance);
}

TEST(TrajectoryGenerator, InvalidStateResetsSourceRunTiming)
{
	TrajectoryGenerator generator;
	update_at(generator, 1'000'000);
	update_at(generator, 1'500'000);

	TrajectoryGenerator::Input invalid = make_valid_input(2'000'000);
	invalid.armed = false;
	TrajectoryGenerator::Output invalid_output{};
	EXPECT_FALSE(generator.update(invalid, invalid_output));
	EXPECT_FALSE(invalid_output.valid);

	const TrajectoryGenerator::Output restarted = update_at(generator, 10'000'000);
	EXPECT_NEAR(restarted.time_in_this_run_s, 0.f, kTolerance);
	EXPECT_NEAR(restarted.position_ned[2], 0.f, kTolerance);
}
