#include "TrajectoryGenerator.hpp"

#include <gtest/gtest.h>

namespace
{

static constexpr float kTolerance = 1e-4f;

TrajectoryGenerator::Input make_input(hrt_abstime timestamp_us, bool armed = true)
{
	TrajectoryGenerator::Input input{};
	input.timestamp_us = timestamp_us;
	input.current_position_ned[0] = 0.f;
	input.current_position_ned[1] = 0.f;
	input.current_position_ned[2] = 0.f;
	input.current_yaw = 0.25f;
	input.state_valid_for_control = true;
	input.armed = armed;
	input.failsafe = false;
	input.manual_height_control_enabled = true;
	input.manual_height_control_valid = true;
	return input;
}

TrajectoryGenerator::Output run_update(TrajectoryGenerator &generator, const TrajectoryGenerator::Input &input)
{
	TrajectoryGenerator::Output output{};
	EXPECT_TRUE(generator.update(input, output));
	return output;
}

} // namespace

TEST(TrajectoryGenerator, DoesNotAutoTakeoffWhenArmed)
{
	TrajectoryGenerator generator;
	const auto output = run_update(generator, make_input(0, true));

	EXPECT_EQ(output.mode, TrajectoryGenerator::Mode::WaitForValidState);
	EXPECT_FALSE(output.valid);
	EXPECT_FLOAT_EQ(output.position_ned[2], 0.f);
}

TEST(TrajectoryGenerator, TakeoffCommandCanLatchBeforeCommanderArms)
{
	TrajectoryGenerator generator;

	auto command = make_input(0, false);
	command.manual_height_stick = TrajectoryGenerator::KEYBOARD_TAKEOFF_COMMAND_STICK;
	const auto waiting = run_update(generator, command);
	EXPECT_EQ(waiting.mode, TrajectoryGenerator::Mode::WaitForValidState);
	EXPECT_FALSE(waiting.valid);

	auto armed = make_input(200'000, true);
	armed.manual_height_stick = 0.f;
	const auto takeoff = run_update(generator, armed);
	EXPECT_EQ(takeoff.mode, TrajectoryGenerator::Mode::Takeoff);
	EXPECT_TRUE(takeoff.valid);
}

TEST(TrajectoryGenerator, KeyboardOneTakesOffAndThenHoversAtCapturedPoint)
{
	TrajectoryGenerator generator;

	auto start = make_input(0, true);
	start.current_position_ned[0] = 2.f;
	start.current_position_ned[1] = -3.f;
	start.current_position_ned[2] = 0.4f;
	start.manual_height_stick = TrajectoryGenerator::KEYBOARD_TAKEOFF_COMMAND_STICK;
	const auto first = run_update(generator, start);
	EXPECT_EQ(first.mode, TrajectoryGenerator::Mode::Takeoff);

	auto hover_input = start;
	hover_input.timestamp_us = static_cast<hrt_abstime>(generator.takeoff_duration_s() * 1e6f);
	hover_input.manual_height_stick = 0.f;
	const auto hover = run_update(generator, hover_input);

	EXPECT_EQ(hover.mode, TrajectoryGenerator::Mode::Hover);
	EXPECT_TRUE(hover.valid);
	EXPECT_NEAR(hover.position_ned[0], 2.f, kTolerance);
	EXPECT_NEAR(hover.position_ned[1], -3.f, kTolerance);
	EXPECT_NEAR(hover.position_ned[2], 0.4f - generator.takeoff_height_m(), kTolerance);
	EXPECT_NEAR(hover.velocity_ned[2], 0.f, kTolerance);
}

TEST(TrajectoryGenerator, ManualHeightControlIsIgnoredDuringTakeoff)
{
	TrajectoryGenerator generator;

	auto start = make_input(0, true);
	start.manual_height_stick = TrajectoryGenerator::KEYBOARD_TAKEOFF_COMMAND_STICK;
	run_update(generator, start);

	auto climb = make_input(1'000'000, true);
	climb.manual_height_stick = 1.f;
	const auto output = run_update(generator, climb);

	EXPECT_EQ(output.mode, TrajectoryGenerator::Mode::Takeoff);
	EXPECT_LT(output.position_ned[2], 0.f);
	EXPECT_GT(output.position_ned[2], -generator.takeoff_height_m());
}

TEST(TrajectoryGenerator, WAndSStyleHeightStickChangesHoverAltitude)
{
	TrajectoryGenerator generator;

	auto start = make_input(0, true);
	start.manual_height_stick = TrajectoryGenerator::KEYBOARD_TAKEOFF_COMMAND_STICK;
	run_update(generator, start);

	auto hover = make_input(2'000'000, true);
	hover.manual_height_stick = 0.f;
	run_update(generator, hover);

	auto init_manual = make_input(2'000'000, true);
	init_manual.manual_height_stick = 0.f;
	run_update(generator, init_manual);

	auto climb = make_input(3'000'000, true);
	climb.manual_height_stick = 1.f;
	const auto climb_output = run_update(generator, climb);
	EXPECT_EQ(climb_output.mode, TrajectoryGenerator::Mode::Hover);
	EXPECT_FLOAT_EQ(climb_output.velocity_ned[2], -0.3f);
	EXPECT_NEAR(climb_output.position_ned[2], -1.03f, kTolerance);

	auto descend = make_input(4'000'000, true);
	descend.manual_height_stick = -1.f;
	const auto descend_output = run_update(generator, descend);
	EXPECT_FLOAT_EQ(descend_output.velocity_ned[2], 0.3f);
	EXPECT_NEAR(descend_output.position_ned[2], -1.f, kTolerance);
}

TEST(TrajectoryGenerator, KeyboardTwoStartsSmoothLandingAndReachesLandedState)
{
	TrajectoryGenerator generator;

	auto start = make_input(0, true);
	start.manual_height_stick = TrajectoryGenerator::KEYBOARD_TAKEOFF_COMMAND_STICK;
	run_update(generator, start);

	auto hover = make_input(2'000'000, true);
	hover.current_position_ned[2] = -1.f;
	hover.manual_height_stick = 0.f;
	run_update(generator, hover);

	auto land = hover;
	land.timestamp_us = 3'000'000;
	land.manual_height_stick = TrajectoryGenerator::KEYBOARD_LAND_COMMAND_STICK;
	const auto landing_start = run_update(generator, land);
	EXPECT_EQ(landing_start.mode, TrajectoryGenerator::Mode::Landing);
	EXPECT_NEAR(landing_start.position_ned[2], -1.f, kTolerance);
	EXPECT_NEAR(landing_start.velocity_ned[2], 0.f, kTolerance);

	auto mid = hover;
	mid.timestamp_us = 5'000'000;
	mid.current_position_ned[2] = -0.5f;
	mid.manual_height_stick = 0.f;
	const auto landing_mid = run_update(generator, mid);
	EXPECT_EQ(landing_mid.mode, TrajectoryGenerator::Mode::Landing);
	EXPECT_GT(landing_mid.position_ned[2], -1.f);
	EXPECT_LT(landing_mid.position_ned[2], 0.f);

	auto end = hover;
	end.timestamp_us = 7'000'000;
	end.current_position_ned[2] = 0.f;
	end.manual_height_stick = 0.f;
	const auto landed = run_update(generator, end);
	EXPECT_EQ(landed.mode, TrajectoryGenerator::Mode::Landed);
	EXPECT_GT(landed.position_ned[2], 0.f);
	EXPECT_NEAR(landed.velocity_ned[2], 0.f, kTolerance);
}

TEST(TrajectoryGenerator, HeightStickDoesNotOverrideLanding)
{
	TrajectoryGenerator generator;

	auto start = make_input(0, true);
	start.manual_height_stick = TrajectoryGenerator::KEYBOARD_TAKEOFF_COMMAND_STICK;
	run_update(generator, start);

	auto hover = make_input(2'000'000, true);
	hover.current_position_ned[2] = -1.f;
	hover.manual_height_stick = 0.f;
	run_update(generator, hover);

	auto land = hover;
	land.timestamp_us = 2'100'000;
	land.manual_height_stick = TrajectoryGenerator::KEYBOARD_LAND_COMMAND_STICK;
	run_update(generator, land);

	auto climb = hover;
	climb.timestamp_us = 3'100'000;
	climb.manual_height_stick = 1.f;
	const auto output = run_update(generator, climb);
	EXPECT_EQ(output.mode, TrajectoryGenerator::Mode::Landing);
	EXPECT_GT(output.velocity_ned[2], 0.f);
}

TEST(TrajectoryGenerator, LegacyCircleCommandNoLongerGeneratesCircle)
{
	TrajectoryGenerator generator;
	generator.set_commanded_mode(TrajectoryGenerator::CommandedMode::Circle);
	EXPECT_EQ(generator.commanded_mode(), TrajectoryGenerator::CommandedMode::Hover);
}
