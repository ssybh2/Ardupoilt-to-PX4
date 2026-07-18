#include "TrajectoryGenerator.hpp"

#include <gtest/gtest.h>

#include <math.h>

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
input.current_yaw = 0.25f;
input.state_valid_for_control = true;
input.armed = true;
input.failsafe = false;
return input;
}

TrajectoryGenerator::Output update_at(TrajectoryGenerator &generator, hrt_abstime timestamp_us,
				      bool manual_enabled = false, bool manual_valid = false, float stick = 0.f)
{
TrajectoryGenerator::Input input = make_valid_input(timestamp_us);
input.manual_height_control_enabled = manual_enabled;
input.manual_height_control_valid = manual_valid;
input.manual_height_stick = stick;

TrajectoryGenerator::Output output{};
EXPECT_TRUE(generator.update(input, output));
EXPECT_TRUE(output.valid);
return output;
}

float norm2(float x, float y)
{
return sqrtf(x * x + y * y);
}

float dot2(float ax, float ay, float bx, float by)
{
return ax * bx + ay * by;
}

} // namespace

TEST(TrajectoryGenerator, ManualHeightControlIsIgnoredBeforeHover)
{
TrajectoryGenerator generator;

update_at(generator, 0);
const TrajectoryGenerator::Output output = update_at(generator, 1'000'000, true, true, 1.f);

EXPECT_EQ(output.mode, TrajectoryGenerator::Mode::Takeoff);
EXPECT_LT(output.position_ned[2], 0.f);
EXPECT_GT(output.position_ned[2], -1.f);
}

TEST(TrajectoryGenerator, TakeoffCompletesIntoHoverAtTakeoffTarget)
{
TrajectoryGenerator generator;

update_at(generator, 0);
const TrajectoryGenerator::Output hover = update_at(generator, 2'000'000);

EXPECT_EQ(hover.mode, TrajectoryGenerator::Mode::Hover);
EXPECT_NEAR(hover.position_ned[0], 0.f, kTolerance);
EXPECT_NEAR(hover.position_ned[1], 0.f, kTolerance);
EXPECT_NEAR(hover.position_ned[2], -generator.takeoff_height_m(), kTolerance);
EXPECT_NEAR(hover.velocity_ned[2], 0.f, kTolerance);
}

TEST(TrajectoryGenerator, HoverOutputPositionDoesNotDrift)
{
TrajectoryGenerator generator;

update_at(generator, 0);
const TrajectoryGenerator::Output first = update_at(generator, 2'000'000);
const TrajectoryGenerator::Output later = update_at(generator, 8'000'000);

EXPECT_EQ(first.mode, TrajectoryGenerator::Mode::Hover);
EXPECT_EQ(later.mode, TrajectoryGenerator::Mode::Hover);

for (int i = 0; i < 3; i++) {
EXPECT_NEAR(later.position_ned[i], first.position_ned[i], kTolerance);
EXPECT_NEAR(later.velocity_ned[i], 0.f, kTolerance);
EXPECT_NEAR(later.acceleration_ned[i], 0.f, kTolerance);
EXPECT_NEAR(later.jerk_ned[i], 0.f, kTolerance);
EXPECT_NEAR(later.snap_ned[i], 0.f, kTolerance);
}
}

TEST(TrajectoryGenerator, ManualHeightControlIntegratesNedVerticalTargetInHover)
{
TrajectoryGenerator generator;

update_at(generator, 0);
const TrajectoryGenerator::Output hover = update_at(generator, 2'000'000);
EXPECT_EQ(hover.mode, TrajectoryGenerator::Mode::Hover);
EXPECT_FLOAT_EQ(hover.position_ned[2], -1.f);

const TrajectoryGenerator::Output initialized = update_at(generator, 2'000'000, true, true, 0.f);
EXPECT_FLOAT_EQ(initialized.position_ned[2], -1.f);
EXPECT_FLOAT_EQ(initialized.velocity_ned[2], 0.f);

const TrajectoryGenerator::Output climb = update_at(generator, 3'000'000, true, true, 1.f);
EXPECT_FLOAT_EQ(climb.velocity_ned[2], -0.3f);
EXPECT_FLOAT_EQ(climb.position_ned[2], -1.03f);

const TrajectoryGenerator::Output descend = update_at(generator, 4'000'000, true, true, -1.f);
EXPECT_FLOAT_EQ(descend.velocity_ned[2], 0.3f);
EXPECT_FLOAT_EQ(descend.position_ned[2], -1.f);
}

TEST(TrajectoryGenerator, ManualHeightControlHoldsOnDeadzoneAndDisablesToDefaultHover)
{
TrajectoryGenerator generator;

update_at(generator, 0);
update_at(generator, 2'000'000);
update_at(generator, 2'000'000, true, true, 0.f);
update_at(generator, 3'000'000, true, true, 1.f);

const TrajectoryGenerator::Output deadzone = update_at(generator, 4'000'000, true, true, 0.05f);
EXPECT_FLOAT_EQ(deadzone.velocity_ned[2], 0.f);
EXPECT_FLOAT_EQ(deadzone.position_ned[2], -1.03f);

const TrajectoryGenerator::Output disabled = update_at(generator, 5'000'000, false, false, 1.f);
EXPECT_EQ(disabled.mode, TrajectoryGenerator::Mode::Hover);
EXPECT_FLOAT_EQ(disabled.position_ned[2], -1.f);
EXPECT_FLOAT_EQ(disabled.velocity_ned[2], 0.f);
}

TEST(TrajectoryGenerator, CircleFixedYawDerivativesAreConsistent)
{
TrajectoryGenerator generator;

update_at(generator, 0);
update_at(generator, 2'000'000);
generator.set_commanded_mode(TrajectoryGenerator::CommandedMode::Circle);

const TrajectoryGenerator::Output circle = update_at(generator, 2'000'000);
const float radius = generator.circle_radius_m();
const float speed = generator.circle_speed_m_s();
const float omega = speed / radius;

EXPECT_EQ(circle.mode, TrajectoryGenerator::Mode::Circle);
EXPECT_NEAR(circle.position_ned[0], radius, kTolerance);
EXPECT_NEAR(circle.position_ned[1], 0.f, kTolerance);
EXPECT_NEAR(circle.position_ned[2], -generator.takeoff_height_m(), kTolerance);
EXPECT_NEAR(circle.velocity_ned[0], 0.f, kTolerance);
EXPECT_NEAR(circle.velocity_ned[1], speed, kTolerance);
EXPECT_NEAR(circle.acceleration_ned[0], -radius * omega * omega, kTolerance);
EXPECT_NEAR(circle.acceleration_ned[1], 0.f, kTolerance);
EXPECT_NEAR(dot2(circle.position_ned[0], circle.position_ned[1],
		 circle.velocity_ned[0], circle.velocity_ned[1]), 0.f, kTolerance);
EXPECT_NEAR(norm2(circle.velocity_ned[0], circle.velocity_ned[1]), speed, kTolerance);
EXPECT_NEAR(norm2(circle.acceleration_ned[0], circle.acceleration_ned[1]), speed * speed / radius, kTolerance);
}

TEST(TrajectoryGenerator, ManualHeightControlAdjustsCircleAltitudeWithoutResettingHorizontalMotion)
{
TrajectoryGenerator generator;

update_at(generator, 0);
update_at(generator, 2'000'000);
generator.set_commanded_mode(TrajectoryGenerator::CommandedMode::Circle);

const TrajectoryGenerator::Output start = update_at(generator, 2'000'000);
const TrajectoryGenerator::Output initialized = update_at(generator, 2'000'000, true, true, 0.f);
const TrajectoryGenerator::Output climb = update_at(generator, 3'000'000, true, true, 1.f);

EXPECT_EQ(initialized.mode, TrajectoryGenerator::Mode::Circle);
EXPECT_EQ(climb.mode, TrajectoryGenerator::Mode::Circle);
EXPECT_NEAR(climb.position_ned[2], start.position_ned[2] - 0.03f, kTolerance);
EXPECT_NEAR(climb.velocity_ned[2], -0.3f, kTolerance);
EXPECT_NEAR(climb.acceleration_ned[2], 0.f, kTolerance);
EXPECT_NEAR(climb.jerk_ned[2], 0.f, kTolerance);
EXPECT_NEAR(climb.snap_ned[2], 0.f, kTolerance);
EXPECT_NEAR(norm2(climb.velocity_ned[0], climb.velocity_ned[1]), generator.circle_speed_m_s(), kTolerance);
EXPECT_NEAR(norm2(climb.acceleration_ned[0], climb.acceleration_ned[1]),
	    generator.circle_speed_m_s() * generator.circle_speed_m_s() / generator.circle_radius_m(), kTolerance);
EXPECT_GT(fabsf(climb.position_ned[1]), 0.1f);
}

TEST(TrajectoryGenerator, ManualHeightControlContinuouslyAdjustsCircleAltitudeAndThenHolds)
{
TrajectoryGenerator generator;

update_at(generator, 0);
update_at(generator, 2'000'000);
generator.set_commanded_mode(TrajectoryGenerator::CommandedMode::Circle);

update_at(generator, 2'000'000, true, true, 0.f);

TrajectoryGenerator::Output climb{};

for (int i = 1; i <= 10; i++) {
	climb = update_at(generator, 2'000'000 + static_cast<hrt_abstime>(i * 100'000), true, true, 1.f);
}

EXPECT_EQ(climb.mode, TrajectoryGenerator::Mode::Circle);
EXPECT_NEAR(climb.position_ned[2], -1.3f, kTolerance);
EXPECT_NEAR(climb.velocity_ned[2], -0.3f, kTolerance);
EXPECT_NEAR(norm2(climb.velocity_ned[0], climb.velocity_ned[1]), generator.circle_speed_m_s(), kTolerance);

const TrajectoryGenerator::Output hold = update_at(generator, 3'100'000, true, true, 0.f);
EXPECT_EQ(hold.mode, TrajectoryGenerator::Mode::Circle);
EXPECT_NEAR(hold.position_ned[2], climb.position_ned[2], kTolerance);
EXPECT_NEAR(hold.velocity_ned[2], 0.f, kTolerance);
EXPECT_NEAR(norm2(hold.velocity_ned[0], hold.velocity_ned[1]), generator.circle_speed_m_s(), kTolerance);
EXPECT_GT(fabsf(hold.position_ned[1]), fabsf(climb.position_ned[1]));
}

TEST(TrajectoryGenerator, CircleReturnsNearStartAfterOnePeriod)
{
TrajectoryGenerator generator;

update_at(generator, 0);
update_at(generator, 2'000'000);
generator.set_commanded_mode(TrajectoryGenerator::CommandedMode::Circle);

const TrajectoryGenerator::Output start = update_at(generator, 2'000'000);
const hrt_abstime period_us = static_cast<hrt_abstime>(generator.circle_period_s() * 1e6f);
const TrajectoryGenerator::Output end = update_at(generator, 2'000'000 + period_us);

EXPECT_EQ(end.mode, TrajectoryGenerator::Mode::Circle);

for (int i = 0; i < 3; i++) {
EXPECT_NEAR(end.position_ned[i], start.position_ned[i], 2e-4f);
EXPECT_NEAR(end.velocity_ned[i], start.velocity_ned[i], 2e-4f);
}
}

TEST(TrajectoryGenerator, ResetClearsCirclePhaseAndMode)
{
TrajectoryGenerator generator;

update_at(generator, 0);
update_at(generator, 2'000'000);
generator.set_commanded_mode(TrajectoryGenerator::CommandedMode::Circle);
update_at(generator, 3'000'000);

generator.reset();
generator.set_commanded_mode(TrajectoryGenerator::CommandedMode::Circle);

update_at(generator, 10'000'000);
update_at(generator, 12'000'000);
const TrajectoryGenerator::Output circle = update_at(generator, 12'000'000);

EXPECT_EQ(circle.mode, TrajectoryGenerator::Mode::Circle);
EXPECT_NEAR(circle.position_ned[0], generator.circle_radius_m(), kTolerance);
EXPECT_NEAR(circle.position_ned[1], 0.f, kTolerance);
}

TEST(TrajectoryGenerator, FixedYawCircleKeepsYawRateAndAccelerationZero)
{
TrajectoryGenerator generator;

update_at(generator, 0);
update_at(generator, 2'000'000);
generator.set_commanded_mode(TrajectoryGenerator::CommandedMode::Circle);

const TrajectoryGenerator::Output circle = update_at(generator, 4'000'000);

EXPECT_EQ(circle.mode, TrajectoryGenerator::Mode::Circle);
EXPECT_NEAR(circle.yaw, 0.25f, kTolerance);
EXPECT_NEAR(circle.yaw_rate, 0.f, kTolerance);
EXPECT_NEAR(circle.yaw_accel, 0.f, kTolerance);
}
