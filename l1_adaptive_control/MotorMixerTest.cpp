#include "MotorMixer.hpp"

#include <gtest/gtest.h>

namespace
{

static constexpr float kHoverThrust = 3.f * 9.80665f;

} // namespace

TEST(MotorMixer, SitlHoverMatchesOriginalThreeIterationMixer)
{
	MotorMixer mixer;
	MotorMixer::Parameters parameters{};
	parameters.real_vehicle = false;
	mixer.set_parameters(parameters);

	MotorMixer::ThrustMoment command{};
	command(0) = kHoverThrust;

	MotorMixer::MotorCommand output{};
	ASSERT_TRUE(mixer.mix(command, output));

	for (int i = 0; i < 4; i++) {
		EXPECT_NEAR(output(i), 57.57798f, 2e-4f);
	}
}

TEST(MotorMixer, SitlRollMomentKeepsOriginalXLayoutSigns)
{
	MotorMixer mixer;
	MotorMixer::Parameters parameters{};
	parameters.real_vehicle = false;
	mixer.set_parameters(parameters);

	MotorMixer::ThrustMoment command{};
	command(0) = kHoverThrust;
	command(1) = 0.1f;

	MotorMixer::MotorCommand output{};
	ASSERT_TRUE(mixer.mix(command, output));

	EXPECT_NEAR(output(0), 56.62741f, 3e-4f);
	EXPECT_NEAR(output(1), 58.51626f, 3e-4f);
	EXPECT_NEAR(output(2), 58.51626f, 3e-4f);
	EXPECT_NEAR(output(3), 56.62741f, 3e-4f);
}
