#include "L1AdaptiveAugmentation.hpp"

#include <gtest/gtest.h>

namespace
{

static constexpr float kMassKg = 3.0f;
static constexpr float kGravityMss = 9.80665f;

L1AdaptiveAugmentation::Input make_hover_input()
{
	L1AdaptiveAugmentation::Input input{};
	input.timestamp_us = 1'000'000;
	input.quat_body_to_ned[0] = 1.f;
	input.baseline_thrust_moment[0] = kMassKg * kGravityMss;
	input.baseline_valid = true;
	input.state_valid = true;
	input.armed = true;
	input.failsafe = false;
	return input;
}

void expect_zero_adaptive(const L1AdaptiveAugmentation::Output &output)
{
	for (int i = 0; i < 4; i++) {
		EXPECT_NEAR(output.adaptive_thrust_moment[i], 0.f, 1e-5f);
	}
}

} // namespace

TEST(L1AdaptiveAugmentation, HoverEquilibriumProducesZeroAdaptiveCommand)
{
	L1AdaptiveAugmentation augmentation;
	L1AdaptiveAugmentation::Input input = make_hover_input();
	L1AdaptiveAugmentation::Output output{};

	EXPECT_TRUE(augmentation.update(input, output));
	ASSERT_TRUE(output.valid);
	expect_zero_adaptive(output);
	EXPECT_NEAR(output.combined_thrust_moment[0], kMassKg * kGravityMss, 1e-4f);

	input.timestamp_us += 4'000;
	EXPECT_TRUE(augmentation.update(input, output));
	ASSERT_TRUE(output.valid);
	expect_zero_adaptive(output);
	EXPECT_NEAR(output.combined_thrust_moment[0], kMassKg * kGravityMss, 1e-4f);
}

TEST(L1AdaptiveAugmentation, InvalidInputResetsAndReturnsZeroOutput)
{
	L1AdaptiveAugmentation augmentation;
	L1AdaptiveAugmentation::Input input = make_hover_input();
	L1AdaptiveAugmentation::Output output{};

	ASSERT_TRUE(augmentation.update(input, output));

	input.timestamp_us += 4'000;
	input.armed = false;
	EXPECT_FALSE(augmentation.update(input, output));
	EXPECT_FALSE(output.valid);

	for (int i = 0; i < 4; i++) {
		EXPECT_FLOAT_EQ(output.adaptive_thrust_moment[i], 0.f);
		EXPECT_FLOAT_EQ(output.combined_thrust_moment[i], 0.f);
	}

	input.timestamp_us += 4'000;
	input.armed = true;
	EXPECT_TRUE(augmentation.update(input, output));
	ASSERT_TRUE(output.valid);
	expect_zero_adaptive(output);
}
