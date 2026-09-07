#include <gtest/gtest.h>

#include "explorer_bridge/pose_gate.hpp"

TEST(PoseGateHarness, RunnerExecutesAssertions)
{
  EXPECT_EQ(1 + 1, 2);
}

TEST(PoseGateHarness, IntentionalFailureIsDetectable_NegativeControl)
{
  EXPECT_FALSE(1 == 2);
}

TEST(PoseGate, FirstPoseAlwaysIntegrates_Positive)
{
  explorer_bridge::Pose2D pose{0.0, 0.0, 0.0};
  EXPECT_TRUE(explorer_bridge::shouldIntegratePose(pose, std::nullopt, 0.25, 0.17));
}

TEST(PoseGate, MoveQuarterMeterIntegrates_Positive)
{
  explorer_bridge::Pose2D last{0.0, 0.0, 0.0};
  explorer_bridge::Pose2D pose{0.25, 0.0, 0.0};
  EXPECT_TRUE(explorer_bridge::shouldIntegratePose(pose, last, 0.25, 0.17));
}

TEST(PoseGate, SubThresholdMotionSkipped_Negative)
{
  explorer_bridge::Pose2D last{0.0, 0.0, 0.0};
  explorer_bridge::Pose2D pose{0.10, 0.0, 0.05};
  EXPECT_FALSE(explorer_bridge::shouldIntegratePose(pose, last, 0.25, 0.17));
}

TEST(PoseGate, YawTenDegreesIntegrates_Positive)
{
  explorer_bridge::Pose2D last{0.0, 0.0, 0.0};
  explorer_bridge::Pose2D pose{0.0, 0.0, 0.17};
  EXPECT_TRUE(explorer_bridge::shouldIntegratePose(pose, last, 0.25, 0.17));
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
