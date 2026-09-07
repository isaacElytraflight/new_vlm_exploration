#include <gtest/gtest.h>

#include "explorer_mission/stuck_progress.hpp"

TEST(StuckProgressHarness, RunnerExecutesAssertions)
{
  EXPECT_EQ(1 + 1, 2);
}

TEST(StuckProgressHarness, IntentionalFailureIsDetectable_NegativeControl)
{
  const bool negative_control = (1 == 2);
  EXPECT_FALSE(negative_control);
}

TEST(StuckProgress, DefaultsMatchPolicy_Positive)
{
  EXPECT_DOUBLE_EQ(explorer_mission::kNavTotalTimeoutS, 300.0);
  EXPECT_DOUBLE_EQ(explorer_mission::kNavStuckTimeoutS, 60.0);
  EXPECT_DOUBLE_EQ(explorer_mission::kNavStuckDistanceM, 1.0);
}

TEST(StuckProgress, MovementOfOneMeterResetsStuckClock_Positive)
{
  explorer_mission::StuckProgressTracker tracker;
  tracker.noteProgress(0.0, 0.0, explorer_mission::kNavStuckDistanceM, 0.0);
  tracker.noteProgress(1.0, 0.0, explorer_mission::kNavStuckDistanceM, 30.0);
  EXPECT_FALSE(tracker.isStuck(explorer_mission::kNavStuckTimeoutS, 89.0));
}

TEST(StuckProgress, SubMeterMotionDoesNotResetStuckClock_Negative)
{
  explorer_mission::StuckProgressTracker tracker;
  tracker.noteProgress(0.0, 0.0, explorer_mission::kNavStuckDistanceM, 0.0);
  // 0.9 m < 1.0 m policy threshold — must still count as stuck after timeout.
  tracker.noteProgress(0.9, 0.0, explorer_mission::kNavStuckDistanceM, 30.0);
  EXPECT_TRUE(tracker.isStuck(explorer_mission::kNavStuckTimeoutS, 60.1));
}

TEST(StuckProgress, StuckAfterTimeoutWithNoProgress_Positive)
{
  explorer_mission::StuckProgressTracker tracker;
  tracker.noteProgress(0.0, 0.0, explorer_mission::kNavStuckDistanceM, 0.0);
  EXPECT_FALSE(tracker.isStuck(explorer_mission::kNavStuckTimeoutS, 60.0));
  EXPECT_TRUE(tracker.isStuck(explorer_mission::kNavStuckTimeoutS, 60.1));
}

TEST(StuckProgress, NoAnchorNotStuck_Negative)
{
  explorer_mission::StuckProgressTracker tracker;
  EXPECT_FALSE(tracker.isStuck(explorer_mission::kNavStuckTimeoutS, 1000.0));
}

TEST(StuckProgress, ResetClearsAnchor_Negative)
{
  explorer_mission::StuckProgressTracker tracker;
  tracker.noteProgress(0.0, 0.0, explorer_mission::kNavStuckDistanceM, 0.0);
  tracker.reset();
  EXPECT_FALSE(tracker.haveAnchor());
  EXPECT_FALSE(tracker.isStuck(explorer_mission::kNavStuckTimeoutS, 100.0));
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
