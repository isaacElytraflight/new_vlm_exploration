#include <gtest/gtest.h>

#include "explorer_mission/return_home_guard.hpp"

TEST(ReturnHomeGuardHarness, RunnerExecutesAssertions)
{
  EXPECT_EQ(1 + 1, 2);
}

TEST(ReturnHomeGuardHarness, IntentionalFailureIsDetectable_NegativeControl)
{
  const bool negative_control = (1 == 2);
  EXPECT_FALSE(negative_control);
}

TEST(ReturnHomeGuard, MaySelectFrontierChild_DefaultPositive)
{
  explorer_mission::ReturnHomeGuard guard;
  EXPECT_TRUE(guard.maySelectFrontierChild());
  EXPECT_FALSE(guard.isAwaitingReturn());
}

TEST(ReturnHomeGuard, OnChildNavFailed_BlocksSiblingSelection_Positive)
{
  explorer_mission::ReturnHomeGuard guard;
  guard.onChildNavFailed(3u);
  EXPECT_TRUE(guard.isAwaitingReturn());
  EXPECT_FALSE(guard.maySelectFrontierChild());
  EXPECT_EQ(guard.scanNodeId(), 3u);
}

TEST(ReturnHomeGuard, OnReturnHomeSucceeded_UnblocksSelection_Positive)
{
  explorer_mission::ReturnHomeGuard guard;
  guard.onChildNavFailed(5u);
  guard.onReturnHomeSucceeded();
  EXPECT_FALSE(guard.isAwaitingReturn());
  EXPECT_TRUE(guard.maySelectFrontierChild());
}

TEST(ReturnHomeGuard, OnReturnHomeAbandoned_UnblocksSelection_Positive)
{
  explorer_mission::ReturnHomeGuard guard;
  guard.onChildNavFailed(9u);
  guard.onReturnHomeAbandoned();
  EXPECT_FALSE(guard.isAwaitingReturn());
  EXPECT_TRUE(guard.maySelectFrontierChild());
}

TEST(ReturnHomeGuard, RemainsBlockedUntilReturnSuccess_Negative)
{
  explorer_mission::ReturnHomeGuard guard;
  guard.onChildNavFailed(1u);
  guard.onChildNavFailed(2u);
  EXPECT_EQ(guard.scanNodeId(), 2u);
  EXPECT_FALSE(guard.maySelectFrontierChild());
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
