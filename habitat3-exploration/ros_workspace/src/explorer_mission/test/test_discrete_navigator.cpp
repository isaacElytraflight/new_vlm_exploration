#include <gtest/gtest.h>

#include "explorer_mission/discrete_navigator.hpp"

TEST(DiscreteNavigator, PlanToPoseTurnsThenMoves)
{
  const auto plan = explorer_mission::planToPose(0.0, 0.0, 0.0, 1.0, 0.0, 0.0);
  ASSERT_FALSE(plan.empty());
  bool has_forward = false;
  for (const auto & step : plan) {
    if (step.direction == explorer_mission::DIR_FORWARD && step.steps > 0) {
      has_forward = true;
    }
  }
  EXPECT_TRUE(has_forward);
}

TEST(DiscreteNavigator, ShortestTurnWraps)
{
  const double turn = explorer_mission::shortestTurnDeg(170.0, -170.0);
  EXPECT_NEAR(turn, 20.0, 1e-6);
}

TEST(DiscreteNavigator, StepConstants)
{
  EXPECT_DOUBLE_EQ(explorer_mission::STEP_M, 0.25);
  EXPECT_DOUBLE_EQ(explorer_mission::TURN_DEG, 10.0);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
