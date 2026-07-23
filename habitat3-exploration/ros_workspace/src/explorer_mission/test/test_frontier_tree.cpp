#include <gtest/gtest.h>

#include <random>

#include "explorer_mission/frontier_tree.hpp"

TEST(FrontierTreeHarness, RunnerExecutesAssertions)
{
  EXPECT_EQ(2 + 2, 4);
}

TEST(FrontierTreeHarness, IntentionalFailureIsDetectable_NegativeControl)
{
  const bool negative_control = (1 == 2);
  EXPECT_TRUE(negative_control == false);
}

TEST(FrontierTree, SelectHighestOpennessChild_DefaultPositive)
{
  explorer_mission::FrontierTree tree;
  tree.createRoot(cv::Point2f(0.0f, 0.0f));
  tree.addChild(0, cv::Point2f(1.0f, 0.0f), 3, false);
  tree.addChild(0, cv::Point2f(2.0f, 0.0f), 1, false);
  tree.addChild(0, cv::Point2f(3.0f, 0.0f), 2, false);

  const auto chosen = tree.selectNextChild(0);
  ASSERT_TRUE(chosen.has_value());
  const auto * node = tree.find(*chosen);
  ASSERT_NE(node, nullptr);
  EXPECT_EQ(node->openness_score, 3);
}

TEST(FrontierTree, SelectLowestOpennessChild_WhenPreferHighestFalse_Positive)
{
  explorer_mission::FrontierTree tree;
  tree.createRoot(cv::Point2f(0.0f, 0.0f));
  tree.addChild(0, cv::Point2f(1.0f, 0.0f), 3, false);
  tree.addChild(0, cv::Point2f(2.0f, 0.0f), 1, false);
  tree.addChild(0, cv::Point2f(3.0f, 0.0f), 2, false);

  const auto chosen = tree.selectNextChild(0, nullptr, false);
  ASSERT_TRUE(chosen.has_value());
  const auto * node = tree.find(*chosen);
  ASSERT_NE(node, nullptr);
  EXPECT_EQ(node->openness_score, 1);
}

TEST(FrontierTree, SelectNextChild_SkipsUnrated_Negative)
{
  explorer_mission::FrontierTree tree;
  tree.createRoot(cv::Point2f(0.0f, 0.0f));
  tree.addChild(0, cv::Point2f(1.0f, 0.0f), explorer_mission::kOpennessNotRated, false);
  tree.addChild(0, cv::Point2f(2.0f, 0.0f), explorer_mission::kOpennessNotRated, false);

  EXPECT_FALSE(tree.selectNextChild(0).has_value());
  EXPECT_FALSE(tree.selectNextChild(0, nullptr, false).has_value());
}

TEST(FrontierTree, SelectNextChild_SkipsFullyExplored_Negative)
{
  explorer_mission::FrontierTree tree;
  tree.createRoot(cv::Point2f(0.0f, 0.0f));
  const uint32_t high = tree.addChild(0, cv::Point2f(1.0f, 0.0f), 4, false);
  tree.addChild(0, cv::Point2f(2.0f, 0.0f), 1, false);
  tree.markFullyExplored(high);

  const auto chosen = tree.selectNextChild(0);
  ASSERT_TRUE(chosen.has_value());
  EXPECT_EQ(tree.find(*chosen)->openness_score, 1);
}

TEST(FrontierTree, TieBreakUsesRandomCandidate)
{
  explorer_mission::FrontierTree tree;
  tree.createRoot(cv::Point2f(0.0f, 0.0f));
  const uint32_t a = tree.addChild(0, cv::Point2f(1.0f, 0.0f), 2, false);
  const uint32_t b = tree.addChild(0, cv::Point2f(2.0f, 0.0f), 2, false);

  std::mt19937 rng(42);
  const auto chosen = tree.selectNextChild(0, &rng);
  ASSERT_TRUE(chosen.has_value());
  EXPECT_TRUE(*chosen == a || *chosen == b);
}

TEST(FrontierTree, ScoreZeroMarksFullyExplored)
{
  explorer_mission::FrontierTree tree;
  tree.createRoot(cv::Point2f(0.0f, 0.0f));
  const uint32_t child = tree.addChild(0, cv::Point2f(1.0f, 0.0f), 255, false);
  tree.setOpennessScore(child, 0);
  const auto * node = tree.find(child);
  ASSERT_NE(node, nullptr);
  EXPECT_TRUE(node->fully_explored);
}

TEST(FrontierTree, HasUnexploredNodesExcluding_terminatesInPlace)
{
  explorer_mission::FrontierTree tree;
  tree.createRoot(cv::Point2f(0.0f, 0.0f));
  const uint32_t child = tree.addChild(0, cv::Point2f(1.0f, 0.0f), 1, false);
  tree.markFullyExplored(child);
  tree.markFullyExplored(0);
  EXPECT_FALSE(tree.hasUnexploredNodesExcluding(child, 0));
}

TEST(FrontierTree, HasUnexploredNodesExcluding_siblingRemaining)
{
  explorer_mission::FrontierTree tree;
  tree.createRoot(cv::Point2f(0.0f, 0.0f));
  const uint32_t child_a = tree.addChild(0, cv::Point2f(1.0f, 0.0f), 1, false);
  tree.addChild(0, cv::Point2f(2.0f, 0.0f), 2, false);
  tree.markFullyExplored(child_a);
  EXPECT_TRUE(tree.hasUnexploredNodesExcluding(child_a, 0));
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
