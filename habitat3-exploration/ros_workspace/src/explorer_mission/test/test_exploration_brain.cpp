#include <memory>
#include <stdexcept>
#include <unordered_map>
#include <vector>

#include <gtest/gtest.h>
#include <opencv2/core.hpp>

#include "explorer_mission/exploration_brain.hpp"

using explorer_mission::BrainAction;
using explorer_mission::BrainContext;
using explorer_mission::ExplorationBrain;
using explorer_mission::ExplorationBrainConfig;
using explorer_mission::FrontierCandidate;
using explorer_mission::createExplorationBrain;

TEST(ExplorationBrainHarness, RunnerExecutesAssertions)
{
  EXPECT_EQ(1 + 1, 2);
}

TEST(ExplorationBrainHarness, IntentionalFailureIsDetectable_NegativeControl)
{
  const bool negative_control = (1 == 2);
  EXPECT_FALSE(negative_control);
}

TEST(ExplorationBrainFactory, CreatesVlmTreeDfs_Positive)
{
  const std::unique_ptr<ExplorationBrain> brain = createExplorationBrain("vlm_tree_dfs");
  ASSERT_NE(brain, nullptr);
  EXPECT_EQ(brain->id(), "vlm_tree_dfs");
  EXPECT_TRUE(brain->wantsVlmScores());
  EXPECT_TRUE(brain->usesFrontierTree());
  EXPECT_NE(brain->frontierTree(), nullptr);
}

TEST(ExplorationBrainFactory, AcceptsVlmDfsAlias_Positive)
{
  const std::unique_ptr<ExplorationBrain> brain = createExplorationBrain("vlm_dfs");
  ASSERT_NE(brain, nullptr);
  EXPECT_EQ(brain->id(), "vlm_tree_dfs");
  EXPECT_TRUE(brain->wantsVlmScores());
  EXPECT_TRUE(brain->usesFrontierTree());
}

TEST(ExplorationBrainFactory, CreatesGreedyNearest_Positive)
{
  const std::unique_ptr<ExplorationBrain> brain = createExplorationBrain("greedy_nearest");
  ASSERT_NE(brain, nullptr);
  EXPECT_EQ(brain->id(), "greedy_nearest");
  EXPECT_FALSE(brain->wantsVlmScores());
  EXPECT_FALSE(brain->usesFrontierTree());
  EXPECT_EQ(brain->frontierTree(), nullptr);
}

TEST(ExplorationBrainFactory, UnknownIdThrows_Negative)
{
  EXPECT_THROW(createExplorationBrain("not_a_brain"), std::invalid_argument);
  EXPECT_THROW(createExplorationBrain(""), std::invalid_argument);
}

TEST(GreedyNearestBrain, SelectsNearestEuclideanFrontier_Positive)
{
  const std::unique_ptr<ExplorationBrain> brain = createExplorationBrain("greedy_nearest");
  brain->onEpisodeStart(cv::Point2f(0.f, 0.f));
  brain->onFrontiersDetected({
    FrontierCandidate{1u, cv::Point2f(5.f, 0.f)},
    FrontierCandidate{2u, cv::Point2f(1.f, 0.f)},
    FrontierCandidate{3u, cv::Point2f(10.f, 0.f)},
  });

  const auto decision = brain->selectNextGoal(BrainContext{cv::Point2f(0.f, 0.f)});
  EXPECT_EQ(decision.action, BrainAction::kNavigateTo);
  EXPECT_EQ(decision.goal_id, 2u);
  EXPECT_NEAR(decision.goal.x, 1.f, 1e-4f);
  EXPECT_NEAR(decision.goal.y, 0.f, 1e-4f);
}

TEST(GreedyNearestBrain, CompletesWhenNoLiveFrontiers_Negative)
{
  const std::unique_ptr<ExplorationBrain> brain = createExplorationBrain("greedy_nearest");
  brain->onEpisodeStart(cv::Point2f(0.f, 0.f));
  brain->onFrontiersDetected({});

  const auto decision = brain->selectNextGoal(BrainContext{cv::Point2f(0.f, 0.f)});
  EXPECT_EQ(decision.action, BrainAction::kComplete);
}

TEST(GreedyNearestBrain, SkipsVisitedAfterArrival_Positive)
{
  const std::unique_ptr<ExplorationBrain> brain = createExplorationBrain("greedy_nearest");
  brain->onEpisodeStart(cv::Point2f(0.f, 0.f));
  brain->onFrontiersDetected({
    FrontierCandidate{1u, cv::Point2f(1.f, 0.f)},
    FrontierCandidate{2u, cv::Point2f(2.f, 0.f)},
  });

  const auto first = brain->selectNextGoal(BrainContext{cv::Point2f(0.f, 0.f)});
  ASSERT_EQ(first.action, BrainAction::kNavigateTo);
  EXPECT_EQ(first.goal_id, 1u);

  brain->onArrived(first.goal_id, first.goal);
  brain->onFrontiersDetected({
    FrontierCandidate{1u, cv::Point2f(1.f, 0.f)},
    FrontierCandidate{2u, cv::Point2f(2.f, 0.f)},
  });

  const auto second = brain->selectNextGoal(BrainContext{cv::Point2f(1.f, 0.f)});
  EXPECT_EQ(second.action, BrainAction::kNavigateTo);
  EXPECT_EQ(second.goal_id, 2u);
}

TEST(GreedyNearestBrain, NeverWantsVlmOrTree_Negative)
{
  const std::unique_ptr<ExplorationBrain> brain = createExplorationBrain("greedy_nearest");
  EXPECT_FALSE(brain->wantsVlmScores());
  EXPECT_FALSE(brain->usesFrontierTree());
}

TEST(VlmTreeDfsBrain, WaitsForVlmBeforeSelecting_Positive)
{
  const std::unique_ptr<ExplorationBrain> brain = createExplorationBrain("vlm_tree_dfs");
  brain->onEpisodeStart(cv::Point2f(0.f, 0.f));
  const auto ids = brain->onFrontiersDetected({
    FrontierCandidate{0u, cv::Point2f(1.f, 0.f)},
    FrontierCandidate{0u, cv::Point2f(2.f, 0.f)},
  });
  ASSERT_EQ(ids.size(), 2u);

  const auto before = brain->selectNextGoal(BrainContext{cv::Point2f(0.f, 0.f)});
  EXPECT_EQ(before.action, BrainAction::kWait);
}

TEST(VlmTreeDfsBrain, SelectsHighestOpennessChild_DefaultPositive)
{
  const std::unique_ptr<ExplorationBrain> brain = createExplorationBrain("vlm_tree_dfs");
  brain->onEpisodeStart(cv::Point2f(0.f, 0.f));
  const auto ids = brain->onFrontiersDetected({
    FrontierCandidate{0u, cv::Point2f(1.f, 0.f)},
    FrontierCandidate{0u, cv::Point2f(2.f, 0.f)},
    FrontierCandidate{0u, cv::Point2f(3.f, 0.f)},
  });
  ASSERT_EQ(ids.size(), 3u);

  brain->onVlmScores({
    {ids[0], 3},
    {ids[1], 1},
    {ids[2], 5},
  });

  const auto decision = brain->selectNextGoal(BrainContext{cv::Point2f(0.f, 0.f)});
  ASSERT_EQ(decision.action, BrainAction::kNavigateTo);
  EXPECT_EQ(decision.goal_id, ids[2]);
  EXPECT_NEAR(decision.goal.x, 3.f, 1e-4f);
}

TEST(VlmTreeDfsBrain, SelectsLowestOpennessWhenPreferHighestFalse_Positive)
{
  ExplorationBrainConfig cfg;
  cfg.dfs_prefer_highest_openness = false;
  cfg.parent_to_nearest_node = false;
  const std::unique_ptr<ExplorationBrain> brain = createExplorationBrain("vlm_tree_dfs", cfg);
  brain->onEpisodeStart(cv::Point2f(0.f, 0.f));
  const auto ids = brain->onFrontiersDetected({
    FrontierCandidate{0u, cv::Point2f(1.f, 0.f)},
    FrontierCandidate{0u, cv::Point2f(2.f, 0.f)},
  });
  brain->onVlmScores({{ids[0], 4}, {ids[1], 2}});

  const auto decision = brain->selectNextGoal(BrainContext{cv::Point2f(0.f, 0.f)});
  ASSERT_EQ(decision.action, BrainAction::kNavigateTo);
  EXPECT_EQ(decision.goal_id, ids[1]);
}

TEST(VlmTreeDfsBrain, SoftFailUnratedAllowsSelection_Positive)
{
  const std::unique_ptr<ExplorationBrain> brain = createExplorationBrain("vlm_tree_dfs");
  brain->onEpisodeStart(cv::Point2f(0.f, 0.f));
  const auto ids = brain->onFrontiersDetected({
    FrontierCandidate{0u, cv::Point2f(1.f, 0.f)},
  });
  EXPECT_EQ(brain->selectNextGoal(BrainContext{}).action, BrainAction::kWait);

  brain->softFailUnrated(1);
  const auto decision = brain->selectNextGoal(BrainContext{cv::Point2f(0.f, 0.f)});
  ASSERT_EQ(decision.action, BrainAction::kNavigateTo);
  EXPECT_EQ(decision.goal_id, ids[0]);
}

TEST(VlmTreeDfsBrain, BacktracksWhenChildrenExhausted_Positive)
{
  const std::unique_ptr<ExplorationBrain> brain = createExplorationBrain("vlm_tree_dfs");
  brain->onEpisodeStart(cv::Point2f(0.f, 0.f));
  const auto ids = brain->onFrontiersDetected({
    FrontierCandidate{0u, cv::Point2f(1.f, 0.f)},
    FrontierCandidate{0u, cv::Point2f(2.f, 0.f)},
  });
  brain->onVlmScores({{ids[0], 3}, {ids[1], 2}});

  const auto to_child = brain->selectNextGoal(BrainContext{});
  ASSERT_EQ(to_child.action, BrainAction::kNavigateTo);
  EXPECT_EQ(to_child.goal_id, ids[0]);
  brain->onArrived(to_child.goal_id, to_child.goal);

  // No new frontiers at child → exhaust child → backtrack to root.
  brain->onFrontiersDetected({});
  const auto back = brain->selectNextGoal(BrainContext{cv::Point2f(1.f, 0.f)});
  ASSERT_EQ(back.action, BrainAction::kNavigateTo);
  EXPECT_EQ(back.goal_id, 0u);
  EXPECT_NEAR(back.goal.x, 0.f, 1e-4f);
}

TEST(VlmTreeDfsBrain, CompletesWhenTreeExhausted_Negative)
{
  const std::unique_ptr<ExplorationBrain> brain = createExplorationBrain("vlm_tree_dfs");
  brain->onEpisodeStart(cv::Point2f(0.f, 0.f));
  brain->onFrontiersDetected({});
  const auto decision = brain->selectNextGoal(BrainContext{});
  EXPECT_EQ(decision.action, BrainAction::kComplete);
}

TEST(VlmTreeDfsBrain, ParentToNearestAttachesToCloserNode_Positive)
{
  ExplorationBrainConfig cfg;
  cfg.parent_to_nearest_node = true;
  cfg.dfs_prefer_highest_openness = true;
  const std::unique_ptr<ExplorationBrain> brain = createExplorationBrain("vlm_tree_dfs", cfg);
  brain->onEpisodeStart(cv::Point2f(0.f, 0.f));

  // Seed a remote node under root, then "visit" it so current is remote.
  const auto seed = brain->onFrontiersDetected({
    FrontierCandidate{0u, cv::Point2f(5.f, 0.f)},
  });
  brain->onVlmScores({{seed[0], 2}});
  brain->onArrived(seed[0], cv::Point2f(5.f, 0.f));

  // New frontier near root while current is remote → attaches under root (nearest).
  const auto batch = brain->onFrontiersDetected({
    FrontierCandidate{0u, cv::Point2f(0.2f, 0.f)},
  });
  ASSERT_EQ(batch.size(), 1u);
  const auto * tree = brain->frontierTree();
  ASSERT_NE(tree, nullptr);
  const auto * child = tree->find(batch[0]);
  ASSERT_NE(child, nullptr);
  EXPECT_EQ(child->parent_id, 0);

  brain->onVlmScores({{batch[0], 4}});
  // Current remote has no unexplored children of its own → batch pick under other parent.
  const auto decision = brain->selectNextGoal(BrainContext{cv::Point2f(5.f, 0.f)});
  ASSERT_EQ(decision.action, BrainAction::kNavigateTo);
  EXPECT_EQ(decision.goal_id, batch[0]);
}

TEST(VlmTreeDfsBrain, ParentToNearestOffKeepsCurrentParent_Negative)
{
  ExplorationBrainConfig cfg;
  cfg.parent_to_nearest_node = false;
  const std::unique_ptr<ExplorationBrain> brain = createExplorationBrain("vlm_tree_dfs", cfg);
  brain->onEpisodeStart(cv::Point2f(0.f, 0.f));

  const auto seed = brain->onFrontiersDetected({
    FrontierCandidate{0u, cv::Point2f(5.f, 0.f)},
  });
  brain->onVlmScores({{seed[0], 2}});
  brain->onArrived(seed[0], cv::Point2f(5.f, 0.f));

  const auto batch = brain->onFrontiersDetected({
    FrontierCandidate{0u, cv::Point2f(0.2f, 0.f)},
  });
  ASSERT_EQ(batch.size(), 1u);
  const auto * child = brain->frontierTree()->find(batch[0]);
  ASSERT_NE(child, nullptr);
  EXPECT_EQ(static_cast<uint32_t>(child->parent_id), seed[0]);
}

TEST(VlmTreeDfsBrain, NavFailedMarksDead_Positive)
{
  const std::unique_ptr<ExplorationBrain> brain = createExplorationBrain("vlm_tree_dfs");
  brain->onEpisodeStart(cv::Point2f(0.f, 0.f));
  const auto ids = brain->onFrontiersDetected({
    FrontierCandidate{0u, cv::Point2f(1.f, 0.f)},
    FrontierCandidate{0u, cv::Point2f(2.f, 0.f)},
  });
  brain->onVlmScores({{ids[0], 5}, {ids[1], 3}});

  const auto first = brain->selectNextGoal(BrainContext{});
  ASSERT_EQ(first.goal_id, ids[0]);
  brain->onNavFailed(ids[0], true);

  const auto second = brain->selectNextGoal(BrainContext{});
  ASSERT_EQ(second.action, BrainAction::kNavigateTo);
  EXPECT_EQ(second.goal_id, ids[1]);
}

TEST(GreedyNearestBrain, VisitedAndLiveSnapshots_Positive)
{
  const std::unique_ptr<ExplorationBrain> brain = createExplorationBrain("greedy_nearest");
  brain->onEpisodeStart(cv::Point2f(0.f, 0.f));
  brain->onFrontiersDetected({
    FrontierCandidate{1u, cv::Point2f(1.f, 0.f)},
    FrontierCandidate{2u, cv::Point2f(2.f, 0.f)},
  });
  EXPECT_TRUE(brain->visitedIds().empty());
  EXPECT_EQ(brain->liveFrontierIds().size(), 2u);

  brain->onArrived(1u, cv::Point2f(1.f, 0.f));
  const auto visited = brain->visitedIds();
  ASSERT_EQ(visited.size(), 1u);
  EXPECT_EQ(visited[0], 1u);
}

TEST(VlmFrontierGraphBrain, BuildsKnnEdgesAndPicksHighest_Positive)
{
  const std::unique_ptr<ExplorationBrain> brain = createExplorationBrain("vlm_frontier_graph");
  brain->onEpisodeStart(cv::Point2f(0.f, 0.f));
  const auto ids = brain->onFrontiersDetected({
    FrontierCandidate{1u, cv::Point2f(1.f, 0.f)},
    FrontierCandidate{2u, cv::Point2f(2.f, 0.f)},
    FrontierCandidate{3u, cv::Point2f(3.f, 0.f)},
  });
  ASSERT_EQ(ids.size(), 3u);
  const auto edges = brain->takePendingGraphEdges();
  EXPECT_FALSE(edges.empty());
  EXPECT_EQ(brain->selectNextGoal(BrainContext{}).action, BrainAction::kWait);

  brain->onVlmScores({{1u, 2}, {2u, 5}, {3u, 1}});
  const auto decision = brain->selectNextGoal(BrainContext{cv::Point2f(0.f, 0.f)});
  ASSERT_EQ(decision.action, BrainAction::kNavigateTo);
  EXPECT_EQ(decision.goal_id, 2u);
}

TEST(VlmFrontierGraphBrain, CompletesWhenNoLive_Negative)
{
  const std::unique_ptr<ExplorationBrain> brain = createExplorationBrain("vlm_frontier_graph");
  brain->onEpisodeStart(cv::Point2f(0.f, 0.f));
  brain->onFrontiersDetected({});
  EXPECT_EQ(brain->selectNextGoal(BrainContext{}).action, BrainAction::kComplete);
}

TEST(VlmChoiceDijkstraBrain, EmitsChoiceRecord_Positive)
{
  const std::unique_ptr<ExplorationBrain> brain = createExplorationBrain("vlm_choice_dijkstra");
  brain->onEpisodeStart(cv::Point2f(0.f, 0.f));
  brain->onFrontiersDetected({
    FrontierCandidate{10u, cv::Point2f(1.f, 0.f)},
    FrontierCandidate{20u, cv::Point2f(2.f, 0.f)},
  });
  brain->takePendingGraphEdges();  // drain
  brain->onVlmScores({{10u, 3}, {20u, 4}});
  const auto decision = brain->selectNextGoal(BrainContext{cv::Point2f(0.f, 0.f)});
  ASSERT_EQ(decision.action, BrainAction::kNavigateTo);
  const auto choice = brain->takePendingVlmChoice();
  ASSERT_TRUE(choice.has_value());
  EXPECT_EQ(choice->selected_frontier_id, decision.goal_id);
  EXPECT_FALSE(choice->prompt.empty());
  EXPECT_FALSE(choice->response.empty());
  EXPECT_FALSE(choice->candidate_ids.empty());
}

TEST(VlmChoiceDijkstraBrain, NoChoiceWithoutScores_Negative)
{
  const std::unique_ptr<ExplorationBrain> brain = createExplorationBrain("vlm_choice_dijkstra");
  brain->onEpisodeStart(cv::Point2f(0.f, 0.f));
  brain->onFrontiersDetected({FrontierCandidate{1u, cv::Point2f(1.f, 0.f)}});
  EXPECT_EQ(brain->selectNextGoal(BrainContext{}).action, BrainAction::kWait);
  EXPECT_FALSE(brain->takePendingVlmChoice().has_value());
}

TEST(VlmTreeDfsBrain, SetConfigUpdatesPreferHighest_Positive)
{
  ExplorationBrainConfig cfg;
  cfg.dfs_prefer_highest_openness = true;
  auto brain = createExplorationBrain("vlm_tree_dfs", cfg);
  brain->onEpisodeStart(cv::Point2f(0.f, 0.f));
  const auto ids = brain->onFrontiersDetected({
    FrontierCandidate{0u, cv::Point2f(1.f, 0.f)},
    FrontierCandidate{0u, cv::Point2f(2.f, 0.f)},
  });
  brain->onVlmScores({{ids[0], 4}, {ids[1], 2}});
  EXPECT_EQ(brain->selectNextGoal(BrainContext{}).goal_id, ids[0]);

  cfg.dfs_prefer_highest_openness = false;
  brain->setConfig(cfg);
  EXPECT_EQ(brain->selectNextGoal(BrainContext{}).goal_id, ids[1]);
}

TEST(ShouldPerformFrontierScan, UnvisitedRequestsScan_Positive)
{
  EXPECT_TRUE(explorer_mission::shouldPerformFrontierScan(/*frontier_is_visited=*/false));
}

TEST(ShouldPerformFrontierScan, VisitedAliveSkipsScan_Negative)
{
  EXPECT_FALSE(explorer_mission::shouldPerformFrontierScan(/*frontier_is_visited=*/true));
}

TEST(VlmTreeDfsBrain, ArrivalMarksVisitedNotDead_Positive)
{
  auto brain = createExplorationBrain("vlm_tree_dfs");
  brain->onEpisodeStart(cv::Point2f(0.f, 0.f));
  const auto ids = brain->onFrontiersDetected({
    FrontierCandidate{0u, cv::Point2f(1.f, 0.f)},
  });
  brain->onVlmScores({{ids[0], 4}});
  EXPECT_FALSE(brain->isVisited(ids[0]));
  EXPECT_FALSE(brain->isDead(ids[0]));
  brain->onArrived(ids[0], cv::Point2f(1.f, 0.f));
  EXPECT_TRUE(brain->isVisited(ids[0]));
  EXPECT_FALSE(brain->isDead(ids[0]));
}

TEST(VlmTreeDfsBrain, NavFailMarksDeadWithoutVisit_Negative)
{
  auto brain = createExplorationBrain("vlm_tree_dfs");
  brain->onEpisodeStart(cv::Point2f(0.f, 0.f));
  const auto ids = brain->onFrontiersDetected({
    FrontierCandidate{0u, cv::Point2f(1.f, 0.f)},
  });
  brain->onVlmScores({{ids[0], 4}});
  brain->onNavFailed(ids[0], true);
  EXPECT_FALSE(brain->isVisited(ids[0]));
  EXPECT_TRUE(brain->isDead(ids[0]));
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
