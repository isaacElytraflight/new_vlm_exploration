#pragma once

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include <opencv2/core.hpp>

#include "explorer_mission/frontier_tree.hpp"

namespace explorer_mission
{

struct FrontierCandidate
{
  /// For greedy/graph: stable id from the caller. For tree DFS: ignored (tree assigns ids).
  uint32_t id{0};
  cv::Point2f position;
};

enum class BrainAction
{
  kNavigateTo,
  kComplete,
  kWait,
};

struct BrainDecision
{
  BrainAction action{BrainAction::kComplete};
  uint32_t goal_id{0};
  cv::Point2f goal;
  std::string detail;
  /// If true, adopt goal_id without physical NavigateToPose (DFS parent hop).
  bool theoretical{false};
};

struct BrainContext
{
  cv::Point2f robot_pose;
};

struct ExplorationBrainConfig
{
  bool dfs_prefer_highest_openness{true};
  bool parent_to_nearest_node{true};
  /// kNN degree for graph brains (clamped to ~3–5 in builders).
  int graph_knn{5};
  /// Reject rediscovered frontiers within this radius of a dead pose.
  /// Survives id churn across detects (all brains).
  float dead_pose_radius_m{1.0f};
};

struct GraphEdge
{
  uint32_t from_id{0};
  uint32_t to_id{0};
  float cost{0.f};
};

struct VlmChoiceRecord
{
  std::string prompt;
  std::string response;
  uint32_t selected_frontier_id{0};
  std::vector<uint32_t> candidate_ids;
};

/// Map-overlay snapshot for non-tree brains (and synthetic tree publish).
struct BrainVizNode
{
  uint32_t id{0};
  cv::Point2f position;
  bool visited{false};
  /// Permanently abandoned (dead). Distinct from visited.
  bool dead{false};
  uint8_t openness_score{255};
};

/// Pluggable high-level exploration policy (Goal C).
/// Orchestration (detection geometry, Nav2, thrash) stays in explore_node.
class ExplorationBrain
{
public:
  virtual ~ExplorationBrain() = default;

  virtual std::string id() const = 0;
  /// True if the brain blocks on numeric VLM openness scores.
  virtual bool wantsVlmScores() const = 0;
  /// True if the brain maintains a frontier tree (vs flat / graph memory).
  virtual bool usesFrontierTree() const = 0;

  /// Tree memory when usesFrontierTree(); otherwise nullptr.
  virtual const FrontierTree * frontierTree() const {return nullptr;}
  virtual FrontierTree * frontierTree() {return nullptr;}

  /// Flat frontier markers when frontierTree() is null (greedy / graph).
  virtual std::vector<BrainVizNode> vizNodes() const {return {};}
  virtual uint32_t vizCurrentNodeId() const {return 0;}

  /// Snapshot helpers for event JSONL (visited / live frontier ids).
  virtual std::vector<uint32_t> visitedIds() const {return {};}
  virtual std::vector<uint32_t> liveFrontierIds() const {return {};}
  /// Visited = physically arrived (one-time 360° scan). Dead = abandoned permanently.
  virtual bool isVisited(uint32_t /*id*/) const {return false;}
  virtual bool isDead(uint32_t /*id*/) const {return false;}

  /// Drain pending graph edges for `exploration/brain/graph_edges` logging.
  virtual std::vector<GraphEdge> takePendingGraphEdges() {return {};}
  /// Drain pending VLM choice record for `exploration/vlm/choice` logging.
  virtual std::optional<VlmChoiceRecord> takePendingVlmChoice() {return std::nullopt;}

  /// Update DFS / parent knobs without resetting memory (ablation param sets).
  virtual void setConfig(const ExplorationBrainConfig & /*config*/) {}

  virtual void onEpisodeStart(const cv::Point2f & start_pose) = 0;
  /// Attach / refresh frontiers. Returns ids that may need VLM rating (tree: new children).
  virtual std::vector<uint32_t> onFrontiersDetected(
    const std::vector<FrontierCandidate> & frontiers) = 0;
  virtual void onVlmScores(const std::unordered_map<uint32_t, uint8_t> & scores) = 0;
  /// Soft-fail still-unrated frontiers (VLM timeout). Default score 1 (not 0).
  virtual void softFailUnrated(uint8_t score = 1) = 0;
  virtual void onArrived(uint32_t goal_id, const cv::Point2f & pose) = 0;
  virtual void onNavFailed(uint32_t goal_id, bool mark_dead) = 0;

  virtual BrainDecision selectNextGoal(const BrainContext & ctx) = 0;
};

/// Factory: `vlm_tree_dfs` (+ alias `vlm_dfs`), `greedy_nearest`,
/// `vlm_frontier_graph`, `vlm_choice_dijkstra`.
std::unique_ptr<ExplorationBrain> createExplorationBrain(
  const std::string & brain_id,
  const ExplorationBrainConfig & config = {});

/// 360° observation scan only at unvisited frontiers (visited-but-alive skips).
inline bool shouldPerformFrontierScan(bool frontier_is_visited)
{
  return !frontier_is_visited;
}

}  // namespace explorer_mission
