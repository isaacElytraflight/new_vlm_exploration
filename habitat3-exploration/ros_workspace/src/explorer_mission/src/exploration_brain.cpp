#include "explorer_mission/exploration_brain.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <sstream>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>

namespace explorer_mission
{
namespace
{

constexpr uint8_t kOpennessUnset = 255;

int clampKnn(int knn)
{
  if (knn < 3) {
    return 3;
  }
  if (knn > 5) {
    return 5;
  }
  return knn;
}

float euclidean(const cv::Point2f & a, const cv::Point2f & b)
{
  const float dx = a.x - b.x;
  const float dy = a.y - b.y;
  return std::sqrt(dx * dx + dy * dy);
}

/// Flat frontier graph with Euclidean kNN edges (Nav2-cost proxy for v1).
class FrontierGraphMemory
{
public:
  void clear()
  {
    nodes_.clear();
    edges_.clear();
    scores_.clear();
    visited_.clear();
    dead_.clear();
    current_id_ = 0;
    have_current_ = false;
    next_id_ = 1;
  }

  void setCurrent(uint32_t id)
  {
    current_id_ = id;
    have_current_ = true;
  }

  bool haveCurrent() const {return have_current_;}
  uint32_t currentId() const {return current_id_;}

  std::vector<uint32_t> addFrontiers(
    const std::vector<FrontierCandidate> & frontiers,
    int knn)
  {
    std::vector<uint32_t> new_ids;
    for (const auto & f : frontiers) {
      const uint32_t id = (f.id != 0) ? f.id : next_id_++;
      if (f.id != 0) {
        next_id_ = std::max(next_id_, f.id + 1);
      }
      if (visited_.count(id) || dead_.count(id)) {
        continue;
      }
      nodes_[id] = f.position;
      if (scores_.find(id) == scores_.end()) {
        scores_[id] = kOpennessUnset;
      }
      new_ids.push_back(id);
    }
    rebuildEdges(clampKnn(knn));
    return new_ids;
  }

  void setScore(uint32_t id, uint8_t score)
  {
    scores_[id] = score;
    if (score == 0) {
      dead_.insert(id);
    }
  }

  void softFailUnrated(const std::vector<uint32_t> & ids, uint8_t score)
  {
    for (uint32_t id : ids) {
      auto it = scores_.find(id);
      if (it != scores_.end() && it->second == kOpennessUnset) {
        it->second = score;
      }
    }
  }

  void markVisited(uint32_t id)
  {
    visited_.insert(id);
  }

  void markDead(uint32_t id)
  {
    dead_.insert(id);
  }

  bool isLive(uint32_t id) const
  {
    return nodes_.count(id) && !visited_.count(id) && !dead_.count(id);
  }

  bool isVisited(uint32_t id) const
  {
    return visited_.count(id) != 0;
  }

  bool isDead(uint32_t id) const
  {
    return dead_.count(id) != 0;
  }

  std::optional<cv::Point2f> position(uint32_t id) const
  {
    const auto it = nodes_.find(id);
    if (it == nodes_.end()) {
      return std::nullopt;
    }
    return it->second;
  }

  uint8_t score(uint32_t id) const
  {
    const auto it = scores_.find(id);
    return it == scores_.end() ? kOpennessUnset : it->second;
  }

  std::vector<uint32_t> liveIds() const
  {
    std::vector<uint32_t> out;
    for (const auto & entry : nodes_) {
      if (isLive(entry.first)) {
        out.push_back(entry.first);
      }
    }
    return out;
  }

  std::vector<uint32_t> visitedIds() const
  {
    return std::vector<uint32_t>(visited_.begin(), visited_.end());
  }

  std::vector<uint32_t> neighbors(uint32_t id) const
  {
    std::vector<uint32_t> out;
    for (const auto & e : edges_) {
      if (e.from_id == id && isLive(e.to_id)) {
        out.push_back(e.to_id);
      } else if (e.to_id == id && isLive(e.from_id)) {
        out.push_back(e.from_id);
      }
    }
    return out;
  }

  const std::vector<GraphEdge> & edges() const {return edges_;}

  /// Dijkstra distances from source over undirected edge costs.
  std::unordered_map<uint32_t, float> dijkstra(uint32_t source) const
  {
    std::unordered_map<uint32_t, float> dist;
    for (const auto & n : nodes_) {
      dist[n.first] = std::numeric_limits<float>::infinity();
    }
    if (!nodes_.count(source)) {
      return dist;
    }
    dist[source] = 0.f;
    using NodeCost = std::pair<float, uint32_t>;
    std::priority_queue<NodeCost, std::vector<NodeCost>, std::greater<NodeCost>> pq;
    pq.push({0.f, source});
    while (!pq.empty()) {
      const auto [d, u] = pq.top();
      pq.pop();
      if (d > dist[u]) {
        continue;
      }
      for (uint32_t v : neighborsIncludingDead(u)) {
        const float w = edgeCost(u, v);
        if (!std::isfinite(w)) {
          continue;
        }
        const float nd = d + w;
        if (nd < dist[v]) {
          dist[v] = nd;
          pq.push({nd, v});
        }
      }
    }
    return dist;
  }

private:
  void rebuildEdges(int knn)
  {
    edges_.clear();
    std::vector<uint32_t> ids;
    ids.reserve(nodes_.size());
    for (const auto & n : nodes_) {
      ids.push_back(n.first);
    }
    for (uint32_t u : ids) {
      std::vector<std::pair<float, uint32_t>> nbrs;
      for (uint32_t v : ids) {
        if (u == v) {
          continue;
        }
        nbrs.push_back({euclidean(nodes_.at(u), nodes_.at(v)), v});
      }
      std::sort(nbrs.begin(), nbrs.end());
      const size_t take = std::min(static_cast<size_t>(knn), nbrs.size());
      for (size_t i = 0; i < take; ++i) {
        const uint32_t v = nbrs[i].second;
        const float cost = nbrs[i].first;
        // Undirected: store once with from < to to avoid dup spam in logs.
        GraphEdge e;
        e.from_id = std::min(u, v);
        e.to_id = std::max(u, v);
        e.cost = cost;
        bool exists = false;
        for (const auto & old : edges_) {
          if (old.from_id == e.from_id && old.to_id == e.to_id) {
            exists = true;
            break;
          }
        }
        if (!exists) {
          edges_.push_back(e);
        }
      }
    }
  }

  std::vector<uint32_t> neighborsIncludingDead(uint32_t id) const
  {
    std::vector<uint32_t> out;
    for (const auto & e : edges_) {
      if (e.from_id == id) {
        out.push_back(e.to_id);
      } else if (e.to_id == id) {
        out.push_back(e.from_id);
      }
    }
    return out;
  }

  float edgeCost(uint32_t a, uint32_t b) const
  {
    const uint32_t lo = std::min(a, b);
    const uint32_t hi = std::max(a, b);
    for (const auto & e : edges_) {
      if (e.from_id == lo && e.to_id == hi) {
        return e.cost;
      }
    }
    return std::numeric_limits<float>::infinity();
  }

  std::unordered_map<uint32_t, cv::Point2f> nodes_;
  std::vector<GraphEdge> edges_;
  std::unordered_map<uint32_t, uint8_t> scores_;
  std::unordered_set<uint32_t> visited_;
  std::unordered_set<uint32_t> dead_;
  uint32_t current_id_{0};
  bool have_current_{false};
  uint32_t next_id_{1};
};

class VlmTreeDfsBrain final : public ExplorationBrain
{
public:
  explicit VlmTreeDfsBrain(const ExplorationBrainConfig & config)
  : config_(config) {}

  std::string id() const override {return "vlm_tree_dfs";}
  bool wantsVlmScores() const override {return true;}
  bool usesFrontierTree() const override {return true;}

  const FrontierTree * frontierTree() const override {return &tree_;}
  FrontierTree * frontierTree() override {return &tree_;}

  void setConfig(const ExplorationBrainConfig & config) override {config_ = config;}

  std::vector<uint32_t> visitedIds() const override
  {
    std::vector<uint32_t> out;
    for (uint32_t id : tree_.allNodeIds()) {
      if (tree_.isVisited(id)) {
        out.push_back(id);
      }
    }
    return out;
  }

  std::vector<uint32_t> liveFrontierIds() const override
  {
    std::vector<uint32_t> out;
    for (uint32_t id : tree_.allNodeIds()) {
      const TreeNode * n = tree_.find(id);
      if (!n || n->fully_explored || n->parent_id < 0) {
        continue;
      }
      out.push_back(id);
    }
    return out;
  }

  bool isVisited(uint32_t id) const override {return tree_.isVisited(id);}
  bool isDead(uint32_t id) const override {return tree_.isDead(id);}

  void onEpisodeStart(const cv::Point2f & start_pose) override
  {
    tree_.createRoot(start_pose);
    tree_.setCurrentNodeId(tree_.rootId());
    last_new_child_ids_.clear();
  }

  std::vector<uint32_t> onFrontiersDetected(
    const std::vector<FrontierCandidate> & frontiers) override
  {
    last_new_child_ids_.clear();
    const std::vector<uint32_t> parent_pool = tree_.allNodeIds();
    for (const auto & f : frontiers) {
      const uint32_t parent_id = tree_.resolveFrontierParentId(
        config_.parent_to_nearest_node, tree_.currentNodeId(), f.position, &parent_pool);
      const uint32_t child_id = tree_.addChild(
        parent_id, f.position, kOpennessNotRated, false);
      if (child_id != std::numeric_limits<uint32_t>::max()) {
        last_new_child_ids_.push_back(child_id);
      }
    }
    return last_new_child_ids_;
  }

  void onVlmScores(const std::unordered_map<uint32_t, uint8_t> & scores) override
  {
    for (const auto & entry : scores) {
      tree_.setOpennessScore(entry.first, entry.second);
    }
  }

  void softFailUnrated(uint8_t score) override
  {
    for (uint32_t id : last_new_child_ids_) {
      const TreeNode * node = tree_.find(id);
      if (node && node->openness_score == kOpennessNotRated) {
        tree_.setOpennessScore(id, score);
      }
    }
  }

  void onArrived(uint32_t goal_id, const cv::Point2f & /*pose*/) override
  {
    tree_.setCurrentNodeId(goal_id);
    tree_.markVisited(goal_id);
  }

  void onNavFailed(uint32_t goal_id, bool mark_dead) override
  {
    if (mark_dead) {
      tree_.markFullyExplored(goal_id);
    }
  }

  BrainDecision selectNextGoal(const BrainContext & /*ctx*/) override
  {
    BrainDecision d;
    TreeNode * current = tree_.find(tree_.currentNodeId());
    if (!current) {
      d.action = BrainAction::kComplete;
      d.detail = "current tree node missing";
      return d;
    }

    if (!tree_.hasUnexploredChildren(current->id)) {
      if (config_.parent_to_nearest_node) {
        const auto batch_pick = tree_.selectBestAmong(
          last_new_child_ids_, nullptr, config_.dfs_prefer_highest_openness);
        if (batch_pick.has_value()) {
          TreeNode * batch_child = tree_.find(*batch_pick);
          if (batch_child && !batch_child->fully_explored &&
            batch_child->openness_score != kOpennessNotRated)
          {
            last_new_child_ids_.clear();
            d.action = BrainAction::kNavigateTo;
            d.goal_id = *batch_pick;
            d.goal = batch_child->position;
            d.detail = "batch/nearest child score=" +
              std::to_string(batch_child->openness_score);
            return d;
          }
        }
      }

      tree_.markFullyExplored(current->id);
      if (!tree_.hasUnexploredNodesExcluding(current->id, tree_.rootId())) {
        d.action = BrainAction::kComplete;
        d.detail = "exploration complete (in place)";
        return d;
      }
      if (current->parent_id < 0) {
        d.action = BrainAction::kComplete;
        d.detail = "root exhausted";
        return d;
      }
      const uint32_t parent_id = static_cast<uint32_t>(current->parent_id);
      TreeNode * parent = tree_.find(parent_id);
      if (!parent) {
        d.action = BrainAction::kComplete;
        d.detail = "parent missing";
        return d;
      }
      d.action = BrainAction::kNavigateTo;
      d.goal_id = parent_id;
      d.goal = parent->position;
      d.detail = "backtracking to parent";
      return d;
    }

    const auto child_id = tree_.selectNextChild(
      current->id, nullptr, config_.dfs_prefer_highest_openness);
    if (!child_id.has_value()) {
      d.action = BrainAction::kWait;
      d.detail = "awaiting VLM scores";
      return d;
    }
    TreeNode * child = tree_.find(*child_id);
    if (!child) {
      d.action = BrainAction::kWait;
      d.detail = "selected child missing";
      return d;
    }
    d.action = BrainAction::kNavigateTo;
    d.goal_id = *child_id;
    d.goal = child->position;
    d.detail = "selected child score=" + std::to_string(child->openness_score);
    return d;
  }

private:
  ExplorationBrainConfig config_;
  FrontierTree tree_;
  std::vector<uint32_t> last_new_child_ids_;
};

class GreedyNearestBrain final : public ExplorationBrain
{
public:
  std::string id() const override {return "greedy_nearest";}
  bool wantsVlmScores() const override {return false;}
  bool usesFrontierTree() const override {return false;}

  std::vector<uint32_t> visitedIds() const override
  {
    return std::vector<uint32_t>(visited_.begin(), visited_.end());
  }

  std::vector<uint32_t> liveFrontierIds() const override
  {
    std::vector<uint32_t> out;
    for (const auto & f : live_) {
      out.push_back(f.id);
    }
    return out;
  }

  bool isVisited(uint32_t id) const override {return visited_.count(id) != 0;}
  bool isDead(uint32_t id) const override {return dead_.count(id) != 0;}

  void onEpisodeStart(const cv::Point2f & /*start_pose*/) override
  {
    live_.clear();
    visited_.clear();
    dead_.clear();
  }

  std::vector<uint32_t> onFrontiersDetected(
    const std::vector<FrontierCandidate> & frontiers) override
  {
    live_.clear();
    std::vector<uint32_t> accepted;
    for (const auto & f : frontiers) {
      if (visited_.count(f.id) || dead_.count(f.id)) {
        continue;
      }
      live_.push_back(f);
      accepted.push_back(f.id);
    }
    return accepted;
  }

  void onVlmScores(const std::unordered_map<uint32_t, uint8_t> & /*scores*/) override {}
  void softFailUnrated(uint8_t /*score*/) override {}

  void onArrived(uint32_t goal_id, const cv::Point2f & /*pose*/) override
  {
    visited_.insert(goal_id);
    eraseLive(goal_id);
  }

  void onNavFailed(uint32_t goal_id, bool mark_dead) override
  {
    if (mark_dead) {
      dead_.insert(goal_id);
      eraseLive(goal_id);
    }
  }

  BrainDecision selectNextGoal(const BrainContext & ctx) override
  {
    BrainDecision d;
    if (live_.empty()) {
      d.action = BrainAction::kComplete;
      d.detail = "no live frontiers";
      return d;
    }
    float best_dist2 = std::numeric_limits<float>::infinity();
    const FrontierCandidate * best = nullptr;
    for (const auto & f : live_) {
      const float dx = f.position.x - ctx.robot_pose.x;
      const float dy = f.position.y - ctx.robot_pose.y;
      const float dist2 = dx * dx + dy * dy;
      if (dist2 < best_dist2) {
        best_dist2 = dist2;
        best = &f;
      }
    }
    if (!best) {
      d.action = BrainAction::kComplete;
      d.detail = "no live frontiers";
      return d;
    }
    d.action = BrainAction::kNavigateTo;
    d.goal_id = best->id;
    d.goal = best->position;
    d.detail = "greedy nearest euclidean";
    return d;
  }

private:
  void eraseLive(uint32_t id)
  {
    live_.erase(
      std::remove_if(
        live_.begin(), live_.end(),
        [id](const FrontierCandidate & f) {return f.id == id;}),
      live_.end());
  }

  std::vector<FrontierCandidate> live_;
  std::unordered_set<uint32_t> visited_;
  std::unordered_set<uint32_t> dead_;
};

class VlmFrontierGraphBrain final : public ExplorationBrain
{
public:
  explicit VlmFrontierGraphBrain(const ExplorationBrainConfig & config)
  : config_(config) {}

  std::string id() const override {return "vlm_frontier_graph";}
  bool wantsVlmScores() const override {return true;}
  bool usesFrontierTree() const override {return false;}

  void setConfig(const ExplorationBrainConfig & config) override {config_ = config;}

  std::vector<uint32_t> visitedIds() const override {return graph_.visitedIds();}
  std::vector<uint32_t> liveFrontierIds() const override {return graph_.liveIds();}
  bool isVisited(uint32_t id) const override {return graph_.isVisited(id);}
  bool isDead(uint32_t id) const override {return graph_.isDead(id);}

  std::vector<GraphEdge> takePendingGraphEdges() override
  {
    std::vector<GraphEdge> out = pending_edges_;
    pending_edges_.clear();
    return out;
  }

  void onEpisodeStart(const cv::Point2f & start_pose) override
  {
    graph_.clear();
    pending_edges_.clear();
    last_new_ids_.clear();
    start_pose_ = start_pose;
  }

  std::vector<uint32_t> onFrontiersDetected(
    const std::vector<FrontierCandidate> & frontiers) override
  {
    last_new_ids_ = graph_.addFrontiers(frontiers, config_.graph_knn);
    pending_edges_ = graph_.edges();
    return last_new_ids_;
  }

  void onVlmScores(const std::unordered_map<uint32_t, uint8_t> & scores) override
  {
    for (const auto & e : scores) {
      graph_.setScore(e.first, e.second);
    }
  }

  void softFailUnrated(uint8_t score) override
  {
    graph_.softFailUnrated(last_new_ids_, score);
  }

  void onArrived(uint32_t goal_id, const cv::Point2f & /*pose*/) override
  {
    graph_.markVisited(goal_id);
    graph_.setCurrent(goal_id);
  }

  void onNavFailed(uint32_t goal_id, bool mark_dead) override
  {
    if (mark_dead) {
      graph_.markDead(goal_id);
    }
  }

  BrainDecision selectNextGoal(const BrainContext & ctx) override
  {
    BrainDecision d;
    auto candidates = candidatePool(ctx);
    if (candidates.empty()) {
      d.action = BrainAction::kComplete;
      d.detail = "graph: no live frontiers";
      return d;
    }

    bool any_unrated = false;
    uint32_t best_id = 0;
    uint8_t best_score = 0;
    bool have_best = false;
    for (uint32_t id : candidates) {
      const uint8_t s = graph_.score(id);
      if (s == kOpennessUnset) {
        any_unrated = true;
        continue;
      }
      const bool better = !have_best ||
        (config_.dfs_prefer_highest_openness ? s > best_score : s < best_score);
      if (better) {
        have_best = true;
        best_score = s;
        best_id = id;
      }
    }
    if (!have_best) {
      d.action = BrainAction::kWait;
      d.detail = any_unrated ? "graph: awaiting VLM scores" : "graph: no rated live";
      return d;
    }
    const auto pos = graph_.position(best_id);
    if (!pos) {
      d.action = BrainAction::kComplete;
      d.detail = "graph: missing pose";
      return d;
    }
    d.action = BrainAction::kNavigateTo;
    d.goal_id = best_id;
    d.goal = *pos;
    d.detail = "graph neighbor score=" + std::to_string(best_score);
    return d;
  }

private:
  std::vector<uint32_t> candidatePool(const BrainContext & ctx) const
  {
    if (graph_.haveCurrent()) {
      auto nbrs = graph_.neighbors(graph_.currentId());
      if (!nbrs.empty()) {
        return nbrs;
      }
    }
    // First hop / isolated: all live, prefer nearer to robot.
    auto live = graph_.liveIds();
    std::sort(live.begin(), live.end(), [&](uint32_t a, uint32_t b) {
      const auto pa = graph_.position(a);
      const auto pb = graph_.position(b);
      if (!pa || !pb) {
        return a < b;
      }
      return euclidean(*pa, ctx.robot_pose) < euclidean(*pb, ctx.robot_pose);
    });
    return live;
  }

  ExplorationBrainConfig config_;
  FrontierGraphMemory graph_;
  std::vector<GraphEdge> pending_edges_;
  std::vector<uint32_t> last_new_ids_;
  cv::Point2f start_pose_{0.f, 0.f};
};

class VlmChoiceDijkstraBrain final : public ExplorationBrain
{
public:
  explicit VlmChoiceDijkstraBrain(const ExplorationBrainConfig & config)
  : config_(config) {}

  std::string id() const override {return "vlm_choice_dijkstra";}
  bool wantsVlmScores() const override {return true;}
  bool usesFrontierTree() const override {return false;}

  void setConfig(const ExplorationBrainConfig & config) override {config_ = config;}

  std::vector<uint32_t> visitedIds() const override {return graph_.visitedIds();}
  std::vector<uint32_t> liveFrontierIds() const override {return graph_.liveIds();}
  bool isVisited(uint32_t id) const override {return graph_.isVisited(id);}
  bool isDead(uint32_t id) const override {return graph_.isDead(id);}

  std::vector<GraphEdge> takePendingGraphEdges() override
  {
    std::vector<GraphEdge> out = pending_edges_;
    pending_edges_.clear();
    return out;
  }

  std::optional<VlmChoiceRecord> takePendingVlmChoice() override
  {
    if (!pending_choice_) {
      return std::nullopt;
    }
    VlmChoiceRecord out = *pending_choice_;
    pending_choice_.reset();
    return out;
  }

  void onEpisodeStart(const cv::Point2f & start_pose) override
  {
    graph_.clear();
    pending_edges_.clear();
    pending_choice_.reset();
    last_new_ids_.clear();
    start_pose_ = start_pose;
  }

  std::vector<uint32_t> onFrontiersDetected(
    const std::vector<FrontierCandidate> & frontiers) override
  {
    last_new_ids_ = graph_.addFrontiers(frontiers, config_.graph_knn);
    pending_edges_ = graph_.edges();
    return last_new_ids_;
  }

  void onVlmScores(const std::unordered_map<uint32_t, uint8_t> & scores) override
  {
    for (const auto & e : scores) {
      graph_.setScore(e.first, e.second);
    }
  }

  void softFailUnrated(uint8_t score) override
  {
    graph_.softFailUnrated(last_new_ids_, score);
  }

  void onArrived(uint32_t goal_id, const cv::Point2f & /*pose*/) override
  {
    graph_.markVisited(goal_id);
    graph_.setCurrent(goal_id);
  }

  void onNavFailed(uint32_t goal_id, bool mark_dead) override
  {
    if (mark_dead) {
      graph_.markDead(goal_id);
    }
  }

  BrainDecision selectNextGoal(const BrainContext & ctx) override
  {
    BrainDecision d;
    auto candidates = dijkstraCandidates(ctx);
    if (candidates.empty()) {
      d.action = BrainAction::kComplete;
      d.detail = "choice+dijkstra: no live frontiers";
      return d;
    }

    bool any_unrated = false;
    uint32_t best_id = 0;
    uint8_t best_score = 0;
    bool have_best = false;
    for (uint32_t id : candidates) {
      const uint8_t s = graph_.score(id);
      if (s == kOpennessUnset) {
        any_unrated = true;
        continue;
      }
      if (!have_best || s > best_score) {
        have_best = true;
        best_score = s;
        best_id = id;
      }
    }
    if (!have_best) {
      d.action = BrainAction::kWait;
      d.detail = any_unrated ? "choice: awaiting VLM scores" : "choice: no rated candidates";
      return d;
    }

    const auto pos = graph_.position(best_id);
    if (!pos) {
      d.action = BrainAction::kComplete;
      d.detail = "choice: missing pose";
      return d;
    }

    // v1: multi-view choice approximated by score-argmax among Dijkstra-ordered
    // candidates; still emit prompt/response + selected id for event JSONL.
    std::ostringstream prompt;
    prompt << "Efficient explorer: among frontiers ";
    for (size_t i = 0; i < candidates.size(); ++i) {
      if (i) {
        prompt << ",";
      }
      prompt << candidates[i];
    }
    prompt << " (Dijkstra-neighborhood), which to take next?";
    std::ostringstream response;
    response << "selected " << best_id << " openness=" << static_cast<int>(best_score)
             << " (dijkstra-v1 score-argmax among candidates)";

    VlmChoiceRecord rec;
    rec.prompt = prompt.str();
    rec.response = response.str();
    rec.selected_frontier_id = best_id;
    rec.candidate_ids = candidates;
    pending_choice_ = rec;

    d.action = BrainAction::kNavigateTo;
    d.goal_id = best_id;
    d.goal = *pos;
    d.detail = "choice+dijkstra selected=" + std::to_string(best_id);
    return d;
  }

private:
  std::vector<uint32_t> dijkstraCandidates(const BrainContext & ctx) const
  {
    if (!graph_.haveCurrent()) {
      // First hop: every live frontier is a candidate.
      auto live = graph_.liveIds();
      std::sort(live.begin(), live.end(), [&](uint32_t a, uint32_t b) {
        const auto pa = graph_.position(a);
        const auto pb = graph_.position(b);
        if (!pa || !pb) {
          return a < b;
        }
        return euclidean(*pa, ctx.robot_pose) < euclidean(*pb, ctx.robot_pose);
      });
      return live;
    }

    const uint32_t source = graph_.currentId();
    auto nbrs = graph_.neighbors(source);
    std::vector<uint32_t> pool = nbrs.empty() ? graph_.liveIds() : nbrs;
    pool.erase(std::remove(pool.begin(), pool.end(), source), pool.end());
    if (pool.empty()) {
      return {};
    }

    const auto dist = graph_.dijkstra(source);
    std::sort(pool.begin(), pool.end(), [&](uint32_t a, uint32_t b) {
      const float da = dist.count(a) ? dist.at(a) : std::numeric_limits<float>::infinity();
      const float db = dist.count(b) ? dist.at(b) : std::numeric_limits<float>::infinity();
      if (da != db) {
        return da < db;
      }
      return a < b;
    });
    return pool;
  }

  ExplorationBrainConfig config_;
  FrontierGraphMemory graph_;
  std::vector<GraphEdge> pending_edges_;
  std::vector<uint32_t> last_new_ids_;
  std::optional<VlmChoiceRecord> pending_choice_;
  cv::Point2f start_pose_{0.f, 0.f};
};

}  // namespace

std::unique_ptr<ExplorationBrain> createExplorationBrain(
  const std::string & brain_id,
  const ExplorationBrainConfig & config)
{
  if (brain_id == "vlm_tree_dfs" || brain_id == "vlm_dfs") {
    return std::make_unique<VlmTreeDfsBrain>(config);
  }
  if (brain_id == "greedy_nearest") {
    return std::make_unique<GreedyNearestBrain>();
  }
  if (brain_id == "vlm_frontier_graph") {
    return std::make_unique<VlmFrontierGraphBrain>(config);
  }
  if (brain_id == "vlm_choice_dijkstra") {
    return std::make_unique<VlmChoiceDijkstraBrain>(config);
  }
  throw std::invalid_argument("Unknown exploration brain id: " + brain_id);
}

}  // namespace explorer_mission
