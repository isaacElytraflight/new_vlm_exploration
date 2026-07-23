#include "explorer_mission/frontier_tree.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace explorer_mission
{

uint32_t FrontierTree::createRoot(const cv::Point2f & position)
{
  nodes_.clear();
  next_id_ = 1;
  TreeNode root;
  root.id = 0;
  root.position = position;
  root.parent_id = -1;
  root.openness_score = kOpennessNotRated;
  root.fully_explored = false;
  nodes_.push_back(root);
  root_id_ = 0;
  current_node_id_ = 0;
  return root_id_;
}

uint32_t FrontierTree::addChild(
  uint32_t parent_id,
  const cv::Point2f & position,
  uint8_t openness_score,
  bool fully_explored)
{
  if (!find(parent_id)) {
    return std::numeric_limits<uint32_t>::max();
  }
  TreeNode child;
  child.id = next_id_++;
  child.position = position;
  child.parent_id = static_cast<int32_t>(parent_id);
  child.openness_score = openness_score;
  child.fully_explored = fully_explored;
  nodes_.push_back(child);

  TreeNode * parent = find(parent_id);
  if (parent) {
    parent->children_ids.push_back(child.id);
  }
  return child.id;
}

void FrontierTree::setOpennessScore(uint32_t id, uint8_t score)
{
  TreeNode * node = find(id);
  if (node) {
    node->openness_score = score;
    if (score == 0) {
      node->fully_explored = true;
    }
  }
}

void FrontierTree::markFullyExplored(uint32_t id)
{
  TreeNode * node = find(id);
  if (node) {
    node->fully_explored = true;
  }
}

bool FrontierTree::hasUnexploredChildren(uint32_t id) const
{
  const TreeNode * node = find(id);
  if (!node) {
    return false;
  }
  for (uint32_t child_id : node->children_ids) {
    const TreeNode * child = find(child_id);
    if (child && !child->fully_explored) {
      return true;
    }
  }
  return false;
}

bool FrontierTree::hasUnexploredNodesExcluding(uint32_t id_a, uint32_t id_b) const
{
  for (const auto & node : nodes_) {
    if (node.id == id_a || node.id == id_b) {
      continue;
    }
    if (!node.fully_explored) {
      return true;
    }
  }
  return false;
}

std::optional<uint32_t> FrontierTree::selectNextChild(
  uint32_t parent_id,
  std::mt19937 * rng,
  bool prefer_highest) const
{
  const TreeNode * parent = find(parent_id);
  if (!parent) {
    return std::nullopt;
  }

  std::vector<uint32_t> candidates;
  bool have_best = false;
  uint8_t best_score = 0;
  for (uint32_t child_id : parent->children_ids) {
    const TreeNode * child = find(child_id);
    if (!child || child->fully_explored) {
      continue;
    }
    if (child->openness_score == kOpennessNotRated) {
      continue;
    }
    const bool better = !have_best ||
      (prefer_highest ? child->openness_score > best_score :
      child->openness_score < best_score);
    if (better) {
      best_score = child->openness_score;
      candidates.clear();
      candidates.push_back(child_id);
      have_best = true;
    } else if (child->openness_score == best_score) {
      candidates.push_back(child_id);
    }
  }

  if (candidates.empty()) {
    return std::nullopt;
  }
  if (candidates.size() == 1) {
    return candidates.front();
  }

  std::mt19937 local_rng{std::random_device{}()};
  std::mt19937 & use_rng = rng ? *rng : local_rng;
  std::uniform_int_distribution<size_t> dist(0, candidates.size() - 1);
  return candidates[dist(use_rng)];
}

std::optional<uint32_t> FrontierTree::selectBestAmong(
  const std::vector<uint32_t> & candidate_ids,
  std::mt19937 * rng,
  bool prefer_highest) const
{
  std::vector<uint32_t> candidates;
  bool have_best = false;
  uint8_t best_score = 0;
  for (uint32_t child_id : candidate_ids) {
    const TreeNode * child = find(child_id);
    if (!child || child->fully_explored) {
      continue;
    }
    if (child->openness_score == kOpennessNotRated) {
      continue;
    }
    const bool better = !have_best ||
      (prefer_highest ? child->openness_score > best_score :
      child->openness_score < best_score);
    if (better) {
      best_score = child->openness_score;
      candidates.clear();
      candidates.push_back(child_id);
      have_best = true;
    } else if (child->openness_score == best_score) {
      candidates.push_back(child_id);
    }
  }

  if (candidates.empty()) {
    return std::nullopt;
  }
  if (candidates.size() == 1) {
    return candidates.front();
  }

  std::mt19937 local_rng{std::random_device{}()};
  std::mt19937 & use_rng = rng ? *rng : local_rng;
  std::uniform_int_distribution<size_t> dist(0, candidates.size() - 1);
  return candidates[dist(use_rng)];
}

std::optional<uint32_t> FrontierTree::findNearestNode(
  const cv::Point2f & position,
  const std::vector<uint32_t> * candidate_ids) const
{
  const TreeNode * best = nullptr;
  double best_dist = 0.0;

  auto consider = [&](const TreeNode & node) {
    const double d = std::hypot(
      position.x - node.position.x,
      position.y - node.position.y);
    if (!best || d < best_dist) {
      best = &node;
      best_dist = d;
    }
  };

  if (candidate_ids) {
    for (uint32_t id : *candidate_ids) {
      const TreeNode * node = find(id);
      if (node) {
        consider(*node);
      }
    }
  } else {
    for (const auto & node : nodes_) {
      consider(node);
    }
  }

  if (!best) {
    return std::nullopt;
  }
  return best->id;
}

std::vector<uint32_t> FrontierTree::allNodeIds() const
{
  std::vector<uint32_t> ids;
  ids.reserve(nodes_.size());
  for (const auto & node : nodes_) {
    ids.push_back(node.id);
  }
  return ids;
}

std::vector<cv::Point2f> FrontierTree::allNodePositions() const
{
  std::vector<cv::Point2f> positions;
  positions.reserve(nodes_.size());
  for (const auto & node : nodes_) {
    positions.push_back(node.position);
  }
  return positions;
}

TreeNode * FrontierTree::find(uint32_t id)
{
  for (auto & node : nodes_) {
    if (node.id == id) {
      return &node;
    }
  }
  return nullptr;
}

const TreeNode * FrontierTree::find(uint32_t id) const
{
  for (const auto & node : nodes_) {
    if (node.id == id) {
      return &node;
    }
  }
  return nullptr;
}

explorer_msgs::msg::FrontierTree FrontierTree::toMsg(
  const std::string & frame_id,
  int32_t stamp_sec,
  uint32_t stamp_nanosec) const
{
  explorer_msgs::msg::FrontierTree msg;
  msg.header.frame_id = frame_id;
  msg.header.stamp.sec = stamp_sec;
  msg.header.stamp.nanosec = stamp_nanosec;
  msg.current_node_id = current_node_id_;
  msg.nodes.reserve(nodes_.size());
  for (const auto & node : nodes_) {
    explorer_msgs::msg::FrontierTreeNode out;
    out.id = node.id;
    out.position.x = node.position.x;
    out.position.y = node.position.y;
    out.position.z = 0.0;
    out.parent_id = node.parent_id;
    out.children_ids = node.children_ids;
    out.openness_score = node.openness_score;
    out.fully_explored = node.fully_explored;
    msg.nodes.push_back(out);
  }
  return msg;
}

}  // namespace explorer_mission
