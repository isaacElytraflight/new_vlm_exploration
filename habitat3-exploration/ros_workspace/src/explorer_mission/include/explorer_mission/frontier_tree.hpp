#pragma once

#include <cstdint>
#include <optional>
#include <random>
#include <vector>

#include <geometry_msgs/msg/point.hpp>
#include <opencv2/core.hpp>

#include <explorer_msgs/msg/frontier_tree.hpp>

namespace explorer_mission
{

constexpr uint8_t kOpennessNotRated = 255;

struct TreeNode
{
  uint32_t id{0};
  cv::Point2f position;
  int32_t parent_id{-1};
  std::vector<uint32_t> children_ids;
  uint8_t openness_score{kOpennessNotRated};
  bool fully_explored{false};
};

class FrontierTree
{
public:
  uint32_t createRoot(const cv::Point2f & position);
  uint32_t addChild(
    uint32_t parent_id,
    const cv::Point2f & position,
    uint8_t openness_score,
    bool fully_explored);
  void setOpennessScore(uint32_t id, uint8_t score);
  void markFullyExplored(uint32_t id);
  bool hasUnexploredChildren(uint32_t id) const;
  bool hasUnexploredNodesExcluding(uint32_t id_a, uint32_t id_b) const;
  std::optional<uint32_t> selectNextChild(uint32_t parent_id, std::mt19937 * rng = nullptr) const;
  std::vector<cv::Point2f> allNodePositions() const;
  TreeNode * find(uint32_t id);
  const TreeNode * find(uint32_t id) const;
  uint32_t rootId() const {return root_id_;}
  void setCurrentNodeId(uint32_t id) {current_node_id_ = id;}
  uint32_t currentNodeId() const {return current_node_id_;}
  explorer_msgs::msg::FrontierTree toMsg(
    const std::string & frame_id,
    int32_t stamp_sec,
    uint32_t stamp_nanosec) const;

private:
  uint32_t next_id_{1};
  uint32_t root_id_{0};
  uint32_t current_node_id_{0};
  std::vector<TreeNode> nodes_;
};

}  // namespace explorer_mission
