#pragma once

#include <cstdint>

namespace explorer_mission
{

/// Blocks sibling frontier selection until the robot returns to its scan node
/// after a child navigation failure (prevents cascade frontier exhaustion).
class ReturnHomeGuard
{
public:
  void onChildNavFailed(uint32_t scan_node_id)
  {
    awaiting_return_ = true;
    scan_node_id_ = scan_node_id;
  }

  void onReturnHomeSucceeded()
  {
    awaiting_return_ = false;
  }

  bool isAwaitingReturn() const
  {
    return awaiting_return_;
  }

  bool maySelectFrontierChild() const
  {
    return !awaiting_return_;
  }

  uint32_t scanNodeId() const
  {
    return scan_node_id_;
  }

private:
  bool awaiting_return_{false};
  uint32_t scan_node_id_{0};
};

}  // namespace explorer_mission
