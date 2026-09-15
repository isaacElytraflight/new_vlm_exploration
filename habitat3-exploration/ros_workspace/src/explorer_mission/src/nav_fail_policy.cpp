#include "explorer_mission/nav_fail_policy.hpp"

#include <cmath>

namespace explorer_mission
{

bool isDefinitiveGoalUnreachable(uint16_t compute_path_error_code)
{
  switch (compute_path_error_code) {
    case kComputePathGoalOutsideMap:
    case kComputePathGoalOccupied:
    case kComputePathNoValidPath:
      return true;
    default:
      return false;
  }
}

bool isSubstantiveNavAttempt(double nav_attempt_s, double min_s)
{
  return std::isfinite(nav_attempt_s) && nav_attempt_s >= min_s;
}

bool isStartClearanceOk(double clearance_m, double min_clearance_m)
{
  if (std::isnan(clearance_m)) {
    return false;
  }
  // +inf (no occupied cells / empty map) counts as clear.
  if (!std::isfinite(clearance_m)) {
    return true;
  }
  return clearance_m >= min_clearance_m;
}

NewGoalFailClass classifyNewGoalNavFailure(
  bool prior_pose_known,
  bool prior_plan_ok,
  bool new_goal_plan_ok,
  bool new_goal_unreachability_definitive,
  bool start_clearance_ok)
{
  if (prior_pose_known && prior_plan_ok && start_clearance_ok &&
    !new_goal_plan_ok && new_goal_unreachability_definitive)
  {
    return NewGoalFailClass::kInaccessible;
  }
  return NewGoalFailClass::kStuck;
}

bool shouldMarkDeadAfterStuckRecovery(bool start_clearance_ok_after)
{
  return start_clearance_ok_after;
}

const char * terminationReasonCStr(TerminationReason reason)
{
  switch (reason) {
    case TerminationReason::kSuccess:
      return "success";
    case TerminationReason::kStuck:
      return "stuck";
    case TerminationReason::kTimeout:
      return "timeout";
    case TerminationReason::kInterrupted:
      return "interrupted";
    case TerminationReason::kUnknown:
    default:
      return "unknown";
  }
}

TerminationReason terminationReasonFromString(const std::string & s)
{
  if (s == "success") {
    return TerminationReason::kSuccess;
  }
  if (s == "stuck") {
    return TerminationReason::kStuck;
  }
  if (s == "timeout") {
    return TerminationReason::kTimeout;
  }
  if (s == "interrupted") {
    return TerminationReason::kInterrupted;
  }
  return TerminationReason::kUnknown;
}

}  // namespace explorer_mission
