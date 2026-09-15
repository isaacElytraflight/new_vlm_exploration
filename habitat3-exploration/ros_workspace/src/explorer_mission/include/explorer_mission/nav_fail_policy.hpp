#pragma once

#include <cstdint>
#include <string>

namespace explorer_mission
{

/// After NavigateToPose to a NEW frontier fails, classify using theoretical
/// plan checks to both the NEW goal and the previously visited / scan pose.
enum class NewGoalFailClass : uint8_t
{
  /// Prior reachable, start OK, NEW definitively unplannable → mark dead, no thrash.
  kInaccessible = 0,
  /// Start wedged / prior bad / NEW still plannable / inconclusive → thrash recovery.
  kStuck = 1,
};

/// Nav2 ComputePathToPose error codes that mean the GOAL itself is unreachable
/// (not TF / start pose / timeout / planner-unavailable noise).
/// Values match nav2_msgs/action/ComputePathToPose.action.
constexpr uint16_t kComputePathNone = 0;
constexpr uint16_t kComputePathGoalOutsideMap = 204;
constexpr uint16_t kComputePathGoalOccupied = 206;
constexpr uint16_t kComputePathNoValidPath = 208;

/// Minimum NavigateToPose wall time used for logging / diagnostics only.
constexpr double kMinSubstantiveNavAttemptS = 1.5;

/// True when ComputePathToPose reported a goal/path failure we trust enough
/// to mark a frontier inaccessible (vs treating as stuck / retry).
bool isDefinitiveGoalUnreachable(uint16_t compute_path_error_code);

/// True when the failed NavigateToPose ran long enough to count as a real attempt.
bool isSubstantiveNavAttempt(double nav_attempt_s, double min_s = kMinSubstantiveNavAttemptS);

/// True when the robot has enough free-space clearance to treat NO_VALID_PATH as
/// a goal problem rather than a wedged start (mass-blacklist hazard).
bool isStartClearanceOk(double clearance_m, double min_clearance_m);

/// Inaccessible only when:
/// - prior is known + reachable,
/// - start clearance is OK (robot not wedged),
/// - NEW has no path with a definitive unreachable code.
/// Low clearance + NO_VALID_PATH → stuck (recover), do not blacklist every frontier.
NewGoalFailClass classifyNewGoalNavFailure(
  bool prior_pose_known,
  bool prior_plan_ok,
  bool new_goal_plan_ok,
  bool new_goal_unreachability_definitive,
  bool start_clearance_ok);

/// After stuck recovery + NEW retry still fails: mark dead only if start is clear
/// (goal problem). If still wedged, terminate stuck instead of burning the tree.
bool shouldMarkDeadAfterStuckRecovery(bool start_clearance_ok_after);

/// How an exploration episode ended (logged + collected into run metrics).
enum class TerminationReason : uint8_t
{
  kSuccess = 0,
  kStuck = 1,
  kTimeout = 2,
  kInterrupted = 3,
  kUnknown = 255,
};

const char * terminationReasonCStr(TerminationReason reason);
TerminationReason terminationReasonFromString(const std::string & s);

}  // namespace explorer_mission
