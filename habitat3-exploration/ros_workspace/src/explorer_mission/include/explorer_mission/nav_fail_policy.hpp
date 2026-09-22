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

/// Inaccessible when:
/// - prior is known + reachable,
/// - NEW has no path with a definitive unreachable code,
/// - AND start clearance is OK (failure is about the goal, not the robot).
/// Instant NO_VALID_PATH while wedged must NOT be inaccessible — that mass-
/// blacklists every remaining frontier (ablation_run_20260920_172413).
/// Low clearance + definitive NO_VALID_PATH → stuck (recover).
/// `nav_attempt_substantive` is retained for call-site / log compatibility.
NewGoalFailClass classifyNewGoalNavFailure(
  bool prior_pose_known,
  bool prior_plan_ok,
  bool new_goal_plan_ok,
  bool new_goal_unreachability_definitive,
  bool start_clearance_ok,
  bool nav_attempt_substantive = true);

/// Stable string for NavFailEvent.fail_class / events.jsonl ("inaccessible"|"stuck").
const char * newGoalFailClassCStr(NewGoalFailClass c);

/// After stuck recovery + NEW retry still fails: mark dead only if start is
/// clear enough that the failure is about the NEW goal. Still-wedged mark_dead
/// falsely invalidates remaining frontiers (ablation_run_20260920_172413).
bool shouldMarkDeadAfterStuckRecovery(bool start_clearance_ok_after);

/// Cap on wedged_keep_live retries for the same goal before soft-skipping it
/// (drop from live_, not geographic dead_). Seed1 (215537): 317 retries / 31 min.
constexpr uint32_t kMaxWedgedKeepLivePerGoal = 1;

/// True once keep-live attempts on this goal reach the cap (soft-skip next).
bool shouldSoftSkipWedgedGoal(uint32_t keep_live_attempts_on_goal);

/// Brain "no live frontiers" is only a real success when the robot is not wedged.
/// Otherwise soft-skips emptied live_ and would fake-complete (seed1 hazard).
bool shouldAcceptBrainComplete(bool start_clearance_ok);

/// When thrash + return-to-prior fails: never end the episode here — mark the
/// NEW frontier dead and continue. Alternate sanctuaries are best-effort only.
/// (Ablation seed0: nearPose short-circuit + terminate left 17 live frontiers.)
bool shouldTerminateAfterReturnToPriorFailed(bool have_alternate_sanctuary);

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
