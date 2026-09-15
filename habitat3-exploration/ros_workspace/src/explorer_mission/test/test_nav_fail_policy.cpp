#include <gtest/gtest.h>

#include <limits>

#include "explorer_mission/nav_fail_policy.hpp"

using explorer_mission::NewGoalFailClass;
using explorer_mission::TerminationReason;
using explorer_mission::classifyNewGoalNavFailure;
using explorer_mission::isDefinitiveGoalUnreachable;
using explorer_mission::isStartClearanceOk;
using explorer_mission::isSubstantiveNavAttempt;
using explorer_mission::kComputePathGoalOccupied;
using explorer_mission::kComputePathGoalOutsideMap;
using explorer_mission::kComputePathNoValidPath;
using explorer_mission::kComputePathNone;
using explorer_mission::kMinSubstantiveNavAttemptS;
using explorer_mission::terminationReasonCStr;
using explorer_mission::terminationReasonFromString;
using explorer_mission::shouldMarkDeadAfterStuckRecovery;

TEST(NavFailPolicyHarness, RunnerExecutesAssertions)
{
  EXPECT_EQ(1 + 1, 2);
}

TEST(NavFailPolicyHarness, IntentionalFailureIsDetectable_NegativeControl)
{
  EXPECT_FALSE(1 == 2);
}

TEST(NavFailPolicy, DefinitiveUnreachableCodes_Positive)
{
  EXPECT_TRUE(isDefinitiveGoalUnreachable(kComputePathNoValidPath));
  EXPECT_TRUE(isDefinitiveGoalUnreachable(kComputePathGoalOccupied));
  EXPECT_TRUE(isDefinitiveGoalUnreachable(kComputePathGoalOutsideMap));
}

TEST(NavFailPolicy, NonDefinitiveCodes_Negative)
{
  EXPECT_FALSE(isDefinitiveGoalUnreachable(kComputePathNone));
  EXPECT_FALSE(isDefinitiveGoalUnreachable(205));  // START_OCCUPIED
}

TEST(NavFailPolicy, StartClearanceOk_Positive)
{
  EXPECT_TRUE(isStartClearanceOk(0.5, 0.25));
  EXPECT_TRUE(isStartClearanceOk(0.25, 0.25));
}

TEST(NavFailPolicy, StartClearanceLowOrUnknown_Negative)
{
  EXPECT_FALSE(isStartClearanceOk(0.10, 0.25));
  EXPECT_FALSE(isStartClearanceOk(std::numeric_limits<double>::quiet_NaN(), 0.25));
}

TEST(NavFailPolicy, InfiniteClearanceMeansOk_Positive)
{
  EXPECT_TRUE(isStartClearanceOk(std::numeric_limits<double>::infinity(), 0.25));
}

TEST(NavFailPolicy, PriorOkDefinitiveWithClearStartMeansInaccessible_Positive)
{
  EXPECT_EQ(
    classifyNewGoalNavFailure(true, true, false, true, /*start_ok=*/true),
    NewGoalFailClass::kInaccessible);
}

TEST(NavFailPolicy, WedgedStartWithNoPathMeansStuck_NotBlacklist_Positive)
{
  // Same NO_VALID_PATH that would mass-blacklist if we ignored start clearance.
  EXPECT_EQ(
    classifyNewGoalNavFailure(true, true, false, true, /*start_ok=*/false),
    NewGoalFailClass::kStuck);
}

TEST(NavFailPolicy, MarkDeadOnlyWhenStartClearAfterRecovery_Positive)
{
  EXPECT_TRUE(explorer_mission::shouldMarkDeadAfterStuckRecovery(true));
}

TEST(NavFailPolicy, DoNotMarkDeadWhenStillWedged_Negative)
{
  EXPECT_FALSE(explorer_mission::shouldMarkDeadAfterStuckRecovery(false));
}

TEST(NavFailPolicy, PriorOkButInconclusiveMeansStuck_Positive)
{
  EXPECT_EQ(
    classifyNewGoalNavFailure(true, true, false, false, true),
    NewGoalFailClass::kStuck);
}

TEST(NavFailPolicy, MissingPriorMeansStuck_Negative)
{
  EXPECT_EQ(
    classifyNewGoalNavFailure(false, true, false, true, true),
    NewGoalFailClass::kStuck);
}

TEST(TerminationReason, RoundTripStrings_Positive)
{
  EXPECT_STREQ(terminationReasonCStr(TerminationReason::kSuccess), "success");
  EXPECT_EQ(terminationReasonFromString("stuck"), TerminationReason::kStuck);
}

TEST(TerminationReason, UnknownString_Negative)
{
  EXPECT_EQ(terminationReasonFromString("nope"), TerminationReason::kUnknown);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
