"""Pure logic for translating /cmd_vel to discrete move intents (testable)."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Tuple

from explorer_msgs.action import DiscreteMove


@dataclass(frozen=True)
class CmdVelThresholds:
    # Pure rotate-in-place (linear≈0): accept small angular so Nav2 can align.
    angular_threshold: float = 0.05
    linear_threshold: float = 0.03
    # Once turning, ignore opposite angular below this (stops ±0.1 rad/s twitch).
    turn_flip_angular_threshold: float = 0.2
    turn_step_deg: float = 10.0
    move_step_m: float = 0.25


@dataclass(frozen=True)
class DiscreteMoveIntent:
    direction: int
    steps: int = 1


def cmd_vel_to_intent(
    linear_x: float,
    angular_z: float,
    thresholds: CmdVelThresholds | None = None,
    last_turn_direction: int | None = None,
) -> Optional[DiscreteMoveIntent]:
    """Return a single discrete step intent, or None if below thresholds.

    Prefer **drive** when linear is significant. RPP path following always mixes
    a little angular with forward velocity; angular-first priority caused endless
    10° turns until nearly perfectly aligned (Habitat jitter).
    Rotate-in-place only when linear is below threshold.

    When already turning, require |angular_z| >= turn_flip_angular_threshold to
    reverse direction — otherwise RPP ±0.1 rad/s flips cancel as 10° left/right.
    """
    t = thresholds or CmdVelThresholds()
    if abs(linear_x) > t.linear_threshold:
        direction = (
            DiscreteMove.Goal.FORWARD if linear_x > 0 else DiscreteMove.Goal.BACKWARD
        )
        return DiscreteMoveIntent(direction=direction, steps=1)
    if abs(angular_z) > t.angular_threshold:
        direction = (
            DiscreteMove.Goal.TURN_LEFT if angular_z > 0 else DiscreteMove.Goal.TURN_RIGHT
        )
        if (
            last_turn_direction is not None
            and direction != last_turn_direction
            and abs(angular_z) < t.turn_flip_angular_threshold
        ):
            direction = last_turn_direction
        return DiscreteMoveIntent(direction=direction, steps=1)
    return None


def apply_realtime_rate_cap(
    linear_x: float,
    angular_z: float,
    *,
    max_linear_m_s: float,
    max_angular_deg_s: float,
) -> Tuple[float, float]:
    """Clamp cmd_vel components to real-time motion limits."""
    max_angular_rad_s = max_angular_deg_s * 3.141592653589793 / 180.0
    lin = max(-max_linear_m_s, min(max_linear_m_s, linear_x))
    ang = max(-max_angular_rad_s, min(max_angular_rad_s, angular_z))
    return lin, ang
