"""Pure logic for translating /cmd_vel to discrete move intents (testable)."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Tuple

from explorer_msgs.action import DiscreteMove


@dataclass(frozen=True)
class CmdVelThresholds:
    angular_threshold: float = 0.15
    linear_threshold: float = 0.03
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
) -> Optional[DiscreteMoveIntent]:
    """Return a single discrete step intent, or None if below thresholds."""
    t = thresholds or CmdVelThresholds()
    if abs(angular_z) > t.angular_threshold:
        direction = (
            DiscreteMove.Goal.TURN_LEFT if angular_z > 0 else DiscreteMove.Goal.TURN_RIGHT
        )
        return DiscreteMoveIntent(direction=direction, steps=1)
    if abs(linear_x) > t.linear_threshold:
        direction = (
            DiscreteMove.Goal.FORWARD if linear_x > 0 else DiscreteMove.Goal.BACKWARD
        )
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
