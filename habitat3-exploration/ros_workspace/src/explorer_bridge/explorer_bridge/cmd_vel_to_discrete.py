"""Pure logic for translating /cmd_vel to discrete move intents (testable)."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Tuple

from explorer_msgs.action import DiscreteMove


@dataclass(frozen=True)
class CmdVelThresholds:
    """Thresholds for quantizing continuous Twist into Habitat DiscreteMove."""

    # Pure rotate-in-place (linear≈0): accept small angular so Nav2 can align.
    angular_threshold: float = 0.05
    linear_threshold: float = 0.03
    # Curvature gate: |angular|/|linear| (rad/m). Above → turn; below → drive.
    # 0.5 catches mild RPP corner arcs (~0.48) that ratio=1.0 used to FORWARD
    # straight into walls (0.25 m Habitat steps).
    turn_over_drive_ratio: float = 0.5
    # Absolute heading gate: never FORWARD while |angular| >= this, even if
    # curvature ratio is low. Matches coarse 0.25 m discrete steps.
    drive_max_angular: float = 0.12
    # Once turning, ignore opposite angular below this — unless we have already
    # committed max_turn_steps_before_flip (≈180°), then allow the short way.
    turn_flip_angular_threshold: float = 0.2
    max_turn_steps_before_flip: int = 18  # 18 × 10° = 180°
    turn_step_deg: float = 10.0
    move_step_m: float = 0.25


@dataclass(frozen=True)
class DiscreteMoveIntent:
    direction: int
    steps: int = 1


def _turn_intent(
    angular_z: float,
    t: CmdVelThresholds,
    last_turn_direction: int | None,
    consecutive_turn_steps: int,
) -> DiscreteMoveIntent:
    direction = (
        DiscreteMove.Goal.TURN_LEFT if angular_z > 0 else DiscreteMove.Goal.TURN_RIGHT
    )
    allow_weak_flip = consecutive_turn_steps >= t.max_turn_steps_before_flip
    if (
        last_turn_direction is not None
        and direction != last_turn_direction
        and abs(angular_z) < t.turn_flip_angular_threshold
        and not allow_weak_flip
    ):
        direction = last_turn_direction
    return DiscreteMoveIntent(direction=direction, steps=1)


def cmd_vel_to_intent(
    linear_x: float,
    angular_z: float,
    thresholds: CmdVelThresholds | None = None,
    last_turn_direction: int | None = None,
    consecutive_turn_steps: int = 0,
) -> Optional[DiscreteMoveIntent]:
    """Return a single discrete step intent, or None if below thresholds.

    Prefer **drive** only when linear is significant and heading is roughly
    aligned. Prefer **turn** when:
    - |angular_z|/|linear_x| >= turn_over_drive_ratio, or
    - |angular_z| >= drive_max_angular (absolute heading gate for 0.25 m steps).
    Rotate-in-place when linear is below threshold.

    When already turning, require |angular_z| >= turn_flip_angular_threshold to
    reverse direction — unless we have already committed max_turn_steps_before_flip
    (≈180°), in which case allow a weak opposite to take the short way.
    """
    t = thresholds or CmdVelThresholds()
    abs_lin = abs(linear_x)
    abs_ang = abs(angular_z)

    if abs_lin > t.linear_threshold:
        high_curvature = (
            abs_ang > t.angular_threshold
            and (abs_ang / abs_lin) >= t.turn_over_drive_ratio
        )
        heading_misaligned = abs_ang >= t.drive_max_angular
        if high_curvature or heading_misaligned:
            return _turn_intent(angular_z, t, last_turn_direction, consecutive_turn_steps)
        direction = (
            DiscreteMove.Goal.FORWARD if linear_x > 0 else DiscreteMove.Goal.BACKWARD
        )
        return DiscreteMoveIntent(direction=direction, steps=1)

    if abs_ang > t.angular_threshold:
        return _turn_intent(angular_z, t, last_turn_direction, consecutive_turn_steps)
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
