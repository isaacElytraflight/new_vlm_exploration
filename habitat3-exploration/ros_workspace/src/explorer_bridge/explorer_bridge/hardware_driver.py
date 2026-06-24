"""Stub driver for future physical robot backend."""

from __future__ import annotations

from explorer_bridge.driver_protocol import ObservationData, StepResult


class HardwareNotConfiguredError(RuntimeError):
    pass


class HardwareDriver:
    def get_observations(self) -> ObservationData:
        raise HardwareNotConfiguredError(
            "driver_backend=hardware is not wired yet. Populate real/ and implement "
            "HardwareDriver before connecting a physical robot."
        )

    def step(self, action: str, count: int = 1) -> StepResult:
        raise HardwareNotConfiguredError(
            "driver_backend=hardware is not wired yet."
        )

    def reset(self) -> None:
        raise HardwareNotConfiguredError(
            "driver_backend=hardware is not wired yet."
        )

    def shutdown(self) -> None:
        pass
