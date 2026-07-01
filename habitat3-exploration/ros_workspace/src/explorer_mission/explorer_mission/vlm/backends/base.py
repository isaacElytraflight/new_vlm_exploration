"""VLM backend protocol."""

from __future__ import annotations

from typing import Protocol, Sequence

from sensor_msgs.msg import CompressedImage


class VlmBackend(Protocol):
    @property
    def name(self) -> str:
        ...

    @property
    def model_label(self) -> str:
        ...

    def validate(self) -> None:
        """Raise on misconfiguration or unreachable provider."""

    def query(self, prompt: str, images: Sequence[tuple[str, CompressedImage]]) -> str:
        """Return model text response for the prompt and labeled images."""
