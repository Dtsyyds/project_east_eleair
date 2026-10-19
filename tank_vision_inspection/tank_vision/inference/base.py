"""Common inference contract; decoding remains model-specific."""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Mapping

import numpy as np


@dataclass(frozen=True, slots=True)
class TensorSpec:
    name: str
    shape: tuple[int | str | None, ...] = ()
    dtype: str = "unknown"
    layout: str | None = None
    color_format: str | None = None
    metadata: Mapping[str, Any] = field(default_factory=dict)


class InferenceBackend(ABC):
    """Runtime-only interface; no backend is allowed to guess model semantics."""

    name: str

    @property
    @abstractmethod
    def available(self) -> bool:
        """Whether the required runtime or injected adapter can be used."""

    @property
    @abstractmethod
    def unavailable_reason(self) -> str | None:
        """A concrete reason when :attr:`available` is false."""

    @property
    @abstractmethod
    def input_spec(self) -> tuple[TensorSpec, ...]:
        """Raw model input contract."""

    @property
    @abstractmethod
    def output_spec(self) -> tuple[TensorSpec, ...]:
        """Raw model output contract."""

    @abstractmethod
    def load_model(self, model_path: str | Path, metadata: Mapping[str, Any] | None = None) -> None:
        """Load one platform-compatible model."""

    @abstractmethod
    def infer(self, inputs: np.ndarray | Mapping[str, np.ndarray]) -> list[np.ndarray]:
        """Return raw output tensors without semantic decoding."""

    @abstractmethod
    def warmup(self, iterations: int = 1) -> None:
        """Run backend-specific warmup when input metadata is sufficient."""

    @abstractmethod
    def close(self) -> None:
        """Release all runtime resources; safe to call repeatedly."""

