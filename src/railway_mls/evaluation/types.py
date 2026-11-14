"""Independent, immutable inputs for the 2018 rail evaluator."""

from dataclasses import dataclass, field

import numpy as np


def _readonly(value, dtype):
    result = np.array(value, dtype=dtype, copy=True)
    result.setflags(write=False)
    return result


@dataclass(frozen=True)
class EvaluationConfig:
    """The frozen source profile. Alternative protocols require a separate profile."""

    profile: str = "source2018"
    candidate_buffer: float = 0.3
    horizontal_threshold: float = 0.12
    vertical_threshold: float = 0.05

    def __post_init__(self):
        if (self.profile, self.candidate_buffer, self.horizontal_threshold,
                self.vertical_threshold) != ("source2018", 0.3, 0.12, 0.05):
            raise ValueError("source2018 has fixed 0.3/0.12/0.05 metre settings")


@dataclass(frozen=True)
class ReferencePoints:
    xyz: np.ndarray
    raw_classification: np.ndarray
    record_ids: np.ndarray | None = None

    def __post_init__(self):
        xyz = _readonly(self.xyz, np.float64)
        labels = np.asarray(self.raw_classification)
        if xyz.ndim != 2 or xyz.shape[1] != 3 or not np.all(np.isfinite(xyz)):
            raise ValueError("reference XYZ must be a finite N by 3 array")
        if (labels.shape != (len(xyz),) or not np.issubdtype(labels.dtype, np.integer)
                or np.any(labels < 0) or np.any(labels > 255)):
            raise ValueError("raw classification must contain N bytes")
        ids = np.arange(len(xyz), dtype=np.int64) if self.record_ids is None else np.asarray(self.record_ids)
        if (ids.shape != (len(xyz),) or not np.issubdtype(ids.dtype, np.integer)
                or np.any(ids < 0) or np.any(ids > np.iinfo(np.int64).max)
                or np.any(ids[1:] <= ids[:-1])):
            raise ValueError("record IDs must be increasing nonnegative int64 values")
        object.__setattr__(self, "xyz", xyz)
        object.__setattr__(self, "raw_classification", _readonly(labels, np.uint8))
        object.__setattr__(self, "record_ids", _readonly(ids, np.int64))

    def __len__(self):
        return len(self.xyz)


@dataclass(frozen=True)
class RailSegments:
    """Endpoints in original file order: shape (segment, begin/end, XYZ)."""

    endpoints: np.ndarray
    provenance: dict = field(default_factory=dict)

    def __post_init__(self):
        endpoints = _readonly(self.endpoints, np.float64)
        if endpoints.shape == (0,):
            endpoints = _readonly(np.empty((0, 2, 3)), np.float64)
        if (endpoints.ndim != 3 or endpoints.shape[1:] != (2, 3)
                or not np.all(np.isfinite(endpoints))):
            raise ValueError("rail endpoints must be a finite N by 2 by 3 array")
        object.__setattr__(self, "endpoints", endpoints)
        object.__setattr__(self, "provenance", dict(self.provenance))

    def __len__(self):
        return len(self.endpoints)


POINT_DTYPE = np.dtype([
    ("record_id", "<i8"), ("segment_id", "<i8"),
    ("raw_classification", "u1"), ("outcome", "u1"),
    ("horizontal", "<f8"), ("signed_vertical", "<f8"),
])


@dataclass
class EvaluationResult:
    status: str
    counts: dict
    coverage: dict
    metrics: dict
    segments: dict
    diagnostics: list[str]
    points: np.ndarray | None = None

    def summary(self):
        return {"status": self.status, "profile": "source2018", "counts": self.counts,
                "coverage": self.coverage, "metrics": self.metrics,
                "segments": self.segments, "diagnostics": self.diagnostics}
