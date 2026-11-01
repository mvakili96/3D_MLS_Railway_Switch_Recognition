"""One inspectable in-memory workflow over the existing source-first components."""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import asdict, dataclass, fields, is_dataclass, replace
from hashlib import sha256
import json
from math import isfinite
from operator import index

import numpy as np

from .config import ExtractionConfig
from .coverage import coverage_summary
from .geometry import direction_mode, source_angle
from .grades import GWADExport, GradeAnalysis, build_gwad_rows
from .initialization import InitializationResult
from .models import InitialLocation, ToePoint, TrackStack
from .orchestration import BidirectionalTracking, TrackingEngine, TrackingInvocation, TrackingPass
from .points import Point3D, PointCloud, PointSelection
from .sampling import MsvcRand, RandSource
from .spatial import Bounds3D, GridIndex


@dataclass(frozen=True)
class PipelineOptions:
    bidirectional: bool = True
    grades: bool = False
    gwad: bool = False
    max_steps: int | None = None
    random_seed: int = 1
    record_random: bool = False
    coordinate_units: str | None = None
    crs: str | None = None

    def __post_init__(self) -> None:
        object.__setattr__(self, "random_seed", index(self.random_seed))
        if self.max_steps is not None:
            limit = index(self.max_steps)
            if limit < 0:
                raise ValueError("max_steps must be nonnegative")
            object.__setattr__(self, "max_steps", limit)
        for name in ("coordinate_units", "crs"):
            if getattr(self, name) is not None and not isinstance(getattr(self, name), str):
                raise ValueError(f"{name} must be an explicit string or None")


@dataclass(frozen=True)
class PipelineStage:
    name: str
    status: str


@dataclass(frozen=True)
class PipelineFinding:
    limitation: str | None
    status: str
    location: str


class _CountedRand:
    def __init__(self, source: RandSource, record: bool):
        self.source, self.record = source, record
        self.draw_count = 0
        self.draws: list[int] = []

    def rand(self) -> int:
        value = self.source.rand()
        self.draw_count += 1
        if self.record:
            self.draws.append(value)
        return value


def _fingerprint(array: np.ndarray, dtype: str) -> str:
    digest = sha256()
    # Bound temporary memory while hashing canonical row order/little-endian data.
    for first in range(0, len(array), 32768):
        digest.update(np.asarray(array[first : first + 32768], dtype=dtype, order="C").tobytes())
    return digest.hexdigest()


def initial_location_from_picks(first: Point3D, second: Point3D) -> InitialLocation:
    """ReceivedPoints' computational prefix, before initialization/dialog handling."""
    with np.errstate(over="ignore", invalid="ignore"):
        angle = float(np.float32(source_angle(first, second)))
    return InitialLocation((first + second) / 2, angle, direction_mode(second, first))


def _state_signature(value):
    if isinstance(value, PointSelection):
        return {"count": len(value), "ordered_ids_sha256": _fingerprint(value.indices, "<i8")}
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    if is_dataclass(value):
        return {field.name: _state_signature(getattr(value, field.name)) for field in fields(value)}
    if isinstance(value, (list, tuple)):
        return [_state_signature(item) for item in value]
    return value


def pipeline_invocations(workflow) -> tuple[tuple[str, TrackingInvocation], ...]:
    if workflow is None:
        return ()
    passes = (
        (("single", workflow),)
        if isinstance(workflow, TrackingPass)
        else (("reverse", workflow.reverse_pass), ("forward", workflow.forward_pass))
    )
    result = []
    for name, current in passes:
        if current is None:
            continue
        result.append((f"{name}.main", current.main))
        result.extend(
            (f"{name}.branches.{i}", branch.tracking)
            for i, branch in enumerate(current.branches)
            if branch.tracking is not None
        )
    return tuple(result)


@dataclass(frozen=True)
class PipelineResult:
    engine: TrackingEngine
    seed: InitialLocation
    options: PipelineOptions
    initialization: InitializationResult
    workflow: TrackingPass | BidirectionalTracking | None
    grades: tuple[GradeAnalysis, ...] | None
    gwad: GWADExport | None
    stages: tuple[PipelineStage, ...]
    findings: tuple[PipelineFinding, ...]
    input_metadata: dict
    random_metadata: dict
    fixture_metadata: dict
    coverage: dict
    status: str

    @property
    def completed(self) -> bool:
        return self.status in ("completed", "completed_with_limitations")

    @property
    def reference_comparable(self) -> bool:
        """No flagged execution policies; this is not whole-binary parity proof."""
        return (
            self.completed
            and self.initialization.status == "ok"
            and self.workflow is not None
            and self.workflow.reference_comparable
        )


def _findings(initialized, workflow, grades) -> tuple[PipelineFinding, ...]:
    result = []

    def add(limitation, status, location):
        result.append(PipelineFinding(limitation, status, location))

    if initialized.status != "ok":
        add(
            "LIM-003"
            if initialized.source_return is None
            else "LIM-006"
            if initialized.status.startswith("nonfinite")
            else None,
            initialized.status,
            "initialization",
        )
    if isinstance(workflow, BidirectionalTracking):
        for label, initialized_again in (
            ("reverse", workflow.reverse_initialization),
            ("forward", workflow.forward_initialization),
        ):
            if initialized_again is not None and initialized_again.status != "ok":
                add(
                    "LIM-003"
                    if initialized_again.source_return is None
                    else "LIM-006"
                    if initialized_again.status.startswith("nonfinite")
                    else None,
                    initialized_again.status,
                    f"{label}.initialization",
                )
        if workflow.reversal is not None and workflow.reversal.status != "ok":
            add("LIM-023", workflow.reversal.status, "reversal")
    for name, invocation in pipeline_invocations(workflow):
        tracking = invocation.result
        if invocation.status.startswith(("undefined_source", "nonfinite")):
            add(
                "LIM-024"
                if "reentry" in invocation.status
                else "LIM-002"
                if "mdl" in invocation.status
                else "LIM-004"
                if "observation_tie" in invocation.status
                else "LIM-006"
                if "gauge" in invocation.status
                else "LIM-007"
                if invocation.status.startswith("nonfinite")
                else "LIM-003",
                invocation.status,
                name,
            )
        if tracking is None:
            continue
        for i, step in enumerate(tracking.steps):
            for rail, model in enumerate(step.models):
                if not model.reference_comparable:
                    add("LIM-002", model.status, f"{name}.steps.{i}.models.{rail}")
            if step.selection is not None and step.selection.status == "unresolved_gauge_tie":
                add("LIM-004", step.selection.status, f"{name}.steps.{i}.selection")
            for attempt_index, attempt in enumerate(step.attempts):
                if (
                    attempt.extraction is not None
                    and attempt.extraction.status == "partial_clustering"
                ):
                    add(
                        "LIM-005",
                        "partial_clustering",
                        f"{name}.steps.{i}.attempts.{attempt_index}",
                    )
        if tracking.recognition is not None:
            for entry in tracking.recognition.entries:
                if entry.status == "ambiguous":
                    add("LIM-020", "ambiguous", f"{name}.recognition.{entry.index}")
        if tracking.toe_detection is not None:
            for entry in tracking.toe_detection.entries:
                if entry.detection is not None and entry.status.startswith(
                    ("undefined_source", "nonfinite")
                ):
                    add(
                        "LIM-001"
                        if "reference_line" in entry.status
                        else "LIM-006"
                        if entry.status.startswith("nonfinite")
                        else "LIM-003",
                        entry.status,
                        f"{name}.toes.{entry.index}",
                    )
        if tracking.switch_box_detection is not None:
            for entry in tracking.switch_box_detection.entries:
                if entry.status.startswith(("undefined_source", "nonfinite")):
                    add(
                        "LIM-025" if entry.status.startswith("undefined_source") else "LIM-027",
                        entry.status,
                        f"{name}.switch_boxes.{entry.candidate_index}",
                    )
            if tracking.switch_box_detection.candidates.status != "ok":
                add(
                    "LIM-010",
                    tracking.switch_box_detection.candidates.status,
                    f"{name}.switch_boxes",
                )
    passes = (
        ()
        if workflow is None
        else (workflow,)
        if isinstance(workflow, TrackingPass)
        else (workflow.reverse_pass, workflow.forward_pass)
    )
    for i, current in enumerate(passes):
        if current is None:
            continue
        for branch in current.branches:
            if branch.status.startswith(("undefined_source", "nonfinite")):
                add("LIM-010", branch.status, f"passes.{i}.branches.{branch.toe_index}")
        for tag in current.tags:
            if not tag.result.reference_comparable:
                add(
                    "LIM-031" if tag.result.status == "inserted_projection" else "LIM-023",
                    tag.result.status,
                    f"passes.{i}.tags.{tag.layer_index}.{tag.toe_index}",
                )
    for i, analysis in enumerate(grades or ()):
        if analysis.status.startswith("nonfinite"):
            add("LIM-029", analysis.status, f"grades.{i}")
    return tuple(result)


def run_pipeline(
    cloud: PointCloud,
    seed: InitialLocation,
    *,
    config: ExtractionConfig | None = None,
    options: PipelineOptions | None = None,
    bounds: Bounds3D | None = None,
    rng: RandSource | None = None,
    supplied_toes: Sequence[ToePoint] = (),
    supplied_stacks: Sequence[TrackStack] = (),
) -> PipelineResult:
    """Run on one owning cloud, mutating shared labels and engine layer state.

    The default follows the bidirectional UI action. Grade/GWAD actions are opt-in
    and defer after a failed/undefined/limited tracking workflow. Supplied state is
    an explicit fixture boundary, never an automatic repair of missing references.
    """
    settings = config if config is not None else ExtractionConfig()
    selected = options if options is not None else PipelineOptions()
    if not isinstance(cloud, PointCloud):
        raise TypeError("wrap XYZ/intensity in PointCloud before running the pipeline")
    if not all(isfinite(v) for v in (*seed.point.as_array(), seed.angle)):
        raise ValueError("pipeline seed coordinates and angle must be finite")
    index(seed.mode)
    for stack in supplied_stacks:
        for segment in stack.track_segments:
            for window in (segment.track1, segment.track2):
                selections = (
                    window.members,
                    window.left_members,
                    window.right_members,
                    window.member_line.members if window.member_line is not None else None,
                )
                if any(points is not None and points.cloud is not cloud for points in selections):
                    raise ValueError("supplied stack memberships must belong to the pipeline cloud")
    grid = GridIndex(cloud, settings.grid_size, bounds=bounds)
    fixtures = {
        "supplied_toes": len(supplied_toes),
        "supplied_stacks": len(supplied_stacks),
        "toe_inputs": [asdict(toe) for toe in supplied_toes],
        "stack_input_sha256": [
            sha256(json.dumps(_state_signature(stack), sort_keys=True).encode()).hexdigest()
            for stack in supplied_stacks
        ],
        "initialized_reference_lines": sum(
            window.member_line is not None
            for stack in supplied_stacks
            for segment in stack.track_segments
            for window in (segment.track1, segment.track2)
        ),
    }
    before = {
        "count": len(cloud),
        "coordinate_units": selected.coordinate_units,
        "crs": selected.crs,
        "bounds": {
            "minimum": grid.bounds.minimum.as_array().tolist(),
            "maximum": grid.bounds.maximum.as_array().tolist(),
        },
        "sha256": {
            "xyz_f64_le": _fingerprint(cloud.xyz, "<f8"),
            "intensity_i32_le": _fingerprint(cloud.intensity, "<i4"),
            "initial_class_codes_i32_le": _fingerprint(cloud.class_codes, "<i4"),
            "initial_segment_ids_i32_le": _fingerprint(cloud.segment_ids, "<i4"),
        },
    }
    source = rng if rng is not None else MsvcRand(selected.random_seed)
    before_state = source.state if isinstance(source, MsvcRand) else None
    random = _CountedRand(source, selected.record_random)
    engine = TrackingEngine(grid, config=settings, rng=random)
    engine.toe_points.extend(replace(toe) for toe in supplied_toes)
    engine.completed_stacks.extend(supplied_stacks)
    initialized = engine.initialize(seed)
    stages = [PipelineStage("grid", "ok"), PipelineStage("initialization", initialized.status)]
    workflow = None
    grades = gwad = None
    if not initialized.success:
        status = (
            initialized.status if initialized.source_return is None else "initialization_failed"
        )
        stages.append(PipelineStage("tracking", "not_run_initialization_failed"))
    else:
        workflow = (
            engine.run_bidirectional(max_steps=selected.max_steps)
            if selected.bidirectional
            else engine.run_pass(max_steps=selected.max_steps)
        )
        status = workflow.status
        stages.append(PipelineStage("tracking", status))
    if status == "completed":
        if selected.grades:
            grades = engine.compute_grades()
            stages.append(PipelineStage("grades", "computed"))
        else:
            stages.append(PipelineStage("grades", "not_requested"))
        if selected.gwad:
            gwad = build_gwad_rows(engine.layers, [f"layer-{i}" for i in range(len(engine.layers))])
            if grades is None:
                grades = gwad.analyses
            stages.append(PipelineStage("gwad", "computed"))
        else:
            stages.append(PipelineStage("gwad", "not_requested"))
    else:
        for name, requested in (("grades", selected.grades), ("gwad", selected.gwad)):
            stages.append(PipelineStage(name, "deferred" if requested else "not_requested"))
    findings = _findings(initialized, workflow, grades)
    if status == "completed" and (findings or not workflow.reference_comparable):
        status = "completed_with_limitations"
    random_metadata = {
        "generator": f"{type(source).__module__}.{type(source).__qualname__}",
        "seed": selected.random_seed if rng is None else None,
        "initial_state": before_state,
        "final_state": source.state if isinstance(source, MsvcRand) else None,
        "draw_count": random.draw_count,
        "recorded_draws": tuple(random.draws) if selected.record_random else None,
    }
    return PipelineResult(
        engine,
        seed,
        selected,
        initialized,
        workflow,
        grades,
        gwad,
        tuple(stages),
        findings,
        before,
        random_metadata,
        fixtures,
        coverage_summary(),
        status,
    )
