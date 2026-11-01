"""Versioned diagnostic JSON/CSV, separate from the pending legacy I/O formats."""

from collections import Counter
from dataclasses import asdict, fields, is_dataclass
import csv
from hashlib import sha256
from importlib.metadata import PackageNotFoundError, version
import json
from math import isfinite, isnan
from pathlib import Path
import platform

import numpy as np

from .grades import format_gwad
from .orchestration import BidirectionalTracking
from .pipeline import PipelineResult, pipeline_invocations
from .points import PointSelection


def _plain(value):
    if isinstance(value, PointSelection):
        return {"count": len(value), "point_ids": value.indices.tolist()}
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    if is_dataclass(value):
        return {field.name: _plain(getattr(value, field.name)) for field in fields(value)}
    if isinstance(value, dict):
        return {key: _plain(item) for key, item in value.items()}
    if isinstance(value, (tuple, list)):
        return [_plain(item) for item in value]
    return value


def strict_json_value(value) -> dict:
    """Encode IEEE values as null plus JSON-pointer annotations, retaining None."""
    nonfinite = []

    def visit(item, path):
        if isinstance(item, float) and not isfinite(item):
            nonfinite.append(
                {
                    "path": path,
                    "value": "NaN" if isnan(item) else "+Infinity" if item > 0 else "-Infinity",
                }
            )
            return None
        if isinstance(item, dict):
            return {
                key: visit(child, path + "/" + str(key).replace("~", "~0").replace("/", "~1"))
                for key, child in item.items()
            }
        if isinstance(item, list):
            return [visit(child, f"{path}/{i}") for i, child in enumerate(item)]
        return item

    result = visit(_plain(value), "")
    result["nonfinite_values"] = nonfinite
    return result


def _invocation_summary(name, invocation, include_steps):
    tracking = invocation.result
    result = {
        "name": name,
        "status": invocation.status,
        "layer_index": invocation.layer_index,
        "direction": invocation.direction,
        "main_track": invocation.main_track,
        "reference_comparable": invocation.reference_comparable,
    }
    if tracking is None:
        return result
    result.update(
        {
            "step_count": len(tracking.steps),
            "accepted_steps": tracking.accepted_steps,
            "distance_from_start": tracking.distance_from_start,
            "next_heights": tracking.next_heights,
            "model_status_counts": dict(
                Counter(model.status for step in tracking.steps for model in step.models)
            ),
            "completed_stack_count": len(tracking.completed_stacks),
            "active_stack_segments": len(tracking.active_stack.track_segments),
            "recognition": _plain(tracking.recognition),
            "toes": None,
            "switch_boxes": None,
        }
    )
    if tracking.toe_detection is not None:
        batch = tracking.toe_detection
        result["toes"] = {
            "main_track": batch.main_track,
            "new_toes": _plain(batch.new_toes),
            "entries": [
                {
                    "index": entry.index,
                    "status": entry.status,
                    "segment_count": entry.segment_count,
                    "score": entry.score,
                    "source_return": entry.detection.source_return if entry.detection else None,
                    "candidate_count": len(entry.detection.candidates)
                    if entry.detection and entry.detection.candidates is not None
                    else 0,
                    "fit_status": entry.detection.fit.status
                    if entry.detection and entry.detection.fit is not None
                    else None,
                }
                for entry in batch.entries
            ],
        }
    if tracking.switch_box_detection is not None:
        batch = tracking.switch_box_detection
        result["switch_boxes"] = {
            "status": batch.status,
            "candidate_status": batch.candidates.status,
            "candidate_count": len(batch.candidates.candidates),
            "new_box_count": len(batch.new_boxes),
            "entries": [
                {
                    "candidate_index": entry.candidate_index,
                    "status": entry.status,
                    "region_count": len(entry.region_points),
                    "height_count": len(entry.height_points),
                    "first_fit_status": entry.first_fit.status if entry.first_fit else None,
                    "second_fit_status": entry.second_fit.status if entry.second_fit else None,
                }
                for entry in batch.entries
            ],
        }
    if include_steps:
        result["steps"] = _plain(tracking.steps)
    return result


def _initialization_summary(initialized):
    if initialized is None:
        return None
    return {
        "status": initialized.status,
        "source_return": initialized.source_return,
        "initial_heights": initialized.initial_heights,
        "location": _plain(initialized.location),
        "head_counts": [len(points) for points in initialized.heads],
        "classifications": [
            {
                "fit": _plain(classification.fit),
                "cluster_counts": [len(cluster) for cluster in classification.clustering.clusters],
                "failed_position": classification.clustering.failed_position,
            }
            for classification in initialized.classifications
        ],
        "fits": [
            {"status": fit.status, "support_count": len(fit.support)} for fit in initialized.fits
        ],
    }


def pipeline_summary(
    result: PipelineResult, *, include_steps: bool = False, context: dict | None = None
) -> dict:
    """Snapshot live results now; stable metadata and no cloud XYZ dump by default."""
    try:
        package_version = version("railway-mls")
    except PackageNotFoundError:
        package_version = "uninstalled"
    engine = result.engine
    class_values, class_counts = np.unique(engine.grid.cloud.class_codes, return_counts=True)
    passes = (
        ()
        if result.workflow is None
        else (("reverse", result.workflow.reverse_pass), ("forward", result.workflow.forward_pass))
        if isinstance(result.workflow, BidirectionalTracking)
        else (("single", result.workflow),)
    )
    payload = {
        "schema_version": 1,
        "status": result.status,
        "completed": result.completed,
        "context": context if context is not None else {},
        "reference_comparable": result.reference_comparable,
        "validation_scope": "Known execution flags only; whole-application parity and real-data accuracy remain unverified.",
        "runtime": {
            "package": package_version,
            "python": platform.python_version(),
            "numpy": np.__version__,
        },
        "config": engine.config.to_dict(),
        "options": asdict(result.options),
        "seed": _plain(result.seed),
        "input": result.input_metadata,
        "random": result.random_metadata,
        "supplied_fixtures": result.fixture_metadata,
        "source_coverage": result.coverage,
        "stages": _plain(result.stages),
        "findings": _plain(result.findings),
        "limitations_encountered": sorted(
            {item.limitation for item in result.findings if item.limitation}
        ),
        "initialization": _initialization_summary(result.initialization),
        "bidirectional": {
            "original_location": _plain(result.workflow.original_location),
            "reverse_location": _plain(result.workflow.reverse_location),
            "reverse_initialization": _initialization_summary(
                result.workflow.reverse_initialization
            ),
            "forward_initialization": _initialization_summary(
                result.workflow.forward_initialization
            ),
            "reversal": _plain(result.workflow.reversal),
        }
        if isinstance(result.workflow, BidirectionalTracking)
        else None,
        "passes": [
            {
                "name": name,
                "status": current.status,
                "branches": [
                    {
                        "toe_index": branch.toe_index,
                        "track_type": branch.toe.track_type,
                        "status": branch.status,
                        "layer_index": branch.layer_index,
                        "seed": _plain(branch.seed),
                        "local_used": branch.toe.used,
                        "initialization_status": branch.initialization.status
                        if branch.initialization
                        else None,
                    }
                    for branch in current.branches
                ],
                "center_tags": _plain(current.tags),
            }
            for name, current in passes
            if current is not None
        ],
        "invocations": [
            _invocation_summary(name, invocation, include_steps)
            for name, invocation in pipeline_invocations(result.workflow)
        ],
        "layers": [
            {
                "index": i,
                "number": layer.number,
                "track_segments": len(layer.track_segments),
                "rail1": _plain(layer.rail1_vector),
                "rail2": _plain(layer.rail2_vector),
                "center": _plain(layer.center_vector),
                "toes": _plain(layer.toe_points),
                "switch_boxes": _plain(layer.switch_boxes),
            }
            for i, layer in enumerate(engine.layers)
        ],
        "global_pools": {
            "completed_stack_count": len(engine.completed_stacks),
            "toes": _plain(engine.toe_points),
            "switch_boxes": _plain(engine.switch_boxes),
        },
        "final_class_counts": {
            str(int(key)): int(count) for key, count in zip(class_values, class_counts)
        },
        "grades": [
            {
                "layer_index": i,
                "status": analysis.status,
                "division": _plain(analysis.division),
                "sections": _plain(analysis.sections),
                "report": _plain(analysis.report),
            }
            for i, analysis in enumerate(result.grades or ())
        ],
        "gwad": {"rows": _plain(result.gwad.rows)} if result.gwad is not None else None,
    }
    return strict_json_value(payload)


def write_pipeline_outputs(
    result: PipelineResult,
    directory: str | Path,
    *,
    include_steps: bool = False,
    context: dict | None = None,
) -> Path:
    """Create a new run directory; existing paths are never overwritten.

    CSV uses full Python float representations and IEEE nan/inf text. JSON adds
    explicit nonfinite paths. Files are diagnostics, not legacy export adapters.
    No input XYZ/intensity or point-label array is exported.
    """
    path = Path(directory)
    report = pipeline_summary(result, include_steps=include_steps, context=context)
    path.mkdir(parents=True, exist_ok=False)
    files = []

    def table(name, header, rows):
        with (path / name).open("w", newline="", encoding="utf-8") as stream:
            writer = csv.writer(stream)
            writer.writerow(header)
            writer.writerows(rows)
        files.append(name)

    def coordinates(point):
        return (None, None, None, None) if point is None else (*point.as_array(), point.point_type)

    table(
        "vectors.csv",
        ("layer", "vector", "vertex", "x", "y", "z", "point_type"),
        (
            (i, name, j, *coordinates(point))
            for i, layer in enumerate(result.engine.layers)
            for name, vector in (
                ("rail1", layer.rail1_vector),
                ("rail2", layer.rail2_vector),
                ("center", layer.center_vector),
            )
            for j, point in enumerate(vector)
        ),
    )
    table(
        "toes.csv",
        ("toe", "track_type", "used", "angle", "mode", "point", "x", "y", "z", "point_type"),
        (
            (i, toe.track_type, toe.used, toe.angle, toe.mode, name, *coordinates(point))
            for i, toe in enumerate(result.engine.toe_points)
            for name, point in (("center", toe.center), ("left", toe.left), ("right", toe.right))
        ),
    )
    table(
        "switch_boxes.csv",
        ("box", "x", "y", "z", "angle", "reference_height"),
        (
            (i, *box.center.as_array(), box.angle, box.reference_height)
            for i, box in enumerate(result.engine.switch_boxes)
        ),
    )
    table(
        "grades.csv",
        (
            "layer",
            "section",
            "begin_x",
            "begin_y",
            "begin_z",
            "end_x",
            "end_y",
            "end_z",
            "arc",
            "chord",
            "height",
            "grade",
            "radius",
            "degree_of_curve",
            "grade_with_curve",
        ),
        (
            (
                i,
                j,
                *section.begin.as_array(),
                *section.end.as_array(),
                section.arc,
                section.chord,
                section.height,
                section.grade,
                section.radius,
                section.degree_of_curve,
                section.grade_with_curve,
            )
            for i, analysis in enumerate(result.grades or ())
            for j, section in enumerate(analysis.sections)
        ),
    )
    if result.gwad is not None:
        (path / "gwad.tsv").write_text(format_gwad(result.gwad), encoding="utf-8")
        files.append("gwad.tsv")
    report["artifacts"] = [
        {"path": name, "sha256": sha256((path / name).read_bytes()).hexdigest()} for name in files
    ]
    manifest = path / "manifest.json"
    manifest.write_text(json.dumps(report, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    return manifest
