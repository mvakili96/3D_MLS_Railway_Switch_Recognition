"""Headless source-based orchestration over a verified mapped scene.

This module deliberately consumes only inference definitions. Reference files,
published values and saved original predictions never enter this workflow.
"""

from collections import Counter
import csv
from dataclasses import asdict
from hashlib import sha256
import json
from operator import index
from pathlib import Path
import time

import numpy as np

from ..artifacts import strict_json_value
from ..models import InitialLocation
from ..orchestration import TrackingEngine
from ..pipeline import pipeline_invocations
from ..points import Point3D
from ..sampling import MsvcRand
from .cache import open_scene_cache
from .catalog import _hash_file
from .manifest import load_inference_spec
from .resources import ResourceLimitError, check_disk, resident_bytes


def _write_report(path, value):
    """Replace only our own status snapshot inside the new output directory."""
    path = Path(path)
    temporary = path.with_suffix(path.suffix+".tmp")
    temporary.write_text(json.dumps(strict_json_value(value), indent=2, allow_nan=False)+"\n")
    temporary.replace(path)


def _point(point):
    return None if point is None else [float(point.x), float(point.y), float(point.z)]


def _initialization(initialized):
    if initialized is None:
        return None
    return {"status": initialized.status, "source_return": initialized.source_return,
            "initial_heights": initialized.initial_heights,
            "fitted_location": None if initialized.location is None else {
                "xyz": _point(initialized.location.point), "angle": initialized.location.angle,
                "mode": initialized.location.mode},
            "head_counts": [len(points) for points in initialized.heads]}


def _invocations(workflow):
    result = []
    for name, invocation in pipeline_invocations(workflow):
        current = invocation.result
        row = {"name": name, "layer": invocation.layer_index, "direction": invocation.direction,
               "main_track": invocation.main_track, "source_stop": invocation.status,
               "reference_comparable": invocation.reference_comparable}
        if current is not None:
            row.update({"attempted_steps": current.attempted_steps, "accepted_steps": current.accepted_steps,
                        "retained_diagnostic_steps": len(current.steps), "history_complete": current.history_complete,
                        "distance_from_start": current.distance_from_start,
                        "next_heights": current.next_heights,
                        "model_status_counts": dict(current.model_status_counts),
                        "diagnostic_counts": dict(current.diagnostic_counts),
                        "recognition": [] if current.recognition is None else [
                            {"index": e.index, "status": e.status, "type": e.binary.type,
                             "score": e.binary.score, "segments": e.segment_count}
                            for e in current.recognition.entries],
                        "toes": [] if current.toe_detection is None else [
                            {"index": e.index, "status": e.status, "segments": e.segment_count,
                             "score": e.score} for e in current.toe_detection.entries],
                        "switch_boxes": [] if current.switch_box_detection is None else [
                            {"status": e.status} for e in current.switch_box_detection.entries]})
        result.append(row)
    return result


def _limitations(invocations, initialization, status):
    ids = set()
    if initialization is not None and initialization["status"] != "ok":
        ids.add("LIM-003")
    if status.startswith("nonfinite"):
        ids.add("LIM-006")
    if status.startswith("undefined_source"):
        ids.add("LIM-003")
    for invocation in invocations:
        for finding in invocation.get("diagnostic_counts", {}):
            ids.add("LIM-002" if finding.startswith("mdl:") else
                    "LIM-004" if finding.startswith("selection:") else "LIM-005")
        for toe in invocation.get("toes", []):
            if "reference_line" in toe["status"]:
                ids.add("LIM-001")
            elif toe["status"].startswith(("undefined_source", "nonfinite")):
                ids.add("LIM-003")
        if any(e["status"].startswith(("undefined_source", "nonfinite"))
               for e in invocation.get("switch_boxes", [])):
            ids.add("LIM-025")
        if invocation.get("source_stop", "").startswith("undefined_source"):
            ids.add("LIM-024" if "reentry" in invocation["source_stop"] else "LIM-003")
    return sorted(ids)


class _Journal:
    """Stream compact events; only last-step/per-invocation scalar metadata stays live."""

    def __init__(self, stream, cache, scene, root, progress, run_id, rng):
        self.stream, self.cache, self.scene, self.root = stream, cache, scene, root
        self.progress, self.run_id, self.rng = progress, run_id, rng
        self.traces, self.last = [], None
        self.tile_ends = np.array([t["row_end"] for t in cache.metadata["tiles"]], dtype=np.int64)
        self.cache_bytes = (sum(f["bytes"] for f in cache.metadata["files"].values())
                            + (cache.path/"cache.json").stat().st_size)

    def resources(self, extra=0):
        self.scene.cloud.memory.require(location="prediction boundary", reclaim=self.scene.cloud.release_pages)
        used = sum(p.stat().st_size for p in self.root.rglob("*") if p.is_file())
        if self.cache_bytes+used+extra > self.cache.disk_bytes:
            raise ResourceLimitError("cache, label state and prediction artifacts exceed disk budget")
        check_disk(self.root, max(extra, 4096), self.cache.disk_bytes)

    def __call__(self, layer, direction, main_track, step):
        if step.number == 1:
            self.traces.append({"sequence": len(self.traces), "layer": layer, "direction": direction,
                                "main_track": main_track, "attempted_steps": 0, "accepted_steps": 0,
                                "support_tiles": {}, "accepted_centers_beyond_first_tile": 0,
                                "diagnostic_counts": {}})
        trace = self.traces[-1]
        trace["attempted_steps"] += 1
        trace["accepted_steps"] += int(step.accepted)
        trace["last_source_status"] = step.status
        findings = ["mdl:"+m.status for m in step.models if not m.reference_comparable]
        if step.selection is not None and step.selection.status == "unresolved_gauge_tie":
            findings.append("selection:"+step.selection.status)
        findings.extend("extraction:partial_clustering" for a in step.attempts
                        if a.extraction is not None and a.extraction.status == "partial_clustering")
        for finding in findings:
            trace["diagnostic_counts"][finding] = trace["diagnostic_counts"].get(finding, 0)+1
        support_tiles = Counter()
        if step.accepted:
            for fit in step.fits:
                ids = np.searchsorted(self.tile_ends, fit.support.indices, side="right")
                unique, counts = np.unique(ids, return_counts=True)
                for tile, count in zip(unique, counts):
                    support_tiles[int(tile)] += int(count)
            for tile, count in support_tiles.items():
                key = str(tile)
                trace["support_tiles"][key] = trace["support_tiles"].get(key, 0)+count
            center = step.candidate.center.as_array()
            boxes = [t["bounds"] for t in self.cache.metadata["tiles"]]
            contains = [all(b["minimum"][i] <= center[i] <= b["maximum"][i] for i in (0, 1)) for b in boxes]
            trace["accepted_centers_beyond_first_tile"] += int(not contains[0] and any(contains[1:]))
        windows = ([] if not step.attempts else
                   [[_point(p) for p in w.vertices]
                    for w in (step.attempts[-1].segment.track1, step.attempts[-1].segment.track2)])
        boundary = None
        if step.status in ("insufficient_window_points", "insufficient_head_points", "no_support_points") and windows:
            vertices = [p for window in windows for p in window]
            b = self.cache.metadata["bounds"]
            outside = any(p[i] < b["minimum"][i] or p[i] > b["maximum"][i] for p in vertices for i in (0, 1))
            overlapping = []
            for tile_index, tile in enumerate(self.cache.metadata["tiles"]):
                box = tile["bounds"]
                if any(window and all(min(p[i] for p in window) <= box["maximum"][i]
                                      and max(p[i] for p in window) >= box["minimum"][i]
                                      for i in (0, 1)) for window in windows):
                    overlapping.append(tile_index)
            boundary = ("scene_header_edge" if outside else "scene_footprint_gap" if not overlapping
                        else "search_overlaps_tile_footprints")
        row = {"kind": "step", "invocation": trace["sequence"], "layer": layer, "direction": direction,
               "main_track": main_track, "number": step.number, "accepted": step.accepted,
               "source_status": step.status,
               "center": _point(step.candidate.center) if step.candidate is not None else None,
               "model_statuses": [m.status for m in step.models],
               "model_line_counts": [len(m.lines) for m in step.models],
               "attempts": [{"phase": a.phase, "length": a.length,
                             "point_counts": [len(w.members) if w.members is not None else 0
                                              for w in (a.segment.track1, a.segment.track2)],
                             "extraction": a.extraction.status if a.extraction is not None else None}
                            for a in step.attempts],
               "last_windows": windows, "boundary_hint": boundary,
               "fit_support_tiles": dict(sorted(support_tiles.items())),
               "rng_state": self.rng.state, "random_draws": self.rng.draw_count}
        self.last = row
        self.stream.write(json.dumps(strict_json_value(row), allow_nan=False)+"\n")
        self.stream.flush()
        if self.progress:
            self.progress(f"{self.run_id}: invocation {trace['sequence']} step {step.number}: {step.status}")
        self.resources()


def _write_geometry(directory, track_layers):
    layers = []
    with (directory/"vectors.csv").open("x", newline="") as vectors, (directory/"segments.csv").open("x", newline="") as segments:
        vertices, lines = csv.writer(vectors), csv.writer(segments)
        vertices.writerow(("layer", "vector", "vertex", "x", "y", "z", "point_type"))
        lines.writerow(("layer", "vector", "segment", "begin_x", "begin_y", "begin_z", "end_x", "end_y", "end_z"))
        for i, layer in enumerate(track_layers):
            sizes = {}
            for name, vector in (("rail1", layer.rail1_vector), ("rail2", layer.rail2_vector), ("center", layer.center_vector)):
                sizes[name] = len(vector)
                for j, point in enumerate(vector):
                    vertices.writerow((i, name, j, point.x, point.y, point.z, point.point_type))
                    if j:
                        lines.writerow((i, name, j-1, *_point(vector[j-1]), *_point(point)))
            layers.append({"layer": i, "vertices": sizes, "source_segment_count": len(layer.track_segments),
                           "empty": not any(sizes.values())})
    return layers


def _one_run(cache, spec, run, directory, root, *, retain_history, cap, progress):
    directory.mkdir(exist_ok=False)
    started = time.monotonic()
    rng, engine, initialized, workflow, journal = MsvcRand(run.random_seed), None, None, None, None
    status, failure, invocations, layers = "initializing", None, [], []
    metadata = {"schema_version": 1, "run_id": run.run_id, "status": status,
                "seed": asdict(run.seed), "seed_provenance": run.seed_provenance,
                "direction": run.direction, "branch_policy": run.branch_policy,
                "initialization_protocol": "direct_supplied_seed_once_then_source_orchestration",
                "max_steps_per_invocation": cap, "retain_history": retain_history,
                "cache_manifest_sha256": cache.metadata["manifest_sha256"],
                "source_baseline": spec.source_baseline, "config": asdict(spec.config),
                "reference_files_read": False, "evaluation_performed": False, "full_reference_coverage": None}
    _write_report(directory/"manifest.json", metadata)
    try:
        with cache.new_run(directory/"state") as scene:
            with (directory/"steps.jsonl").open("x", encoding="utf-8", buffering=1) as stream:
                journal = _Journal(stream, cache, scene, root, progress, run.run_id, rng)
                engine = TrackingEngine(scene.grid, config=spec.config, rng=rng, retain_history=retain_history,
                                        step_callback=journal)
                try:
                    journal.resources()
                    seed = InitialLocation(Point3D(*run.seed.xyz), run.seed.angle, run.seed.mode)
                    initialized = engine.initialize(seed)
                    if progress:
                        progress(f"{run.run_id}: initialization {initialized.status}")
                    if initialized.success:
                        workflow = (engine.run_bidirectional(max_steps=cap) if run.direction == "bidirectional"
                                    else engine.run_pass(max_steps=cap))
                        status = workflow.status
                    else:
                        status = initialized.status if initialized.source_return is None else "initialization_failed"
                    journal.resources()
                except (ResourceLimitError, MemoryError, OSError, ValueError, RuntimeError) as error:
                    status = "resource_limited" if isinstance(error, (ResourceLimitError, MemoryError)) else "run_failed"
                    failure = {"type": type(error).__name__, "error": str(error)}
                except KeyboardInterrupt:
                    status, failure = "interrupted", {"error": "interrupted by user"}
                invocations = _invocations(workflow)
                layers = _write_geometry(directory, engine.layers if engine is not None else ())
                resources = {"observed_peak_rss_bytes": scene.cloud.memory.observed_peak,
                             "rss_at_export_bytes": resident_bytes()}
    except (ResourceLimitError, MemoryError, OSError, ValueError, RuntimeError) as error:
        status = "resource_limited" if isinstance(error, (ResourceLimitError, MemoryError)) else "run_failed"
        failure = {"type": type(error).__name__, "error": str(error)}
        resources = {"rss_at_failure_bytes": resident_bytes()}
        if not (directory/"vectors.csv").exists():
            layers = _write_geometry(directory, engine.layers if engine is not None else ())
    except KeyboardInterrupt:
        status, failure = "interrupted", {"error": "interrupted by user"}
        resources = {"rss_at_failure_bytes": resident_bytes()}
        if not (directory/"vectors.csv").exists():
            layers = _write_geometry(directory, engine.layers if engine is not None else ())
    initialization = _initialization(initialized)
    limitations = _limitations(invocations+(journal.traces if journal else []), initialization, status)
    comparable = workflow is not None and workflow.reference_comparable and not limitations
    if status == "completed" and not comparable:
        status = "completed_with_limitations"
    metadata.update({"status": status, "failure": failure, "initialization": initialization,
                     "invocations": invocations, "layers": layers, "limitations_encountered": limitations,
                     "reference_comparable": comparable,
                     "source_state_policy": "retain_source_layers_stacks_and_supports_without_pruning",
                     "postpass_summary_complete": workflow is not None, "resume_supported": False,
                     "random": {"generator": "MSVC", "seed": run.random_seed, "initial_state": run.random_seed,
                                "final_state": rng.state, "draw_count": rng.draw_count},
                     "journal_invocations": [] if journal is None else journal.traces,
                     "last_step": None if journal is None else journal.last,
                     "resources": resources, "elapsed_seconds": time.monotonic()-started,
                     "diagnostic_limit_reached": status == "step_limit",
                     "coverage": {"accepted_centers_beyond_first_tile":
                         sum(t["accepted_centers_beyond_first_tile"] for t in journal.traces) if journal else 0,
                         "interpretation": "Header-based geometry/support diagnostics; reference coverage not evaluated."}})
    files = {}
    for name in ("vectors.csv", "segments.csv", "steps.jsonl", "state/class_codes.bin", "state/segment_ids.bin"):
        path = directory/name
        if path.is_file():
            files[name] = {"bytes": path.stat().st_size, "sha256": _hash_file(path)}
    metadata["artifacts"] = files
    _write_report(directory/"manifest.json", metadata)
    return metadata


def run_predictions(manifest, cache_path, output, *, run_ids=None, data_root=None,
                    retain_history=False, max_steps=None, progress=None):
    """Attempt selected ready runs, preserving partial vectors and source stops."""
    output = Path(output)
    if output.exists():
        raise FileExistsError(output)
    if output.resolve().is_relative_to(Path(cache_path).resolve()):
        raise ValueError("prediction outputs must be outside the immutable cache directory")
    spec = load_inference_spec(manifest, data_root=data_root)
    if max_steps is not None:
        max_steps = index(max_steps)
        if max_steps < 0:
            raise ValueError("max_steps must be nonnegative")
    known = {run.run_id: run for run in spec.runs}
    if run_ids is not None:
        if not run_ids or len(set(run_ids)) != len(run_ids) or set(run_ids)-known.keys():
            raise ValueError("select unique declared run IDs")
        pending = [r for r in run_ids if known[r].status != "ready"]
        if pending:
            raise ValueError(f"requested runs need initialization: {pending}")
    selected = [r for r in spec.runs if r.status == "ready" and (run_ids is None or r.run_id in run_ids)]
    if not selected:
        raise ValueError("no ready prediction runs")
    if any(r.branch_policy != "strict" for r in selected):
        raise ValueError("headless source profile requires strict branches; viewer reference recovery is separate")
    cache = open_scene_cache(cache_path, inference=spec)
    output.mkdir(parents=True, exist_ok=False)
    report = {"schema_version": 1, "kind": "headless_prediction_run", "status": "running",
              "manifest": str(Path(manifest).resolve()),
              "manifest_sha256": sha256(Path(manifest).read_bytes()).hexdigest(),
              "cache": str(cache.path.resolve()), "cache_manifest_sha256": cache.metadata["manifest_sha256"],
              "compatibility_fingerprint": cache.metadata["compatibility_fingerprint"],
              "pending_seed_runs": [r.run_id for r in spec.runs if r.status == "pending"],
              "selected_runs": [r.run_id for r in selected], "runs": [],
              "max_steps_override": max_steps, "reference_files_read": False,
              "evaluation_performed": False, "nine_path_complete": False}
    _write_report(output/"manifest.json", report)
    cache_bytes = (sum(f["bytes"] for f in cache.metadata["files"].values())
                   + (cache.path/"cache.json").stat().st_size)
    for run in selected:
        # Admission for retained states from earlier independent runs, not just
        # the one-state cache-backend estimate.
        used = sum(p.stat().st_size for p in output.rglob("*") if p.is_file())
        if cache_bytes+used+cache.metadata["point_count"]*8+4096 > cache.disk_bytes:
            report["runs"].append({"run_id": run.run_id, "status": "resource_limited",
                                   "failure": {"error": "retained states exceed experiment disk budget"}})
        else:
            result = _one_run(cache, spec, run, output/run.run_id, output, retain_history=retain_history,
                              cap=run.max_steps if max_steps is None else max_steps, progress=progress)
            report["runs"].append(result)
        _write_report(output/"manifest.json", report)
        if report["runs"][-1]["status"] == "interrupted":
            break
    statuses = [r["status"] for r in report["runs"]]
    report["status"] = ("partial" if any(s not in ("completed", "completed_with_limitations") for s in statuses)
                        else "completed_with_limitations" if "completed_with_limitations" in statuses
                        else "completed")
    _write_report(output/"manifest.json", report)
    return report
