"""Explore supported rail continuations over the verified prediction scene."""

from collections import deque
from dataclasses import asdict, replace
from hashlib import sha256
from itertools import pairwise
from operator import index
from pathlib import Path
from tempfile import TemporaryDirectory

from ..branching import (
    CONFIRM_LIMIT,
    CONFIRM_WINDOWS,
    REJOIN_WINDOWS,
    BranchProgress,
    TraceCoverage,
    accepted_rails,
    outgoing_starts,
)
from ..continuations import ContinuationTracker
from ..evaluation.io import file_fingerprint
from ..geometry import distance_2d
from ..models import InitialLocation
from ..orchestration import TrackingEngine
from ..points import Point3D
from ..sampling import MsvcRand
from .cache import open_scene_cache
from .manifest import load_inference_spec
from .runner import (
    _initialization,
    _invocations,
    _Journal,
    _layer_from_rail_centers,
    _point,
    _write_geometry,
    _write_report,
)


def _failure(error):
    return {"type": type(error).__name__, "message": str(error)}


def _trace_source(template, config, run, output, cap, coverage, queue, progress):
    directory = output / "source"
    directory.mkdir()
    rng = MsvcRand(run.random_seed)
    positions = {}
    initialized = workflow = None
    error = None
    with (
        template.private_run() as scene,
        (directory / "steps.jsonl").open("x", encoding="utf-8") as stream,
    ):
        journal = _Journal(stream, template.cache, scene, output, None, run.run_id, rng)

        def observe(layer, direction, main_track, step):
            journal(layer, direction, main_track, step)
            sequence = journal.last["invocation"]
            route = f"source-{sequence:02d}"
            if step.number == 1:
                positions[route] = [tuple(state.point for state in step.current_states)]
            rails = accepted_rails(step)
            if rails is not None:
                positions[route].append(rails)
            queue.extend(outgoing_starts(step, config, route, rng.state))
            if progress and step.number % 25 == 0:
                progress(f"{route}: window {step.number}, {len(queue)} candidate starts")

        engine = TrackingEngine(
            scene.grid, config=config, rng=rng, retain_history=False, step_callback=observe
        )
        try:
            journal.resources()
            seed = InitialLocation(Point3D(*run.seed.xyz), run.seed.angle, run.seed.mode)
            initialized = engine.initialize(seed)
            status = initialized.status
            if initialized.success:
                workflow = (engine.run_bidirectional(max_steps=cap) if run.direction == "bidirectional"
                            else engine.run_pass(max_steps=cap))
                status = workflow.status
        except (OSError, ValueError, RuntimeError, MemoryError) as exception:
            status, error = getattr(exception, "status", "failed"), _failure(exception)
        layers = _write_geometry(directory, engine.layers)
    accepted = directory / "accepted"
    accepted.mkdir()
    _write_geometry(accepted, (_layer_from_rail_centers(rails) for rails in positions.values()))
    for route, rails in positions.items():
        coverage.add(route, rails)
    return {
        "status": status,
        "error": error,
        "initialization": _initialization(initialized),
        "invocations": _invocations(workflow),
        "journal_routes": [
            {"route": route, "invocation": sequence, "accepted_steps": len(rails) - 1,
             "layer": journal.traces[sequence]["layer"], "accepted_layer": sequence}
            for sequence, (route, rails) in enumerate(positions.items())
        ],
        "layers": layers,
        "random_state": rng.state,
        "random_draws": rng.draw_count,
        "artifacts": {
            name: file_fingerprint(directory / name)
            for name in ("steps.jsonl", "vectors.csv", "accepted/vectors.csv")
        },
    }


def _trace_branch(template, config, start, output, route, cap, coverage):
    directory = output / route
    directory.mkdir()
    pair = start.pair
    layer = pair.seed_layer(config)
    rails = layer.rail1_vector[0], layer.rail2_vector[0]
    positions = [rails]
    rng = MsvcRand(start.random_state)
    discoveries, monitor = [], BranchProgress()
    error = None
    attempted = 0
    with (
        template.private_run() as scene,
        (directory / "steps.jsonl").open("x", encoding="utf-8") as stream,
    ):
        journal = _Journal(stream, template.cache, scene, output, None, route, rng)

        def observe(step):
            fitted = accepted_rails(step)
            if fitted is not None:
                match = coverage.match(positions[-1], fitted)
                positions.append(fitted)
                stopped = monitor.observe(step.number, match)
                if stopped is not None:
                    tracker.status = stopped
            if monitor.rejoin_at is None:
                discoveries.extend(outgoing_starts(
                    step, config, route, rng.state, preceding_steps=start.elapsed_steps,
                ))
            journal(0, 0, False, step)

        tracker = ContinuationTracker(
            layer, scene.grid, (rails[0].z, rails[1].z), config=config, rng=rng,
            main_track=False, retain_history=False, step_callback=observe,
        )
        try:
            journal.resources()
            result = tracker.run(max_steps=cap)
            status, attempted = result.status, result.attempted_steps
        except (OSError, ValueError, RuntimeError, MemoryError) as exception:
            status, error = getattr(exception, "status", "failed"), _failure(exception)
            attempted = tracker.attempted_count

        accepted = len(positions) - 1
        if monitor.rejoin_at is not None:
            positions = positions[:monitor.rejoin_at + 1]
        count = len(positions)
        accepted_layer = replace(
            layer,
            track_segments=layer.track_segments[:count],
            rail1_vector=layer.rail1_vector[:count],
            rail2_vector=layer.rail2_vector[:count],
            center_vector=layer.center_vector[:count],
        )
        layers = _write_geometry(directory, [accepted_layer])
    if monitor.confirmed_at is not None:
        coverage.add(route, positions)
        if monitor.rejoin_at is not None:
            discoveries = [candidate for candidate in discoveries if candidate.step < count]
    else:
        discoveries.clear()
    centers = accepted_layer.center_vector
    return {
        "route": route,
        "parent": start.parent,
        "parent_window": start.step,
        "steps_before_branch": start.elapsed_steps,
        "pair_indices": pair.indices,
        "support_counts": pair.support_counts,
        "seed_gauge_m": pair.gauge,
        "seed_begin_xyz": [_point(point) for point in pair.begins],
        "seed_end_xyz": [_point(point) for point in pair.ends],
        "status": status,
        "error": error,
        "confirmed_at": monitor.confirmed_at,
        "rejoin": None if monitor.rejoin_at is None else {
            "window": monitor.rejoin_at, **asdict(monitor.rejoin),
        },
        "attempted_steps": attempted,
        "accepted_steps": accepted,
        "exported_steps": count - 1,
        "length_xy_m": sum(distance_2d(a, b) for a, b in pairwise(centers)),
        "initial_random_state": start.random_state,
        "random_state": rng.state,
        "random_draws": rng.draw_count,
        "layers": layers,
        "artifacts": {"vectors.csv": file_fingerprint(directory / "vectors.csv")},
    }, discoveries


def track_branches(manifest, cache_path, output, *, run_id, max_steps=None, max_branch_attempts=None,
                   data_root=None, progress=None):
    """Preserve the source traces and explore detected alternatives in queue order."""
    if max_steps is not None:
        max_steps = index(max_steps)
        if max_steps < 0:
            raise ValueError("max_steps must be nonnegative")
    if max_branch_attempts is not None:
        max_branch_attempts = index(max_branch_attempts)
        if max_branch_attempts < 0:
            raise ValueError("max_branch_attempts must be nonnegative")
    output = Path(output)
    if output.exists():
        raise FileExistsError(output)
    if output.resolve().is_relative_to(Path(cache_path).resolve()):
        raise ValueError("branch outputs must be outside the immutable cache directory")
    spec = load_inference_spec(manifest, data_root=data_root)
    run = next((item for item in spec.runs if item.run_id == run_id), None)
    if run is None or run.status != "ready" or run.branch_policy != "strict":
        raise ValueError("select one ready run with the strict source branch policy")
    if progress:
        progress("Verifying the scene cache")
    cache = open_scene_cache(cache_path, inference=spec)
    cap = run.max_steps if max_steps is None else max_steps
    output.mkdir(parents=True, exist_ok=False)
    report = {
        "schema_version": 1,
        "kind": "automatic_branch_tracking",
        "status": "running",
        "run_id": run_id,
        "direction": run.direction,
        "seed": asdict(run.seed),
        "seed_provenance": run.seed_provenance,
        "random_seed": run.random_seed,
        "config": asdict(spec.config),
        "source_baseline": spec.source_baseline,
        "inference_manifest_sha256": sha256(Path(manifest).read_bytes()).hexdigest(),
        "cache_manifest_sha256": cache.metadata["manifest_sha256"],
        "max_steps": cap,
        "max_branch_attempts": max_branch_attempts,
        "confirmation_windows": CONFIRM_WINDOWS,
        "confirmation_limit": CONFIRM_LIMIT,
        "rejoin_windows": REJOIN_WINDOWS,
        "reference_files_read": False,
        "source": None,
        "branches": [],
        "pending_candidates": 0,
        "candidates_at_step_limit": 0,
    }
    report_path = output / "report.json"
    _write_report(report_path, report)
    queue = deque()
    try:
        coverage = TraceCoverage(spec.config)
        with (
            TemporaryDirectory(dir=output, prefix=".states-") as scratch,
            cache.new_run(Path(scratch) / "labels") as template,
        ):
            source = _trace_source(template, spec.config, run, output, cap, coverage, queue, progress)
            report["source"] = source
            report["pending_candidates"] = len(queue)
            _write_report(report_path, report)
            while queue:
                if max_branch_attempts is not None and len(report["branches"]) >= max_branch_attempts:
                    break
                start = queue.popleft()
                remaining = cap - start.elapsed_steps if cap is not None else None
                if remaining is not None and remaining <= 0:
                    report["candidates_at_step_limit"] += 1
                    continue
                route = f"branch-{len(report['branches']) + 1:03d}"
                if progress:
                    progress(f"{route}: {start.parent} window {start.step}, pair {start.pair.indices}")
                try:
                    branch, discovered = _trace_branch(
                        template, spec.config, start, output, route, remaining, coverage
                    )
                except (OSError, ValueError, RuntimeError, MemoryError) as error:
                    branch = {
                        "route": route, "parent": start.parent, "parent_window": start.step,
                        "pair_indices": start.pair.indices, "confirmed_at": None,
                        "status": getattr(error, "status", "failed"), "error": _failure(error),
                    }
                    discovered = []
                report["branches"].append(branch)
                queue.extend(discovered)
                report["pending_candidates"] = len(queue)
                _write_report(report_path, report)
        has_errors = (
            source["error"] is not None or source["initialization"]["source_return"] is not True
            or source["status"].startswith(("undefined_source", "nonfinite"))
            or any(b["error"] or (
                b["confirmed_at"] is not None
                and b["status"].startswith(("undefined_source", "nonfinite"))
            ) for b in report["branches"])
        )
        bounded = (
            queue or report["candidates_at_step_limit"] or source["status"] == "step_limit"
            or any(b["status"] == "step_limit" for b in report["branches"])
        )
        report["status"] = "partial" if has_errors else "limits_reached" if bounded else "queue_drained"
    except KeyboardInterrupt:
        report["status"] = "interrupted"
        raise
    except (OSError, ValueError, RuntimeError, MemoryError) as error:
        report["status"] = getattr(error, "status", "failed")
        report["error"] = _failure(error)
    finally:
        report["pending_candidates"] = len(queue)
        _write_report(report_path, report)
    return report
