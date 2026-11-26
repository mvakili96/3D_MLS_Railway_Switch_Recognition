"""Bounded rail-pair lookahead from one window of a source prediction run."""

from dataclasses import asdict, replace
from hashlib import sha256
from itertools import pairwise
from operator import index
from pathlib import Path
from tempfile import TemporaryDirectory

from ..continuations import ContinuationTracker, fitted_window_layer, pairs_at_step
from ..geometry import distance_2d
from ..models import InitialLocation
from ..orchestration import TrackingEngine
from ..points import Point3D
from ..sampling import MsvcRand
from .cache import open_scene_cache
from .manifest import load_inference_spec
from .runner import _initialization, _invocations, _Journal, _point, _write_geometry, _write_report


def _source_prefix(cache, config, run, output, split_step, progress):
    directory = output / "prefix"
    directory.mkdir()
    rng = MsvcRand(run.random_seed)
    with (
        TemporaryDirectory(dir=output, prefix=".state-") as scratch,
        cache.new_run(Path(scratch) / "labels") as scene,
        (directory / "steps.jsonl").open("x", encoding="utf-8") as stream,
    ):
        journal = _Journal(stream, cache, scene, output, None, run.run_id, rng)

        def observe(layer, direction, main_track, step):
            journal(layer, direction, main_track, step)
            if progress and step.number % 25 == 0:
                progress(f"{run.run_id}: source prefix, window {step.number}")

        journal.resources()
        engine = TrackingEngine(
            scene.grid, config=config, rng=rng, retain_history=False, step_callback=observe
        )
        initial = engine.initialize(InitialLocation(
            Point3D(*run.seed.xyz), run.seed.angle, run.seed.mode
        ))
        workflow = main_pass = None
        if initial.success:
            if run.direction == "bidirectional":
                workflow = engine.run_bidirectional(max_steps=split_step)
                main_pass = workflow.forward_pass
            else:
                workflow = main_pass = engine.run_pass(max_steps=split_step)
        prefix = {
            "status": workflow.status if workflow is not None else initial.status,
            "initialization": _initialization(initial),
            "invocations": _invocations(workflow),
            "layers": _write_geometry(directory, engine.layers),
            "random_state": rng.state,
            "random_draws": rng.draw_count,
            "split": None,
        }
        result = main_pass.main.result if main_pass is not None else None
        if result is None or not result.steps or result.steps[-1].number != split_step:
            return prefix, (), None
        step = result.steps[-1]
        prefix["split"] = {
            "step": step.number,
            "source_status": step.status,
            "source_accepted": step.accepted,
            "source_selected_pair": (step.selection.selected_indices
                                     if step.selection is not None else None),
            "predicted_center": _point(step.attempts[-1].segment.center) if step.attempts else None,
            "model_line_counts": [len(model.lines) for model in step.models],
        }
        pairs = pairs_at_step(step, config)
        # The winning MDL lines share the incoming heading. Preserve the
        # accepted motion and headings for the source-selected continuation.
        selected = prefix["split"]["source_selected_pair"]
        fitted = (
            fitted_window_layer(step.candidate)
            if step.status == "accepted" and any(pair.indices == selected for pair in pairs)
            else None
        )
        return prefix, pairs, fitted


def _follow_pair(cache, config, indices, layer, random_state, output, name, lookahead_steps):
    directory = output / name
    directory.mkdir()
    heights = layer.rail1_vector[0].z, layer.rail2_vector[0].z
    rng = MsvcRand(random_state)
    selected_pairs = []
    with (
        TemporaryDirectory(dir=output, prefix=".state-") as scratch,
        cache.new_run(Path(scratch) / "labels") as scene,
        (directory / "steps.jsonl").open("x", encoding="utf-8") as stream,
    ):
        journal = _Journal(stream, cache, scene, output, None, name, rng)

        def observe(step):
            journal(0, 0, False, step)
            if step.selection is not None:
                selected_pairs.append({"step": step.number, "indices": step.selection.selected_indices})

        journal.resources()
        result = ContinuationTracker(
            layer,
            scene.grid,
            heights,
            config=config,
            rng=rng,
            main_track=False,
            retain_history=False,
            step_callback=observe,
        ).run(max_steps=lookahead_steps)
        # The source can append a vector vertex before rejecting the step.
        count = len(layer.track_segments) - (result.status == "nonfinite_tracking_geometry")
        accepted_layer = replace(
            layer,
            track_segments=layer.track_segments[:count],
            rail1_vector=layer.rail1_vector[:count],
            rail2_vector=layer.rail2_vector[:count],
            center_vector=layer.center_vector[:count],
        )
        layers = _write_geometry(directory, [accepted_layer])
    centers = accepted_layer.center_vector
    return {
        "directory": name,
        "pair_indices": indices,
        "status": result.status,
        "attempted_steps": result.attempted_steps,
        "accepted_steps": len(centers) - 1,
        "begin": _point(centers[0]),
        "end": _point(centers[-1]),
        "length_xy_m": sum(distance_2d(a, b) for a, b in pairwise(centers)),
        "random_state": rng.state,
        "random_draws": rng.draw_count,
        "selected_pairs": selected_pairs,
        "layers": layers,
    }


def probe_switch(manifest, cache_path, output, *, run_id, split_step, lookahead_steps,
                 data_root=None, progress=None):
    """Replay through one forward window and follow each gauge-compatible pair."""
    split_step, lookahead_steps = index(split_step), index(lookahead_steps)
    if min(split_step, lookahead_steps) <= 0:
        raise ValueError("split and lookahead steps must be positive")
    output = Path(output)
    if output.exists():
        raise FileExistsError(output)
    if output.resolve().is_relative_to(Path(cache_path).resolve()):
        raise ValueError("switch probe outputs must be outside the immutable cache directory")
    spec = load_inference_spec(manifest, data_root=data_root)
    run = next((item for item in spec.runs if item.run_id == run_id), None)
    if run is None or run.status != "ready" or run.branch_policy != "strict":
        raise ValueError("select one ready run with the strict source branch policy")
    if progress:
        progress("Verifying the scene cache")
    cache = open_scene_cache(cache_path, inference=spec)
    output.mkdir(parents=True, exist_ok=False)
    report = {
        "schema_version": 1,
        "kind": "switch_continuation_probe",
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
        "split_step": split_step,
        "lookahead_steps": lookahead_steps,
        "reference_files_read": False,
        "prefix": None,
        "pairs": [],
        "branches": [],
    }
    report_path = output / "report.json"
    _write_report(report_path, report)
    try:
        prefix, pairs, fitted = _source_prefix(cache, spec.config, run, output, split_step, progress)
        report["prefix"] = prefix
        report["pairs"] = [
            {"indices": pair.indices, "gauge_m": pair.gauge, "shared_length_m": pair.shared_length,
             "support_counts": pair.support_counts, "predicted_offset_m": pair.continuity_error,
             "rails": [[_point(a), _point(b)] for a, b in zip(pair.begins, pair.ends)]}
            for pair in pairs
        ]
        _write_report(report_path, report)
        if prefix["split"] is None:
            report["status"] = "split_not_reached"
        elif len(pairs) < 2:
            report["status"] = "no_split"
        else:
            for number, pair in enumerate(pairs, start=1):
                name = f"branch-{number:02d}"
                if progress:
                    progress(f"{run_id}: following {name} from model pair {pair.indices}")
                use_fitted = (
                    fitted is not None and pair.indices == prefix["split"]["source_selected_pair"]
                )
                layer = fitted if use_fitted else pair.seed_layer(spec.config)
                branch = _follow_pair(
                    cache, spec.config, pair.indices, layer, prefix["random_state"],
                    output, name, lookahead_steps,
                )
                branch["seed_pose"] = "source_fitted_window" if use_fitted else "paired_mdl_lines"
                report["branches"].append(branch)
                _write_report(report_path, report)
            complete = all(branch["status"] == "step_limit" for branch in report["branches"])
            report["status"] = "lookahead_complete" if complete else "lookahead_partial"
    except KeyboardInterrupt:
        report["status"] = "interrupted"
        raise
    except (OSError, ValueError, RuntimeError, MemoryError) as error:
        report["status"] = getattr(error, "status", "failed")
        report["error"] = {"type": type(error).__name__, "message": str(error)}
    finally:
        _write_report(report_path, report)
    return report
