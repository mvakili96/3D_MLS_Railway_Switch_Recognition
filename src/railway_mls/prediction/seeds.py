"""Capture a manually chosen raw-cloud pose and version a ready run definition."""

from copy import deepcopy
import math
from pathlib import Path

from ..evaluation.io import file_fingerprint
from .catalog import write_json_new
from .manifest import _read_json, load_prediction_manifest


def _pose(value):
    if not isinstance(value, dict) or set(value) != {"xyz", "angle", "mode"}:
        raise ValueError("seed requires xyz, angle and mode")
    xyz = value["xyz"]
    if not isinstance(xyz, list) or len(xyz) != 3:
        raise ValueError("seed.xyz requires three coordinates")
    if any(isinstance(v, bool) or not isinstance(v, (float, int)) or not math.isfinite(v)
           for v in [*xyz, value["angle"]]):
        raise ValueError("seed coordinates/angle must be finite")
    if isinstance(value["mode"], bool) or not isinstance(value["mode"], int) or not 0 <= value["mode"] <= 4:
        raise ValueError("seed mode must be an integer from 0 to 4")
    return deepcopy(value)


def write_seed_capture(output, pose, *, data_file, random_seed, config, provenance):
    """Save the user's supplied confirmation pose, not a fitted rail or branch toe."""
    if Path(output).exists():
        raise FileExistsError(output)
    seed = _pose({"xyz": pose.point.as_array().tolist(), "angle": pose.angle, "mode": pose.mode})
    if not isinstance(provenance, str) or not provenance.strip():
        raise ValueError("seed capture requires provenance")
    document = {"schema_version": 1, "kind": "manual_tracking_seed", "seed": seed,
                "raw_source": file_fingerprint(data_file), "random_seed": random_seed,
                "viewer_config": config.to_dict(), "provenance": provenance,
                "capture_policy": "literal confirmed pose on raw cloud; no reference labels or original model vertices",
                "headless_protocol": "direct supplied-seed initialization; viewer preview draws are not replayed"}
    write_json_new(output, document)
    return document


def prepare_seed_manifest(manifest, run_id, capture, output, *, provenance):
    """Create a new experiment definition; never edit an existing frozen manifest."""
    original = Path(manifest).resolve()
    output = Path(output).resolve()
    if output.exists():
        raise FileExistsError(output)
    if not isinstance(provenance, str) or not provenance.strip():
        raise ValueError("record why this manually selected start belongs to the intended path")
    experiment = load_prediction_manifest(original)
    captured = _read_json(capture)
    if captured.get("kind") != "manual_tracking_seed" or captured.get("schema_version") != 1:
        raise ValueError("expected a schema 1 manual_tracking_seed capture")
    seed = _pose(captured["seed"])
    matches = [tile for tile in experiment.inference.tiles if tile.sha256 == captured["raw_source"]["sha256"]]
    if not matches:
        raise ValueError("captured raw tile is outside the declared benchmark input collection")
    document = _read_json(original)
    run = next((r for r in document["inference"]["runs"] if r["run_id"] == run_id), None)
    if run is None:
        raise ValueError("unknown declared run ID; declare separately named restarts explicitly")
    run.update(status="ready", seed=seed,
               seed_provenance=f"{provenance.strip()} Capture {Path(capture).resolve()} SHA256 {file_fingerprint(capture)['sha256']}; {captured['provenance']}")
    # Rebase relative input/reference paths when the new definition moves folders.
    import os
    document["inference"]["data_root"] = os.path.relpath(experiment.inference.data_root, output.parent)
    document["evaluation"]["reference_manifest"] = os.path.relpath(experiment.evaluation.reference_manifest, output.parent)
    # Changing an initialization invalidates prior layer-review decisions for that run.
    mappings = document["evaluation"]["mappings"]
    targets = [m for m in mappings if m["run_id"] == run_id]
    if len({m["path_id"] for m in targets}) != 1:
        raise ValueError("seed preparation needs one intended path per independent run")
    replacement = {**targets[0], "status": "pending", "layer": None,
                   "note": "New manually captured initialization; exported layers require review."}
    document["evaluation"]["mappings"] = [m for m in mappings if m["run_id"] != run_id]+[replacement]
    write_json_new(output, document)
    try:
        load_prediction_manifest(output)
    except (ValueError, OSError):
        output.unlink()
        raise
    return document
