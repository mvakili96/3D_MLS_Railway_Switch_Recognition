"""Versioned experiment definitions with an annotation-free inference interface."""

from dataclasses import asdict, dataclass
from hashlib import sha256
import json
import math
from pathlib import Path
import re

from ..config import ExtractionConfig
from ..coverage import coverage_summary
from .catalog import CatalogError, catalog_scene


ADE_BASELINE = coverage_summary()["baseline_id"]


def _reject_constant(value):
    raise CatalogError(f"nonfinite JSON constant: {value}")


def _unique_object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise CatalogError(f"duplicate JSON field: {key}")
        result[key] = value
    return result


def _read_json(path):
    return json.loads(Path(path).read_text(encoding="utf-8"),
                      parse_constant=_reject_constant, object_pairs_hook=_unique_object)


def _object(value, fields, location):
    if not isinstance(value, dict) or set(value) != set(fields):
        raise CatalogError(f"{location}: expected fields {', '.join(fields)}")


def _integer(value, minimum, maximum, location):
    if not isinstance(value, int) or isinstance(value, bool) or not minimum <= value <= maximum:
        raise CatalogError(f"{location}: expected integer in [{minimum}, {maximum}]")
    return value


def _text(value, location):
    if not isinstance(value, str) or not value.strip():
        raise CatalogError(f"{location}: expected nonempty text")
    return value


def _finite(value, location):
    try:
        finite = not isinstance(value, bool) and isinstance(value, (int, float)) and math.isfinite(value)
    except OverflowError:
        finite = False
    if not finite:
        raise CatalogError(f"{location}: expected finite number")
    return float(value)


def _list(value, location):
    if not isinstance(value, list) or not value:
        raise CatalogError(f"{location}: expected nonempty list")
    return value


@dataclass(frozen=True)
class TileSpec:
    path: Path
    sha256: str


@dataclass(frozen=True)
class SeedSpec:
    xyz: tuple[float, float, float]
    angle: float
    mode: int


@dataclass(frozen=True)
class RunSpec:
    run_id: str
    status: str
    seed: SeedSpec | None
    seed_provenance: str
    random_seed: int
    direction: str
    branch_policy: str
    max_steps: int | None


@dataclass(frozen=True)
class InferenceSpec:
    """Everything future prediction code needs; no evaluator/reference paths."""

    data_root: Path
    tiles: tuple[TileSpec, ...]
    input_collection: str
    source_baseline: str
    record_selection: str
    bounds_policy: str
    header_roundoff: str
    coordinate_units: str
    config: ExtractionConfig
    memory_bytes: int
    cache_disk_bytes: int
    runs: tuple[RunSpec, ...]


@dataclass(frozen=True)
class PathMapping:
    run_id: str
    status: str
    path_id: int | None
    layer: int | None
    note: str


@dataclass(frozen=True)
class EvaluationSpec:
    reference_manifest: Path
    expected_path_ids: tuple[int, ...]
    mappings: tuple[PathMapping, ...]


@dataclass(frozen=True)
class PredictionManifest:
    path: Path
    experiment_id: str
    inference: InferenceSpec
    evaluation: EvaluationSpec


def _read_document(path):
    path = Path(path).resolve()
    document = _read_json(path)
    _object(document, ("schema_version", "kind", "experiment_id", "inference", "evaluation"), "manifest")
    if type(document["schema_version"]) is not int or document["schema_version"] != 1 or document["kind"] != "prediction_experiment":
        raise CatalogError("expected prediction_experiment schema 1")
    _text(document["experiment_id"], "experiment_id")
    return path, document


def _parse_inference(path, data, data_root):
    _object(data, ("data_root", "tiles", "input_collection", "source_baseline", "record_selection",
                   "bounds_policy", "header_roundoff", "coordinate_units", "config", "budgets", "runs"), "inference")
    root_text = _text(data["data_root"], "inference.data_root")
    root = Path(data_root).resolve() if data_root is not None else (path.parent/root_text).resolve()
    if data["source_baseline"] != ADE_BASELINE:
        raise CatalogError("inference.source_baseline differs from the frozen ADE tracker")
    if data["record_selection"] != "per_file_even_zero_based":
        raise CatalogError("only per_file_even_zero_based source selection is implemented")
    if data["bounds_policy"] != "fixed_header_union":
        raise CatalogError("bounds_policy must be fixed_header_union")
    if data["header_roundoff"] not in ("omit_outside_within_four_ulps", "reject_outside"):
        raise CatalogError("unknown header_roundoff policy")
    _text(data["input_collection"], "inference.input_collection")
    if data["coordinate_units"] != "m":
        raise CatalogError("this prediction profile requires explicitly declared metre coordinates")
    tiles, paths = [], set()
    for tile in _list(data["tiles"], "inference.tiles"):
        _object(tile, ("path", "sha256"), "tile")
        raw = Path(_text(tile["path"], "tile.path"))
        resolved = (root/raw).resolve()
        if raw.is_absolute() or not resolved.is_relative_to(root) or raw.suffix.casefold() not in (".las", ".laz"):
            raise CatalogError("tile.path must be an extracted LAS/LAZ path within data_root")
        if resolved in paths:
            raise CatalogError("duplicate tile path")
        paths.add(resolved)
        if not isinstance(tile["sha256"], str) or not re.fullmatch("[0-9a-f]{64}", tile["sha256"]):
            raise CatalogError("tile.sha256 must be a lowercase SHA256 digest")
        tiles.append(TileSpec(resolved, tile["sha256"]))
    defaults = ExtractionConfig().to_dict()
    _object(data["config"], tuple(defaults), "inference.config")
    for name, default in defaults.items():
        if type(default) is int:
            _integer(data["config"][name], 0, 2**31-1, f"config.{name}")
        else:
            _finite(data["config"][name], f"config.{name}")
    for name in ("grid_size", "skip_rate", "track_window_width", "track_window_length", "track_gauge"):
        if data["config"][name] <= 0:
            raise CatalogError(f"config.{name} must be positive")
    _object(data["budgets"], ("memory_bytes", "cache_disk_bytes"), "inference.budgets")
    budgets = {k: _integer(v, 1, 2**63-1, f"budgets.{k}") for k, v in data["budgets"].items()}
    runs, run_ids = [], set()
    for run in _list(data["runs"], "inference.runs"):
        _object(run, ("run_id", "status", "seed", "seed_provenance", "random_seed", "direction",
                      "branch_policy", "max_steps"), "run")
        identity = _text(run["run_id"], "run_id")
        if not re.fullmatch("[A-Za-z0-9_-]+", identity) or identity in run_ids:
            raise CatalogError("run IDs must be unique filename-safe identifiers")
        run_ids.add(identity)
        if run["status"] not in ("ready", "pending"):
            raise CatalogError("run status must be ready or pending")
        seed = None
        if run["status"] == "pending":
            if run["seed"] is not None:
                raise CatalogError("pending seeds must be null; do not supply guessed values")
        else:
            _object(run["seed"], ("xyz", "angle", "mode"), "seed")
            xyz = run["seed"]["xyz"]
            if not isinstance(xyz, list) or len(xyz) != 3:
                raise CatalogError("seed.xyz must contain three finite coordinates")
            seed = SeedSpec(tuple(_finite(v, "seed.xyz") for v in xyz),
                            _finite(run["seed"]["angle"], "seed.angle"),
                            _integer(run["seed"]["mode"], 0, 4, "seed.mode"))
        _text(run["seed_provenance"], "seed_provenance")
        _integer(run["random_seed"], 0, 2**32-1, "random_seed")
        if run["direction"] not in ("single", "bidirectional"):
            raise CatalogError("direction must be single or bidirectional")
        if run["branch_policy"] not in ("strict", "fitted_references"):
            raise CatalogError("branch_policy must be strict or fitted_references")
        if run["max_steps"] is not None:
            _integer(run["max_steps"], 0, 2**31-1, "max_steps")
        runs.append(RunSpec(identity, run["status"], seed, run["seed_provenance"], run["random_seed"],
                            run["direction"], run["branch_policy"], run["max_steps"]))
    return InferenceSpec(root, tuple(tiles), data["input_collection"], data["source_baseline"],
                         data["record_selection"], data["bounds_policy"], data["header_roundoff"],
                         data["coordinate_units"], ExtractionConfig(**data["config"]),
                         budgets["memory_bytes"], budgets["cache_disk_bytes"], tuple(runs))


def load_inference_spec(path, *, data_root=None):
    """Load only prediction settings; evaluation content and files are never read."""
    resolved, document = _read_document(path)
    return _parse_inference(resolved, document["inference"], data_root)


def load_prediction_manifest(path, *, data_root=None):
    resolved, document = _read_document(path)
    inference = _parse_inference(resolved, document["inference"], data_root)
    data = document["evaluation"]
    _object(data, ("reference_manifest", "expected_path_ids", "mappings"), "evaluation")
    reference = (resolved.parent/_text(data["reference_manifest"], "reference_manifest")).resolve()
    ids = tuple(_integer(v, 1, 2**31-1, "expected_path_id")
                for v in _list(data["expected_path_ids"], "expected_path_ids"))
    if len(set(ids)) != len(ids):
        raise CatalogError("duplicate expected path ID")
    mappings, seen, assignments = [], set(), set()
    known_runs = {run.run_id for run in inference.runs}
    for mapping in _list(data["mappings"], "evaluation.mappings"):
        _object(mapping, ("run_id", "status", "path_id", "layer", "note"), "mapping")
        identity = _text(mapping["run_id"], "mapping.run_id")
        if identity not in known_runs:
            raise CatalogError("mapping must name a declared inference run")
        seen.add(identity)
        if mapping["status"] not in ("pending", "reviewed"):
            raise CatalogError("mapping status must be pending or reviewed")
        pid, layer = mapping["path_id"], mapping["layer"]
        if pid is not None:
            _integer(pid, 1, 2**31-1, "mapping.path_id")
            if pid not in ids:
                raise CatalogError("mapping path ID is outside expected scope")
        if layer is not None:
            _integer(layer, 0, 2**31-1, "mapping.layer")
        if mapping["status"] == "reviewed" and (pid is None or layer is None):
            raise CatalogError("reviewed mappings require an explicit path ID and layer")
        if (identity, layer) in assignments:
            raise CatalogError("each run/layer can have only one mapping")
        assignments.add((identity, layer))
        mappings.append(PathMapping(identity, mapping["status"], pid, layer,
                                    _text(mapping["note"], "mapping.note")))
    if seen != known_runs:
        raise CatalogError("each declared run needs an explicit reviewed or pending mapping")
    return PredictionManifest(resolved, document["experiment_id"], inference,
                              EvaluationSpec(reference, ids, tuple(mappings)))


def audit_prediction_manifest(path, *, data_root=None, progress=None):
    """Check metadata/hashes and configuration readiness, not algorithm success."""
    experiment = load_prediction_manifest(path, data_root=data_root)
    inference = experiment.inference
    catalog = catalog_scene([t.path for t in inference.tiles], root=inference.data_root, progress=progress)
    errors = list(catalog["issues"])
    actual = {entry["order"]: entry for entry in catalog["tiles"]}
    for order, tile in enumerate(inference.tiles):
        if order in actual and actual[order]["sha256"] != tile.sha256:
            errors.append({"path": str(tile.path), "status": "sha256_mismatch"})
    ready_runs, pending_runs = [], []
    for run in inference.runs:
        if run.status == "pending":
            pending_runs.append(run.run_id)
        else:
            contained = any(all(t["bounds"]["minimum"][axis] <= run.seed.xyz[axis]
                                <= t["bounds"]["maximum"][axis] for axis in range(3))
                            for t in catalog["tiles"])
            if not contained:
                errors.append({"run_id": run.run_id, "status": "seed_outside_available_tile_footprints"})
            else:
                ready_runs.append(run.run_id)
    reference_info = None
    try:
        reference_path = experiment.evaluation.reference_manifest
        document = _read_json(reference_path)
        if (not isinstance(document, dict)
                or type(document.get("schema_version")) is not int or document["schema_version"] != 1
                or document.get("profile") != "source2018"):
            raise CatalogError("reference manifest must use source2018 schema 1")
        cases = _list(document.get("cases"), "reference cases")
        case_ids = [_text(case["case_id"], "reference case_id") for case in cases]
        if len(set(case_ids)) != len(case_ids):
            raise CatalogError("duplicate reference case ID")
        pids = {_integer(case["path_id"], 1, 2**31-1, "reference path_id") for case in cases}
        missing = set(experiment.evaluation.expected_path_ids)-pids
        if missing:
            raise CatalogError(f"reference manifest is missing expected paths {sorted(missing)}")
        reference_info = {"path": str(reference_path),
                          "sha256": sha256(reference_path.read_bytes()).hexdigest(),
                          "case_count": len(cases), "path_ids": sorted(pids),
                          "annotation_or_model_files_opened": False}
    except (OSError, ValueError, KeyError, TypeError) as error:
        errors.append({"status": "reference_manifest_error", "message": str(error)})
    pending_mappings = [m.run_id for m in experiment.evaluation.mappings if m.status == "pending"]
    mapped = {m.path_id for m in experiment.evaluation.mappings
              if m.status == "reviewed" and m.run_id in ready_runs}
    configuration_ready = not errors and set(experiment.evaluation.expected_path_ids) <= mapped
    return {"schema_version": 1, "kind": "prediction_experiment_audit",
            "status": "invalid" if errors else "valid_with_pending" if pending_runs or pending_mappings else "valid",
            "experiment_id": experiment.experiment_id, "manifest": str(experiment.path),
            "manifest_sha256": sha256(experiment.path.read_bytes()).hexdigest(),
            "errors": errors, "catalog": catalog, "reference_scope": reference_info,
            "readiness": {"ready_seed_runs": ready_runs, "pending_seed_runs": pending_runs,
                          "pending_path_mappings": pending_mappings,
                          "input_configuration_ready": not errors and bool(ready_runs),
                          "expected_path_configuration_ready": configuration_ready,
                          "prediction_runner_available": False,
                          "seed_initialization_verified": False,
                          "predictions_generated": False, "nine_path_complete": False},
            "policies": {"record_selection": inference.record_selection,
                         "bounds_policy": inference.bounds_policy,
                         "header_roundoff": inference.header_roundoff,
                         "source_baseline": inference.source_baseline,
                         "config": asdict(inference.config),
                         "memory_bytes": inference.memory_bytes,
                         "cache_disk_bytes": inference.cache_disk_bytes,
                         "budgets_enforced_at_catalog_stage": False},
            "validation_scope": "Configuration, headers and byte hashes only. Seed footprints do not "
                                "prove initialization success; pending routes remain unassigned."}
