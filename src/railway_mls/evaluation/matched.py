"""Explicit, frozen prediction-to-reference mappings and paired evaluations.

This module consumes frozen exports. It never invokes a tracking runner,
chooses a layer by score, crops annotations, or joins separate polylines.
"""

import csv
from dataclasses import asdict
from hashlib import sha256
import json
from pathlib import Path
import re
import shutil

import numpy as np

from ..prediction.manifest import load_inference_spec
from .io import file_fingerprint, read_python_vectors
from .runner import _write_json, load_cases, run_evaluation
from .types import RailSegments


def _object(value, fields, name):
    if not isinstance(value, dict) or set(value) != set(fields):
        raise ValueError(f"{name}: expected fields {', '.join(fields)}")


def _identity(value):
    if not isinstance(value, str) or not re.fullmatch(r"[A-Za-z0-9_-]+", value):
        raise ValueError("expected a nonempty safe ID (letters, digits, underscore, hyphen)")
    return value


def _integer(value, minimum=0):
    if isinstance(value, bool) or not isinstance(value, int) or value < minimum:
        raise ValueError(f"expected integer >= {minimum}")
    return value


def _text(value):
    if not isinstance(value, str) or not value.strip():
        raise ValueError("expected nonempty text")
    return value


def _unique(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError(f"duplicate JSON field: {key}")
        result[key] = value
    return result


def _json(path, expected=None):
    path = Path(path).resolve()
    payload = path.read_bytes()
    fingerprint = {"path": str(path), "bytes": len(payload), "sha256": sha256(payload).hexdigest()}
    if expected is not None and fingerprint["sha256"] != expected:
        raise ValueError(f"{path}: SHA256 mismatch")
    def reject(value):
        raise ValueError(f"nonfinite JSON constant: {value}")
    return json.loads(payload, object_pairs_hook=_unique, parse_constant=reject), fingerprint


def _pinned(path, expected):
    if not isinstance(expected, str) or not re.fullmatch(r"[0-9a-f]{64}", expected):
        raise ValueError(f"{path}: an explicit SHA256 pin is required")
    fingerprint = file_fingerprint(path)
    if fingerprint["sha256"] != expected:
        raise ValueError(f"{path}: SHA256 mismatch")
    return fingerprint


def _source(entry, parent):
    _object(entry, ("source_id", "directory", "manifest_sha256", "inference_manifest"), "prediction source")
    sid = _identity(entry["source_id"])
    directory = (parent/_text(entry["directory"])).resolve()
    _pinned(directory/"manifest.json", entry["manifest_sha256"])
    root, fingerprint = _json(directory/"manifest.json", entry["manifest_sha256"])
    if root.get("kind") != "headless_prediction_run" or root.get("schema_version") != 1:
        raise ValueError("prediction source must be a schema 1 headless prediction export")
    if root.get("reference_files_read") is not False or root.get("evaluation_performed") is not False:
        raise ValueError("expected prediction exports generated without reference evaluation")
    definition = (parent/_text(entry["inference_manifest"])).resolve()
    definition_fp = _pinned(definition, root.get("manifest_sha256"))
    inference = load_inference_spec(definition)  # Parses configuration only; no LAS/cache reads.
    runs = {}
    for record in root["runs"]:
        rid = _identity(record["run_id"])
        if rid in runs:
            raise ValueError("duplicate run ID in frozen prediction export")
        runs[rid] = record
    return sid, {"directory": directory, "root": root, "runs": runs, "inference": inference,
                 "manifest": fingerprint, "inference_manifest": definition_fp}


def _fragment(entry, sources):
    _object(entry, ("source_id", "run_id", "layer", "run_manifest_sha256"), "fragment")
    sid, rid, layer = _identity(entry["source_id"]), _identity(entry["run_id"]), _integer(entry["layer"])
    if sid not in sources or rid not in sources[sid]["runs"]:
        raise ValueError(f"unknown prediction source/run: {sid}/{rid}")
    source = sources[sid]
    recorded = source["runs"][rid]
    directory = source["directory"]/rid
    _pinned(directory/"manifest.json", entry["run_manifest_sha256"])
    metadata, fingerprint = _json(directory/"manifest.json", entry["run_manifest_sha256"])
    # The JSON writer adds a document-level nonfinite annotation list. Nested
    # root summaries have their annotations at the root, not inside each run.
    if {key: value for key, value in metadata.items() if key != "nonfinite_values"} != recorded:
        raise ValueError(f"{rid}: run manifest differs from pinned root export")
    if metadata.get("schema_version") != 1 or metadata.get("branch_policy") != "strict":
        raise ValueError("matched source evaluation requires the strict prediction profile")
    if metadata.get("status") in (None, "initializing", "running"):
        raise ValueError("prediction run must be frozen after completion or a recorded stop/failure")
    inference = source["inference"]
    if (metadata.get("source_baseline") != inference.source_baseline
            or metadata.get("config") != asdict(inference.config)
            or metadata.get("reference_files_read") is not False
            or metadata.get("evaluation_performed") is not False):
        raise ValueError(f"{rid}: prediction provenance differs from frozen inference definition")
    declared = next((run for run in inference.runs if run.run_id == rid), None)
    if (declared is None or declared.status != "ready" or declared.branch_policy != "strict"
            or metadata.get("seed") != {**asdict(declared.seed), "xyz": list(declared.seed.xyz)}
            or metadata.get("direction") != declared.direction
            or metadata.get("random", {}).get("seed") != declared.random_seed):
        raise ValueError(f"{rid}: seed/direction/RNG policy differs from frozen inference definition")
    if layer not in [item["layer"] for item in metadata.get("layers", [])]:
        raise ValueError(f"{rid}: unknown layer {layer}")
    vectors = directory/"vectors.csv"
    vector_fp = _pinned(vectors, metadata["artifacts"]["vectors.csv"]["sha256"])
    if vector_fp["bytes"] != metadata["artifacts"]["vectors.csv"]["bytes"]:
        raise ValueError(f"{rid}: vector byte count differs from frozen export")
    rails = read_python_vectors(vectors, layer=layer)
    if not all(len(rail) for rail in rails):
        raise ValueError(f"{rid}/{layer}: fragment requires segments on both rails")
    return {**entry, "rails": rails, "vectors": vector_fp, "run_manifest": fingerprint,
            "run_summary": metadata}


def concatenate_fragments(fragments):
    """Append segment arrays in declared order; reject exact cross-fragment duplicates.

    Source repeats within a layer remain intact. A shared segment contributed by
    two separate fragments needs an explicit revised export/mapping, not deduplication.
    """
    seen = set()
    for fragment in fragments:
        current = set()
        for rail in fragment["rails"]:
            for segment in rail.endpoints:
                a, b = (tuple(point) for point in segment)
                current.add(tuple(sorted((a, b))))
        if seen & current:
            raise ValueError("separate fragments contain an identical segment; review the mapping")
        seen.update(current)
    return tuple(RailSegments(np.concatenate([f["rails"][side].endpoints for f in fragments]),
                              {"format": "ordered_explicit_fragments", "side": side})
                 for side in (0, 1))


def load_matched_plan(manifest, *, data_root=None, path_ids=None):
    """Validate reviewed identities, complete path scopes, and all consumed input pins."""
    path = Path(manifest).resolve()
    document, fingerprint = _json(path)
    _object(document, ("schema_version", "kind", "profile", "reference_manifest",
                       "prediction_sources", "paths"), "matched evaluation manifest")
    if (document["schema_version"], document["kind"], document["profile"]) != (
            1, "matched_prediction_evaluation", "source2018"):
        raise ValueError("expected schema 1 matched_prediction_evaluation, profile source2018")
    reference = document["reference_manifest"]
    _object(reference, ("path", "sha256"), "reference manifest")
    reference_path = (path.parent/_text(reference["path"])).resolve()
    reference_fp = _pinned(reference_path, reference["sha256"])
    all_cases, context = load_cases(reference_path, data_root=data_root)
    selected = None
    if path_ids is not None:
        if (not path_ids or any(isinstance(pid, bool) or not isinstance(pid, int) for pid in path_ids)
                or len(path_ids) != len(set(path_ids))):
            raise ValueError("select unique integer path IDs")
        selected = set(path_ids)
        if selected-set(map(int, context["expected_case_ids_by_path"])):
            raise ValueError("selected path ID is outside reference scope")
    sources = {}
    if not isinstance(document["prediction_sources"], list):
        raise ValueError("prediction_sources must be a list")
    for entry in document["prediction_sources"]:
        sid, source = _source(entry, path.parent)
        if sid in sources:
            raise ValueError("duplicate source ID")
        sources[sid] = source
    mappings, pending, seen_paths, seen_fragments = [], [], set(), set()
    if not isinstance(document["paths"], list):
        raise ValueError("paths must be a list")
    for mapping in document["paths"]:
        _object(mapping, ("path_id", "status", "case_ids", "fragments", "review_note"), "path mapping")
        pid = _integer(mapping["path_id"], 1)
        if str(pid) not in context["expected_case_ids_by_path"] or pid in seen_paths:
            raise ValueError("unknown/duplicate reference path ID")
        seen_paths.add(pid)
        if selected is not None and pid not in selected:
            continue
        _text(mapping["review_note"])
        expected = context["expected_case_ids_by_path"][str(pid)]
        if mapping["case_ids"] != expected:
            raise ValueError(f"Path {pid}: case_ids must list the complete reference path in manifest order")
        if not isinstance(mapping["fragments"], list):
            raise ValueError("fragments must be a list")
        if mapping["status"] == "pending":
            if mapping["fragments"]:
                raise ValueError("pending mappings must not assign prediction fragments")
            pending.append(mapping)
            continue
        if mapping["status"] != "reviewed" or not mapping["fragments"]:
            raise ValueError("a reviewed mapping requires explicit fragments")
        fragments = [_fragment(entry, sources) for entry in mapping["fragments"]]
        for fragment in fragments:
            identity = (fragment["vectors"]["sha256"], fragment["layer"])
            if identity in seen_fragments:
                raise ValueError("prediction fragment assigned more than once (including copied exports)")
            seen_fragments.add(identity)
        rails = concatenate_fragments(fragments)
        cases = [case for case in all_cases if case["path_id"] == pid]
        for case in cases:
            for key in ("las", "left", "right"):
                _pinned(case[key], case.get("sha256", {}).get(key))
        for side in ("left", "right"):
            if len({case["sha256"][side] for case in cases}) != 1:
                raise ValueError("a reference path must use the same original model in every part")
        mappings.append({**mapping, "fragments": fragments, "rails": rails, "cases": cases})
    if not mappings:
        raise ValueError("no reviewed path mappings to evaluate")
    if selected is not None and selected != {m["path_id"] for m in mappings}:
        raise ValueError("every selected path requires a reviewed mapping")
    return {"manifest": fingerprint, "reference_manifest": reference_fp,
            "sources": sources, "mappings": mappings, "pending": pending,
            "unmapped_path_ids": sorted(int(pid) for pid in context["expected_case_ids_by_path"]
                                       if int(pid) not in {m["path_id"] for m in mappings}),
            "context": context}


def coverage_diagnostics(counts, coverage):
    positive = coverage["positive"]
    return {"positive_assignment_percent": 100*coverage["assigned_positive"]/positive if positive else None,
            "all_positive_success_percent": 100*counts["nTP"]/positive if positive else None}


def _write_segments(path, rail):
    with path.open("x", newline="") as stream:
        stream.write(f"{len(rail)}\n")
        csv.writer(stream).writerows(rail.endpoints.reshape(-1, 6).tolist())


def run_matched_evaluation(manifest, output, *, data_root=None, chunk_size=250_000,
                           point_outcomes=False, overlay=False, progress=None, path_ids=None):
    """Score both models on identical complete annotation parts, with fresh sessions."""
    if chunk_size <= 0:
        raise ValueError("chunk_size must be positive")
    plan = load_matched_plan(manifest, data_root=data_root, path_ids=path_ids)
    if overlay:
        from .overlay import require_plotting
        require_plotting()
    path = Path(output).resolve()
    # Never place generated files inside a frozen prediction or reference input directory.
    protected = [source["directory"] for source in plan["sources"].values()]
    protected.append(Path(plan["context"]["dataset_root"]))
    if any(path == parent or parent in path.parents for parent in protected):
        raise ValueError("evaluation output must be separate from frozen prediction/reference inputs")
    path.mkdir(parents=True, exist_ok=False)
    report = {"schema_version": 1, "kind": "matched_prediction_evaluation", "status": "running",
              "manifest": plan["manifest"], "reference_manifest": plan["reference_manifest"],
              "mapping_policy": "explicit post-run review; no automatic matching or score-based selection",
              "scope": "complete declared reference paths; independent annotation parts, overlaps retained",
              "comparison_note": "Source metrics use each model's own assigned points. Different assignment coverage prevents an equal-coverage accuracy claim.",
              "metric_definitions": {
                  "positive_assignment_percent": "100*assigned_positive/all reference-positive records",
                  "all_positive_success_percent": "100*nTP/all reference-positive records; coverage diagnostic, not source recall",
                  "path_error_statistics": "not pooled; see source error statistics for each case"},
              "pending_mappings": plan["pending"], "unmapped_path_ids": plan["unmapped_path_ids"],
              "nine_path_complete": False, "mappings": [], "models": {}, "cases": [], "paths": [],
              "failures": [], "reference_files_used_for_prediction": False}
    _write_json(path/"report.json", report)
    provenance_dir = path/"provenance"
    provenance_dir.mkdir()
    for name, fingerprint in (("mapping.json", plan["manifest"]),
                              ("reference.json", plan["reference_manifest"])):
        shutil.copyfile(fingerprint["path"], provenance_dir/name)
        _pinned(provenance_dir/name, fingerprint["sha256"])
    report["prediction_sources"] = {}
    for sid, source in plan["sources"].items():
        copied = {}
        for name in ("manifest", "inference_manifest"):
            original = source[name]
            destination = provenance_dir/f"{sid}-{name}.json"
            shutil.copyfile(original["path"], destination)
            copied[name] = _pinned(destination, original["sha256"])
        report["prediction_sources"][sid] = {"manifest": source["manifest"],
                                              "inference_manifest": source["inference_manifest"],
                                              "copied_provenance": copied}
    cases_by_model = {"python": [], "original": []}
    for mapping in plan["mappings"]:
        pid = mapping["path_id"]
        vector_dir = path/f"path-{pid}-vectors"
        vector_dir.mkdir()
        files = {model: {} for model in cases_by_model}
        for side, rail in zip(("left", "right"), mapping["rails"]):
            files["python"][side] = vector_dir/f"python-{side}.txt"
            files["original"][side] = vector_dir/f"original-{side}.txt"
            _write_segments(files["python"][side], rail)
            shutil.copyfile(mapping["cases"][0][side], files["original"][side])
            _pinned(files["original"][side], mapping["cases"][0]["sha256"][side])
        fragments, offsets = [], [0, 0]
        for index, fragment in enumerate(mapping["fragments"]):
            copied = vector_dir/f"fragment-{index}-vectors.csv"
            shutil.copyfile(fragment["vectors"]["path"], copied)
            _pinned(copied, fragment["vectors"]["sha256"])
            ranges = {side: [offsets[i], offsets[i]+len(fragment["rails"][i])]
                      for i, side in enumerate(("left", "right"))}
            offsets = [ranges[side][1] for side in ("left", "right")]
            fragments.append({key: value for key, value in fragment.items() if key != "rails"})
            fragments[-1].update({"segment_ranges_per_side": ranges,
                                  "copied_vectors": file_fingerprint(copied)})
        report["mappings"].append({"path_id": pid, "case_ids": mapping["case_ids"],
                                   "review_note": mapping["review_note"], "fragments": fragments,
                                   "model_files": {model: {side: file_fingerprint(file)
                                                          for side, file in sides.items()}
                                                   for model, sides in files.items()}})
        for model, sides in files.items():
            for case in mapping["cases"]:
                cases_by_model[model].append({**case, **sides, "sha256": {
                    "las": case["sha256"]["las"],
                    **{side: file_fingerprint(file)["sha256"] for side, file in sides.items()}},
                    "tracking_policies": "strict frozen Python exports" if model == "python" else "saved original vectors"})
    _write_json(path/"report.json", report)
    for model, cases in cases_by_model.items():
        if progress:
            progress(f"Evaluating {model} predictions")
        result = run_evaluation(cases, path/model, chunk_size=chunk_size, point_outcomes=point_outcomes,
                                context=plan["context"], progress=progress)
        report["models"][model] = {"report": file_fingerprint(path/model/"report.json"),
                                   "settings": result["settings"], "source_fingerprint": result["source_fingerprint"]}
        report["failures"].extend({"model": model, **failure} for failure in result["failures"])
        for case in result["cases"]:
            report["cases"].append({"model": model, "case_id": case["case_id"], "path_id": case["path_id"],
                                     "status": case["status"], **case["counts"], **case["coverage"],
                                     **case["metrics"], **coverage_diagnostics(case["counts"], case["coverage"])})
        for totals in result["path_totals"]:
            report["paths"].append({"model": model, **totals, **coverage_diagnostics(totals, totals)})
        _write_json(path/"report.json", report)
    # Successful paired rows must describe exactly the same reference records.
    for mapping in plan["mappings"]:
        for cid in mapping["case_ids"]:
            pair = [row for row in report["cases"] if row["case_id"] == cid]
            if len(pair) != 2 or any(pair[0][key] != pair[1][key] for key in ("total", "positive", "background")):
                report["failures"].append({"case_id": cid, "error": "incomplete or inconsistent paired reference scope"})
    report["status"] = "failed" if report["failures"] else "completed"
    for name in ("cases", "paths"):
        rows = report[name]
        with (path/f"{name}.csv").open("x", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=list(rows[0]) if rows else ["model"])
            writer.writeheader()
            # CSV has no JSON NaN annotations; use empty fields for unavailable numbers.
            writer.writerows({key: None if isinstance(value, float) and not np.isfinite(value) else value
                              for key, value in row.items()} for row in rows)
    _write_json(path/"report.json", report)
    if overlay and report["status"] == "completed":
        from .overlay import plot_matched_evaluation
        try:
            report["overlays"] = plot_matched_evaluation(path/"report.json")
        except (OSError, ValueError, ImportError) as error:
            report["status"] = "failed"
            report["failures"].append({"stage": "overlay", "error": str(error)})
        _write_json(path/"report.json", report)
    return report
