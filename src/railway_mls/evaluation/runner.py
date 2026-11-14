"""Manifest batches and inspectable artifacts for the source2018 profile."""

import csv
from dataclasses import asdict
import json
from pathlib import Path
import platform
import sys

import numpy as np

from ..artifacts import strict_json_value
from .audit import source_fingerprint
from .core import evaluate_chunks, evaluate_rails
from .history import reconcile_history
from .io import file_fingerprint, iter_reference_las, read_python_vectors, read_rail_segments
from .types import EvaluationConfig, RailSegments, ReferencePoints


def _write_json(path, payload):
    Path(path).write_text(json.dumps(strict_json_value(payload), indent=2, allow_nan=False)+"\n")


def _validate_identity(entry, seen):
    identity = entry["case_id"]
    if (not isinstance(identity, str) or not identity
            or any(c not in "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-" for c in identity)
            or identity in seen):
        raise ValueError("case IDs must be unique safe filename components")
    if not isinstance(entry["path_id"], int) or isinstance(entry["path_id"], bool):
        raise ValueError("path_id must be an integer")
    seen.add(identity)


def load_cases(manifest, *, data_root=None, selected=None):
    path = Path(manifest).resolve()
    document = json.loads(path.read_text())
    if document.get("schema_version") != 1 or document.get("profile") != "source2018":
        raise ValueError("expected evaluation manifest schema 1 and profile source2018")
    root = Path(data_root).resolve() if data_root else (path.parent/document["dataset_root"]).resolve()
    cases, seen, expected_by_path = [], set(), {}
    for entry in document["cases"]:
        _validate_identity(entry, seen)
        identity = entry["case_id"]
        expected_by_path.setdefault(str(entry["path_id"]), []).append(identity)
        if selected and identity not in selected:
            continue
        case = {**entry, **{key: root/entry[key] for key in ("las", "left", "right")}}
        cases.append(case)
    if selected and set(selected)-seen:
        raise ValueError(f"unknown case IDs: {sorted(set(selected)-seen)}")
    if not cases:
        raise ValueError("no evaluation cases selected")
    return cases, {"manifest": file_fingerprint(path), "dataset_root": str(root),
                   "scope": document.get("scope", "independent cases"),
                   "expected_case_ids_by_path": expected_by_path,
                   "case_selection": sorted(selected) if selected else "all",
                   "historical_workbook": root/document["historical_workbook"] if document.get("historical_workbook") else None,
                   "historical_outcomes": root/document["historical_outcomes"] if document.get("historical_outcomes") else None}


def run_evaluation(cases, output, *, chunk_size=250_000, point_outcomes=False,
                   context=None, historical=False, progress=None):
    """Process one case at a time. Failures are recorded, never represented as passes."""
    if chunk_size <= 0:
        raise ValueError("chunk_size must be positive")
    context = {} if context is None else dict(context)
    # Validate referenced files before creating an output directory.
    if not cases:
        raise ValueError("no evaluation cases")
    seen = set()
    for case in cases:
        _validate_identity(case, seen)
        keys = ("las", "python_vectors") if "python_vectors" in case else ("las", "left", "right")
        for key in keys:
            if not Path(case[key]).is_file():
                raise FileNotFoundError(case[key])
    path = Path(output)
    path.mkdir(parents=True, exist_ok=False)
    report = {"schema_version": 1, "status": "running", "settings": asdict(EvaluationConfig()),
              "source_fingerprint": source_fingerprint(), "context": {
                  key: str(value) if isinstance(value, Path) else value for key, value in context.items()},
              "runtime": {"python": platform.python_version(), "numpy": np.__version__},
              "chunk_size": chunk_size, "cases": [], "failures": [],
              "limitations": ["LIM-012"], "aggregation_scope": "independent annotation parts; overlaps retained",
              "metric_definitions": {
                  "nTP": "assigned reference-positive passes",
                  "nFP": "assigned reference-positive failures (conventional FN)",
                  "nFN": "assigned background passes (conventional FP)",
                  "nTN": "assigned background failures",
                  "source_precision_percent": "100*nTP/(nTP+nFP); assigned positive success percentage",
                  "source_recall_percent": "100*nTP/(nTP+nFN); source-printed name",
                  "unassigned": "excluded by source; reported as coverage, outcome 0 means unavailable native ClassCodeRe"}}
    _write_json(path/"report.json", report)
    for index, case in enumerate(cases):
        identity = case["case_id"]
        if progress:
            progress(f"[{index+1}/{len(cases)}] {identity}")
        try:
            fingerprints = {}
            for key in (("las", "python_vectors") if "python_vectors" in case else ("las", "left", "right")):
                fingerprints[key] = file_fingerprint(case[key])
                expected = case.get("sha256", {}).get(key)
                if expected and fingerprints[key]["sha256"] != expected:
                    raise ValueError(f"{identity}: input SHA256 mismatch for {key}")
            left, right = (read_python_vectors(case["python_vectors"], layer=case["layer"])
                           if "python_vectors" in case else
                           (read_rail_segments(case["left"]), read_rail_segments(case["right"])))
            stream = None
            try:
                callback = None
                if point_outcomes:
                    stream = (path/f"{identity}-points.csv").open("x", newline="")
                    writer = csv.writer(stream)
                    writer.writerow(("record_id", "segment_id", "raw_classification", "outcome", "horizontal", "signed_vertical"))
                    def callback(rows):
                        writer.writerows(rows.tolist())
                result = evaluate_chunks(iter_reference_las(case["las"], chunk_size), left, right,
                                         point_callback=callback)
            finally:
                if stream is not None:
                    stream.close()
            summary = {"case_id": identity, "path_id": case["path_id"], **result.summary(),
                       "inputs": fingerprints, "prediction_provenance": {
                           "left": left.provenance, "right": right.provenance,
                           "tracking_policies": case.get("tracking_policies", "not supplied")}}
            _write_json(path/f"{identity}.json", summary)
            report["cases"].append(summary)
        except (OSError, ValueError, ImportError) as error:
            report["failures"].append({"case_id": identity, "error": str(error)})
            if progress:
                progress(f"{identity}: {error}")
        _write_json(path/"report.json", report)
    report["status"] = "failed" if report["failures"] else "completed"
    report["path_totals"] = []
    for pid in sorted({r["path_id"] for r in report["cases"]}):
        group = [r for r in report["cases"] if r["path_id"] == pid]
        counts = {name: sum(r["counts"][name] for r in group) for name in ("nTP", "nFP", "nTN", "nFN")}
        coverage = {name: sum(r["coverage"][name] for r in group) for name in group[0]["coverage"]}
        with np.errstate(all="ignore"):
            precision = float(np.divide(counts["nTP"], counts["nTP"]+counts["nFP"])*100)
            recall = float(np.divide(counts["nTP"], counts["nTP"]+counts["nFN"])*100)
        report["path_totals"].append({"path_id": pid, "case_ids": [r["case_id"] for r in group],
                                      "complete_manifest_path": (set(r["case_id"] for r in group)
                                          == set(context["expected_case_ids_by_path"][str(pid)]))
                                          if context.get("expected_case_ids_by_path") else None,
                                      **counts, **coverage, "source_precision_percent": precision,
                                      "source_recall_percent": recall})
    if historical:
        try:
            if not context.get("historical_workbook") or not context.get("historical_outcomes"):
                raise ValueError("historical reconciliation requires workbook and TPFN paths in the manifest")
            report["historical_reconciliation"] = reconcile_history(
                report["cases"], context["historical_workbook"], context["historical_outcomes"],
                expected_case_ids=context.get("expected_case_ids_by_path"))
        except (OSError, ValueError, KeyError) as error:
            report["status"] = "failed"
            report["failures"].append({"stage": "historical_reconciliation", "error": str(error)})
    with (path/"cases.csv").open("x", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=["case_id", "path_id", "status", "nTP", "nFP", "nTN", "nFN",
                                                  *next(iter(report["cases"]), {"coverage": {}})["coverage"],
                                                  *next(iter(report["cases"]), {"metrics": {}})["metrics"]])
        writer.writeheader()
        for result in report["cases"]:
            writer.writerow({**{key: result[key] for key in ("case_id", "path_id", "status")},
                             **result["counts"], **result["coverage"], **result["metrics"]})
    with (path/"paths.csv").open("x", newline="") as stream:
        rows = report["path_totals"]
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]) if rows else ["path_id"])
        writer.writeheader()
        writer.writerows(rows)
    _write_json(path/"report.json", report)
    return report


def evaluation_demo():
    reference = ReferencePoints([[.5, 0, 0], [.6, .2, .06], [.7, .05, 0],
                                 [.8, .2, 0], [2, 0, 0]], np.array([3, 3, 1, 1, 3]))
    result = evaluate_rails(reference, RailSegments([[[0, 0, 0], [1, 0, 0]]]), RailSegments([]))
    return strict_json_value({"data": "generated evaluator fixture; no Vancouver data used",
                              "settings": asdict(EvaluationConfig()), **result.summary()})


def add_cli(commands):
    commands.add_parser("evaluation-demo", help="score a small generated source2018 evaluation fixture")
    audit = commands.add_parser("evaluation-audit", help="check the separate frozen 2018 evaluator source")
    audit.add_argument("--repo", default=".")
    parser = commands.add_parser("evaluate-rails", help="evaluate annotated LAS against ordered rail segments")
    parser.add_argument("--manifest")
    parser.add_argument("--data-root", help="override the manifest dataset root")
    parser.add_argument("--case", action="append", help="select a manifest case ID; repeat to select several")
    parser.add_argument("--las")
    parser.add_argument("--left")
    parser.add_argument("--right")
    parser.add_argument("--python-vectors", help="pipeline vectors.csv instead of native left/right files")
    parser.add_argument("--layer", type=int, help="required explicit layer for --python-vectors")
    parser.add_argument("--output", required=True)
    parser.add_argument("--chunk-size", type=int, default=250_000)
    parser.add_argument("--point-outcomes", action="store_true")
    parser.add_argument("--historical", action="store_true", help="reconcile saved workbook and TPFN evidence")


def run_cli(args, parser):
    from .audit import evaluation_audit
    if args.command == "evaluation-demo":
        print(json.dumps(evaluation_demo(), indent=2, allow_nan=False))
        return 0
    if args.command == "evaluation-audit":
        report = evaluation_audit(args.repo)
        print(json.dumps(report, indent=2, allow_nan=False))
        return 0 if report["status"] == "ok" else 1
    explicit = any((args.las, args.left, args.right, args.python_vectors, args.layer is not None))
    if args.manifest and explicit:
        parser.error("--manifest cannot be combined with explicit input files/layer")
    if not args.manifest and (args.data_root or args.case or args.historical):
        parser.error("--data-root, --case and --historical require --manifest")
    if args.chunk_size <= 0:
        parser.error("--chunk-size must be positive")
    try:
        if args.manifest:
            cases, context = load_cases(args.manifest, data_root=args.data_root, selected=args.case)
        else:
            if not args.las:
                parser.error("supply --manifest or --las with rail predictions")
            if args.python_vectors:
                if args.left or args.right or args.layer is None:
                    parser.error("--python-vectors requires --layer and cannot combine with --left/--right")
                case = {"python_vectors": args.python_vectors, "layer": args.layer}
            else:
                if not args.left or not args.right or args.layer is not None:
                    parser.error("--las requires both --left and --right, or --python-vectors with --layer")
                case = {"left": args.left, "right": args.right}
            cases, context = [{"case_id": "case", "path_id": 1, "las": args.las, **case}], {}
        report = run_evaluation(cases, args.output, chunk_size=args.chunk_size,
                                point_outcomes=args.point_outcomes, context=context,
                                historical=args.historical, progress=lambda message: print(message, file=sys.stderr))
    except (OSError, ValueError, ImportError, KeyError) as error:
        print(json.dumps({"status": "failed", "error": str(error)}), file=sys.stderr)
        return 1
    print(json.dumps(strict_json_value({"status": report["status"], "output": str(Path(args.output)/"report.json"),
                                       "cases_completed": len(report["cases"]), "failures": report["failures"],
                                       "path_totals": report["path_totals"]}), indent=2, allow_nan=False))
    return 0 if report["status"] == "completed" else 1
