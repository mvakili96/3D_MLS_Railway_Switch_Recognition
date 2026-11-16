"""Nine-path experiment accounting around the unchanged tracker/evaluator.

Missing starts, stopped runs and unreviewed layers remain rows, not zero scores.
Annotation values are read only by the separate matched evaluation stage.
"""

import csv
import math
from pathlib import Path
import shutil

from ..evaluation.io import file_fingerprint
from ..evaluation.matched import _json, _pinned, load_matched_plan, run_matched_evaluation
from ..evaluation.runner import load_cases
from .catalog import write_json_new
from .manifest import load_prediction_manifest
from .runner import _write_report, run_predictions


def _contract(manifest):
    experiment = load_prediction_manifest(manifest)
    if experiment.evaluation.expected_path_ids != tuple(range(1, 10)):
        raise ValueError("nine-path benchmark requires expected path IDs 1 through 9")
    cases, context = load_cases(experiment.evaluation.reference_manifest)
    if set(context["expected_case_ids_by_path"]) != set(map(str, range(1, 10))):
        raise ValueError("reference manifest must account for exactly paths 1 through 9")
    by_run = {}
    for mapping in experiment.evaluation.mappings:
        if mapping.path_id is None:
            raise ValueError("each benchmark run needs an intended path ID before prediction")
        if mapping.run_id in by_run and by_run[mapping.run_id] != mapping.path_id:
            raise ValueError("one independent benchmark run cannot target several paths")
        by_run[mapping.run_id] = mapping.path_id
    if set(by_run.values()) != set(range(1, 10)):
        raise ValueError("declare at least one ready or pending run for every path")
    if any(run.branch_policy != "strict" for run in experiment.inference.runs):
        raise ValueError("primary benchmark requires strict branches, including pending runs")
    return {"experiment": experiment, "cases": cases, "context": context, "path_by_run": by_run,
            "manifest": file_fingerprint(experiment.path),
            "reference_manifest": file_fingerprint(experiment.evaluation.reference_manifest)}


def _ledger(contract, records=()):
    known = {row["run_id"]: row for row in records}
    paths, cases = [], []
    for pid in range(1, 10):
        runs = [r for r in contract["experiment"].inference.runs if contract["path_by_run"][r.run_id] == pid]
        missing = [r.run_id for r in runs if r.status == "pending"]
        attempts = [known[r.run_id] for r in runs if r.run_id in known]
        geometry = [r for r in attempts if any(all(layer.get("vertices", {}).get(side, 0) > 1
                    for side in ("rail1", "rail2")) for layer in r.get("layers", []))]
        partial = any(r["status"] not in ("completed", "completed_with_limitations") for r in attempts)
        status = (("partial_prediction" if partial else "prediction_available") if geometry else "prediction_failed" if attempts
                  else "not_run" if any(r.status == "ready" for r in runs) else "missing_seed")
        review = "pending" if geometry else "unavailable"
        ids = contract["context"]["expected_case_ids_by_path"][str(pid)]
        path = {"path_id": pid, "run_ids": [r.run_id for r in runs], "missing_seed_runs": missing,
                "attempted_run_ids": [r["run_id"] for r in attempts], "prediction_status": status,
                "attempt_statuses": {r["run_id"]: r["status"] for r in attempts},
                "case_ids": ids, "layer_review_status": review,
                "evaluation_status": "not_evaluated", "python": None, "original": None}
        path.update(extraction_seconds=sum(r.get("elapsed_seconds", 0) for r in attempts) if attempts else None,
                    observed_peak_rss_bytes=max((r.get("resources", {}).get("observed_peak_rss_bytes", 0)
                                                 for r in attempts), default=None),
                    limitations_encountered=sorted({lim for r in attempts for lim in r.get("limitations_encountered", [])}))
        paths.append(path)
        for cid in ids:
            cases.append({"case_id": cid, "path_id": pid, "prediction_status": status,
                          "layer_review_status": review, "evaluation_status": "not_evaluated",
                          "reason": "awaiting_layer_review" if geometry else status,
                          "python": None, "original": None})
    return {"schema_version": 1, "kind": "nine_path_benchmark", "status": "partial",
            "experiment_manifest": contract["manifest"], "reference_manifest": contract["reference_manifest"],
            "source_baseline": contract["experiment"].inference.source_baseline,
            "profile": "source2018", "paths": paths, "cases": cases,
            "accounted_path_count": len(paths), "accounted_case_count": len(cases),
            "all_paths_have_seeds": all(not p["missing_seed_runs"] for p in paths),
            "all_reference_cases_evaluated": False, "full_spatial_coverage": False,
            "scope_note": "Accounting for all parts does not establish prediction coverage. Missing scores are unavailable, not zero.",
            "prediction_exports": [], "run_summaries": list(records), "failures": []}


def _save(output, report):
    output = Path(output)
    _write_report(output/"report.json", report)
    for name, identities in (("paths", ["path_id"]), ("cases", ["case_id", "path_id"])):
        fields = identities+["prediction_status", "layer_review_status", "evaluation_status", "reason"]
        if name == "paths":
            fields += ["attempt_statuses", "extraction_seconds", "observed_peak_rss_bytes"]
        model_fields = ("nTP", "nFP", "nTN", "nFN", "positive", "assigned_positive", "unassigned_positive",
                        "positive_assignment_percent", "all_positive_success_percent",
                        "source_precision_percent", "source_recall_percent")
        fields += [f"{model}_{field}" for model in ("python", "original") for field in model_fields]
        with (output/f"{name}.csv").open("w", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=fields)
            writer.writeheader()
            for row in report[name]:
                flat = {key: row.get(key) for key in identities+["prediction_status", "layer_review_status", "evaluation_status", "reason"]}
                if name == "paths":
                    flat.update(attempt_statuses=str(row["attempt_statuses"]), extraction_seconds=row["extraction_seconds"],
                                observed_peak_rss_bytes=row["observed_peak_rss_bytes"])
                for model in ("python", "original"):
                    flat.update({f"{model}_{field}": (row.get(model) or {}).get(field) for field in model_fields})
                writer.writerow({key: None if isinstance(value, float) and not math.isfinite(value) else value
                                 for key, value in flat.items()})


def _new_output(output, contract, protected=()):
    output = Path(output).resolve()
    parents = [Path(contract["context"]["dataset_root"]), contract["experiment"].inference.data_root,
               *map(Path, protected)]
    if any(output == parent.resolve() or parent.resolve() in output.parents for parent in parents):
        raise ValueError("benchmark output must be outside input, cache and frozen prediction directories")
    output.mkdir(parents=True, exist_ok=False)
    shutil.copyfile(contract["manifest"]["path"], output/"experiment-snapshot.json")
    shutil.copyfile(contract["reference_manifest"]["path"], output/"reference-snapshot.json")
    _pinned(output/"experiment-snapshot.json", contract["manifest"]["sha256"])
    _pinned(output/"reference-snapshot.json", contract["reference_manifest"]["sha256"])
    return output


def plan_benchmark(manifest, output):
    """Write all expected cases without opening raw/reference point payloads."""
    contract = _contract(manifest)
    output = _new_output(output, contract)
    report = _ledger(contract)
    report["stage"] = "planning"
    report["reference_point_records_read"] = False
    _save(output, report)
    return report


def _mapping_template(contract, sources):
    return {"schema_version": 1, "kind": "matched_prediction_evaluation", "profile": "source2018",
            "reference_manifest": {"path": contract["reference_manifest"]["path"],
                                   "sha256": contract["reference_manifest"]["sha256"]},
            "prediction_sources": sources,
            "paths": [{"path_id": pid, "status": "pending", "fragments": [],
                       "case_ids": contract["context"]["expected_case_ids_by_path"][str(pid)],
                       "review_note": "Review frozen run/layer geometry before assigning fragments; never choose by score."}
                      for pid in range(1, 10)]}


def run_benchmark(manifest, cache, output, *, run_ids=None, max_steps=None,
                  retain_history=False, progress=None):
    """Attempt ready declared starts, keeping every missing path/part in the ledger."""
    contract = _contract(manifest)
    declared = {r.run_id: r for r in contract["experiment"].inference.runs}
    if run_ids is not None and (not run_ids or len(run_ids) != len(set(run_ids))
                               or any(r not in declared or declared[r].status != "ready" for r in run_ids)):
        raise ValueError("select unique declared ready benchmark runs")
    ready = [r.run_id for r in declared.values() if r.status == "ready" and (run_ids is None or r.run_id in run_ids)]
    if max_steps is not None and (isinstance(max_steps, bool) or not isinstance(max_steps, int) or max_steps < 0):
        raise ValueError("max_steps must be a nonnegative integer")
    output = _new_output(output, contract, [cache])
    report = _ledger(contract)
    report.update(stage="prediction", reference_point_records_read=False)
    _save(output, report)
    predictions = None
    if ready:
        try:
            predictions = run_predictions(manifest, cache, output/"predictions", run_ids=ready,
                                           max_steps=max_steps, retain_history=retain_history, progress=progress)
        except (OSError, ValueError, ImportError, MemoryError) as error:
            report["failures"].append({"stage": "prediction", "error": str(error)})
            report["status"] = "failed"
            _save(output, report)
            return report
        except KeyboardInterrupt:
            report["failures"].append({"stage": "prediction", "error": "interrupted before a frozen prediction export was returned"})
            report["status"] = "interrupted"
            _save(output, report)
            return report
        report = {**report, **_ledger(contract, predictions["runs"])}
        report["stage"] = "prediction"
        report["reference_point_records_read"] = False
        report["prediction_exports"] = [file_fingerprint(output/"predictions/manifest.json")]
    sources, candidates = [], []
    if predictions:
        sources.append({"source_id": "batch", "directory": str(output/"predictions"),
                        "manifest_sha256": report["prediction_exports"][0]["sha256"],
                        "inference_manifest": str(contract["experiment"].path)})
        for run in predictions["runs"]:
            leaf = output/"predictions"/run["run_id"]/"manifest.json"
            if not leaf.is_file():
                continue  # Admission failures have no run directory or invented geometry.
            for layer in run.get("layers", []):
                candidates.append({"intended_path_id": contract["path_by_run"][run["run_id"]],
                                   "run_status": run["status"], "vertices": layer["vertices"],
                                   "fragment": {"source_id": "batch", "run_id": run["run_id"], "layer": layer["layer"],
                                                "run_manifest_sha256": file_fingerprint(leaf)["sha256"]}})
    write_json_new(output/"mapping-template.json", _mapping_template(contract, sources))
    write_json_new(output/"layer-candidates.json", {"status": "requires_explicit_review", "candidates": candidates})
    _save(output, report)
    return report


def _exports(contract, directories):
    sources, records = {}, {}
    for directory in directories:
        directory = Path(directory).resolve()
        report, _ = _json(directory/"report.json")
        if (report.get("kind") != "nine_path_benchmark" or report.get("stage") != "prediction"
                or report["experiment_manifest"]["sha256"] != contract["manifest"]["sha256"]
                or report["reference_manifest"]["sha256"] != contract["reference_manifest"]["sha256"]):
            raise ValueError("prediction batch does not belong to this frozen benchmark definition/reference")
        for fingerprint in report["prediction_exports"]:
            _pinned(fingerprint["path"], fingerprint["sha256"])
            root, _ = _json(fingerprint["path"], fingerprint["sha256"])
            if root["manifest_sha256"] != contract["manifest"]["sha256"]:
                raise ValueError("prediction definition differs from benchmark")
            sources[fingerprint["sha256"]] = fingerprint
            for run in root["runs"]:
                if run["run_id"] in records:
                    raise ValueError("run attempted in several batches; declare separately named restarts instead of choosing a result")
                records[run["run_id"]] = run
    return sources, list(records.values())


def evaluate_benchmark(manifest, prediction_outputs, mapping, output, *, chunk_size=250_000,
                       point_outcomes=False, overlay=False, progress=None):
    """Score explicitly reviewed fragments, retaining missing paths as unavailable."""
    contract = _contract(manifest)
    if not prediction_outputs or chunk_size <= 0:
        raise ValueError("supply prediction batches and a positive chunk_size")
    sources, records = _exports(contract, prediction_outputs)
    plan = load_matched_plan(mapping)
    if plan["reference_manifest"]["sha256"] != contract["reference_manifest"]["sha256"]:
        raise ValueError("mapping references a different reference manifest")
    for source in plan["sources"].values():
        if (source["manifest"]["sha256"] not in sources
                or source["inference_manifest"]["sha256"] != contract["manifest"]["sha256"]):
            raise ValueError("mapping must use the supplied frozen benchmark batches")
    for assignment in plan["mappings"]:
        for fragment in assignment["fragments"]:
            if contract["path_by_run"].get(fragment["run_id"]) != assignment["path_id"]:
                raise ValueError("reviewed route differs from predeclared intended path; declare a separate experiment")
    output = _new_output(output, contract, prediction_outputs)
    report = _ledger(contract, records)
    report.update(stage="evaluation", prediction_exports=list(sources.values()),
                  mapping=file_fingerprint(mapping), reference_point_records_read=True)
    _save(output, report)
    try:
        matched = run_matched_evaluation(mapping, output/"matched", chunk_size=chunk_size,
                                         point_outcomes=point_outcomes, overlay=overlay, progress=progress)
    except (OSError, ValueError, ImportError) as error:
        report["status"] = "failed"
        report["failures"].append({"stage": "evaluation", "error": str(error)})
        _save(output, report)
        return report
    report["matched_report"] = file_fingerprint(output/"matched/report.json")
    report["failures"] = matched["failures"]
    for case in report["cases"]:
        if case["path_id"] in [m["path_id"] for m in matched["mappings"]]:
            case["layer_review_status"] = "reviewed"
        rows = [r for r in matched["cases"] if r["case_id"] == case["case_id"]]
        if len(rows) == 2:
            case.update(evaluation_status="evaluated", reason=None)
            for row in rows:
                case[row["model"]] = row
        elif case["path_id"] in [m["path_id"] for m in matched["mappings"]]:
            case.update(evaluation_status="evaluation_failed", reason="incomplete paired evaluation")
    for path in report["paths"]:
        if path["path_id"] in [m["path_id"] for m in matched["mappings"]]:
            path["layer_review_status"] = "reviewed"
        group = [c for c in report["cases"] if c["path_id"] == path["path_id"]]
        if all(c["evaluation_status"] == "evaluated" for c in group):
            path["evaluation_status"] = "evaluated"
        elif any(c["evaluation_status"] == "evaluation_failed" for c in group):
            path["evaluation_status"] = "evaluation_failed"
        for row in matched["paths"]:
            if row["path_id"] == path["path_id"]:
                path[row["model"]] = row
    report["all_reference_cases_evaluated"] = all(c["evaluation_status"] == "evaluated" for c in report["cases"])
    report["status"] = ("failed" if report["failures"] else "evaluated" if report["all_reference_cases_evaluated"] else "partial")
    _save(output, report)
    return report
