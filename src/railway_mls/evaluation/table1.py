"""Table I-style reporting over reviewed frozen predictions, with source metrics.

Combining annotation parts and pooling selected paths are explicit reporting
policies. The native GUI did not define a multi-file or cross-path Total row.
"""

import csv
from dataclasses import asdict
import math
from pathlib import Path
import shutil
from tempfile import TemporaryDirectory

import numpy as np

from ..prediction.benchmark import _contract, _exports, _ledger, _new_output
from .audit import source_fingerprint
from .core import _read_residuals, evaluate_chunks, source_error_statistics
from .io import file_fingerprint, iter_reference_las, read_rail_segments
from .matched import (_json, _object, _pinned, coverage_diagnostics, load_matched_plan,
                      run_matched_evaluation)
from .runner import _write_json
from .types import EvaluationConfig, ReferencePoints


COUNTS = ("nTP", "nFP", "nTN", "nFN")
COVERAGE = ("total", "positive", "background", "assigned_positive", "assigned_background",
            "unassigned_positive", "unassigned_background")
ERRORS = ("mean_horizontal", "std_horizontal", "mean_absolute_vertical", "std_vertical_source")
POLICIES = {
    "parts": "Full declared parts in manifest order, no sampling/deduplication/cropping; raw class byte 3 is positive",
    "statistics": "Re-evaluate concatenated parts; source segment order, then part/local-record order; never average case standard deviations",
    "vertical": "Absolute vertical mean; source STD uses signed residuals around that absolute mean, with N-1 denominator",
    "length": "Report-only commented source diagnostic: half the sum of left/right XY segment lengths, once per model/path; no fragment bridges or reference-extent trimming",
    "source_precision_percent": "100*nTP/(nTP+nFP); assigned-positive success percentage",
    "source_recall_percent": "100*nTP/(nTP+nFN); source-printed name, not the paper's recall protocol",
    "coverage": "Assignment = assigned positives/all positives; all-positive success = nTP/all positives; separate from native percentages",
    "pool": "Selected-path report summary only: retain each path's own assignment, pool its residuals in path/segment/part/record order; not a native cross-path output",
    "scope": "Complete scoring scope does not establish complete spatial coverage, native whole-application equivalence or published-result reproduction",
}


def segment_length_xy(left, right):
    """Literal ordering of the source's commented length diagnostic."""
    length = 0.0
    for rail in (left, right):
        for begin, end in rail.endpoints:
            dx, dy = float(end[0]-begin[0]), float(end[1]-begin[1])
            length += math.sqrt(dx*dx+dy*dy)
    return length/2


def iter_path_reference(parts, chunk_size=250_000):
    """Rebase original zero-based IDs using declared part intervals, retaining XYZ/labels."""
    for part in parts:
        path = part["las"]["path"]
        _pinned(path, part["las"]["sha256"])
        loaded = 0
        for chunk in iter_reference_las(path, chunk_size):
            if len(chunk) and (chunk.record_ids[0] != loaded or chunk.record_ids[-1] != loaded+len(chunk)-1):
                raise ValueError("full reference parts require contiguous original LAS record IDs")
            if part["record_begin"]+loaded+len(chunk) > np.iinfo(np.int64).max:
                raise ValueError("combined reference record IDs exceed int64 capacity")
            yield ReferencePoints(chunk.xyz, chunk.raw_classification, chunk.record_ids+part["record_begin"])
            loaded += len(chunk)
        if loaded != part["record_end"]-part["record_begin"]:
            raise ValueError(f"{part['case_id']}: reference record count changed since part evaluation")
        _pinned(path, part["las"]["sha256"])


class _ResidualPool:
    """Disk spool of assigned positives; two passes preserve source reduction order."""

    def __init__(self, directory):
        self.directory = Path(directory)
        self.directory.mkdir()
        self.active = set()

    def append(self, path_id, rows):
        keep = (rows["raw_classification"] == 3) & (rows["segment_id"] >= 0)
        positives = rows[keep]
        for segment in np.unique(positives["segment_id"]):
            group = positives[positives["segment_id"] == segment]
            identity = (path_id, int(segment))
            self.active.add(identity)
            with (self.directory/f"{identity[0]}-{identity[1]}").open("ab") as stream:
                np.column_stack((group["horizontal"], group["signed_vertical"])).tofile(stream)

    def blocks(self):
        for path_id, segment in sorted(self.active):
            yield from _read_residuals(self.directory/f"{path_id}-{segment}")


def _record_map(mapping, part_report):
    parts, offset = [], 0
    for case in mapping["cases"]:
        rows = [r for r in part_report["cases"] if r["case_id"] == case["case_id"]]
        if len(rows) != 2 or rows[0]["total"] != rows[1]["total"]:
            raise ValueError("each part requires a complete paired evaluation before aggregation")
        count = rows[0]["total"]
        parts.append({"case_id": case["case_id"], "path_id": mapping["path_id"],
                      "record_begin": offset, "record_end": offset+count,
                      "las": _pinned(case["las"], case["sha256"]["las"])})
        offset += count
    return parts


def _metrics_row(pid, model, result, length):
    row = {"path_id": pid, "model": model, "evaluation_status": result.status, "length_xy_m": length,
           **result.counts, **result.coverage, **result.metrics,
           **coverage_diagnostics(result.counts, result.coverage)}
    for name in ERRORS:
        value = row.pop(name)
        row[name+"_m"] = value
        row[name+"_cm"] = value*100
    return row


def _historical(path):
    document, fingerprint = _json(path)
    _object(document, ("schema_version", "kind", "source", "rows"), "historical table")
    if (document["schema_version"], document["kind"]) != (1, "historical_table1"):
        raise ValueError("expected schema 1 historical_table1")
    source = document["source"]
    _object(source, ("path", "sha256", "description"), "historical source")
    source_fp = _pinned(Path(path).resolve().parent/source["path"], source["sha256"])
    seen = set()
    for row in document["rows"]:
        _object(row, ("path_id", "length_m", "mean_horizontal_cm", "std_horizontal_cm",
                      "signed_mean_vertical_cm", "std_vertical_cm", "reported_recall_percent"), "historical row")
        pid = row["path_id"]
        if isinstance(pid, bool) or not isinstance(pid, int) or pid <= 0 or pid in seen:
            raise ValueError("historical path IDs must be unique positive integers")
        seen.add(pid)
        if any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v)
               for key, v in row.items() if key != "path_id"):
            raise ValueError("historical table entries must be finite numeric values")
    return {"manifest": fingerprint, "source": source_fp, "description": source["description"],
            "interpretation": "Published rounded values under the historical protocol; never substituted into measured rows",
            "rows": document["rows"]}


def _number(value, decimals):
    if value is None or not math.isfinite(value):
        return "N/A"
    return f"{value:.{decimals}f}"


def _markdown(report):
    paths = ', '.join(map(str, report["requested_path_ids"]))
    lines = [f"# Table I-style results — path(s) {paths}", "",
             f"Report status: **{report['status']}**. Source profile: `source2018`. "
             "Results below use complete declared reference parts and the frozen mapped predictions.", "",
             "| Path | Model | Prediction status | Length XY (m) | Horizontal mean ± STD (cm) | Absolute vertical mean ± source STD (cm) | Precision (source, %) | Recall (source, %) |",
             "|---|---|---|---:|---:|---:|---:|---:|"]
    for row in report["rows"]:
        lines.append(f"| {row['path_id']} | {row['model']} | {row['prediction_status']} | {_number(row['length_xy_m'], 3)} | "
                     f"{_number(row['mean_horizontal_cm'], 1)} ± {_number(row['std_horizontal_cm'], 1)} | "
                     f"{_number(row['mean_absolute_vertical_cm'], 1)} ± {_number(row['std_vertical_source_cm'], 1)} | "
                     f"{_number(row['source_precision_percent'], 2)} | {_number(row['source_recall_percent'], 2)} |")
    lines += ["", "Coverage diagnostics (additional to the source percentages):", "",
              "| Path | Model | Reference positives | Assigned | Unassigned | Assignment (%) | Successful / all positives (%) |",
              "|---|---|---:|---:|---:|---:|---:|"]
    for row in report["rows"]:
        lines.append(f"| {row['path_id']} | {row['model']} | {row['positive']:,} | {row['assigned_positive']:,} | "
                     f"{row['unassigned_positive']:,} | {_number(row['positive_assignment_percent'], 2)} | "
                     f"{_number(row['all_positive_success_percent'], 2)} |")
    if len(report["requested_path_ids"]) > 1 and report["selected_path_summary"]:
        lines += ["", f"Report-level pooled summary for selected paths {paths}:", "",
                  "| Model | Length XY (m) | Horizontal mean ± STD (cm) | Absolute vertical mean ± source STD (cm) | Precision (source, %) | Recall (source, %) |",
                  "|---|---:|---:|---:|---:|---:|"]
        for model, row in report["selected_path_summary"].items():
            lines.append(f"| {model} | {_number(row['length_xy_m'], 3)} | "
                         f"{_number(row['mean_horizontal']*100, 1)} ± {_number(row['std_horizontal']*100, 1)} | "
                         f"{_number(row['mean_absolute_vertical']*100, 1)} ± {_number(row['std_vertical_source']*100, 1)} | "
                         f"{_number(row['source_precision_percent'], 2)} | {_number(row['source_recall_percent'], 2)} |")
    lines += ["", "Notes:", ""]
    lines += [f"- {value}." for value in report["policies"].values()]
    lines += ["- The selected-path summary pools measured residuals in its declared scope; it is not a native Total row.",
              "- Error statistics include all assigned positives, including failed accuracy checks. Unassigned positives are excluded from native percentages/statistics."]
    for path in report["path_details"]:
        lines.append(f"- Path {path['path_id']}: parts {', '.join(path['case_ids'])}; "
                     f"source statuses {', '.join(path['run_statuses'])}; limitations {', '.join(path['limitations'])}.")
    lines += ["", "Eight unrequested paths are outside this report." if len(report["unrequested_path_ids"]) == 8
              else f"Unrequested experiment paths: {report['unrequested_path_ids']}."]
    if report.get("historical"):
        lines += ["", "## Historical paper comparison (different protocol)", "",
                  report["historical"]["description"], "",
                  "| Path | Published length (m) | Horizontal mean ± STD (cm) | Signed vertical mean ± STD (cm) | Published recall (%) |",
                  "|---|---:|---:|---:|---:|"]
        for row in report["historical"]["rows"]:
            if row["path_id"] in report["requested_path_ids"]:
                lines.append(f"| {row['path_id']} | {_number(row['length_m'], 3)} | "
                             f"{_number(row['mean_horizontal_cm'], 1)} ± {_number(row['std_horizontal_cm'], 1)} | "
                             f"{_number(row['signed_mean_vertical_cm'], 1)} ± {_number(row['std_vertical_cm'], 1)} | "
                             f"{_number(row['reported_recall_percent'], 2)} |")
        lines += ["", report["historical"]["interpretation"]+"."]
    if report.get("overlays"):
        lines += ["", "## Inspection overlays", ""]
        lines += [f"![Path {item['path_id']}](parts/{Path(item['path']).name})" for item in report["overlays"]]
    if report["failures"]:
        lines += ["", "## Failures", ""]+[f"- {failure}" for failure in report["failures"]]
    return '\n'.join(lines)+'\n'


def _save(output, report):
    (output/"table1.md").write_text(_markdown(report), encoding="utf-8")
    with (output/"table1.csv").open("w", newline="") as stream:
        rows = report["rows"]
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]) if rows else ["path_id", "model"])
        writer.writeheader()
        writer.writerows({key: None if isinstance(value, float) and not math.isfinite(value) else value
                          for key, value in row.items()} for row in rows)
    with (output/"selected-path-summary.csv").open("w", newline="") as stream:
        rows = [{"model": model, **{key: ';'.join(map(str, value)) if key == "scope_path_ids" else value
                                  for key, value in summary.items()}}
                for model, summary in report["selected_path_summary"].items()]
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]) if rows else ["model", "scope_path_ids"])
        writer.writeheader()
        writer.writerows({key: None if isinstance(value, float) and not math.isfinite(value) else value
                          for key, value in row.items()} for row in rows)
    report["artifacts"] = {name: file_fingerprint(output/name) for name in ("table1.md", "table1.csv", "selected-path-summary.csv")}
    _write_json(output/"report.json", report)


def run_table1_report(experiment, predictions, mapping, output, *, path_ids, chunk_size=250_000,
                      overlay=False, historical=None, progress=None):
    """Re-evaluate complete selected paths; preserve all frozen source stops and gaps."""
    if not predictions or not path_ids or chunk_size <= 0:
        raise ValueError("supply prediction batches, explicit path IDs and a positive chunk size")
    contract = _contract(experiment)
    sources, records = _exports(contract, predictions)
    plan = load_matched_plan(mapping, path_ids=path_ids)
    selected = sorted(path_ids)
    if plan["reference_manifest"]["sha256"] != contract["reference_manifest"]["sha256"]:
        raise ValueError("mapping reference manifest differs from experiment")
    for source in plan["sources"].values():
        if (source["manifest"]["sha256"] not in sources
                or source["inference_manifest"]["sha256"] != contract["manifest"]["sha256"]):
            raise ValueError("mapping must use the supplied frozen experiment batches")
    for assignment in plan["mappings"]:
        if any(contract["path_by_run"].get(f["run_id"]) != assignment["path_id"] for f in assignment["fragments"]):
            raise ValueError("reviewed route differs from intended experiment path")
    history = _historical(historical) if historical else None
    if overlay:
        from .overlay import require_plotting
        require_plotting()
    output = _new_output(output, contract, [*predictions, *[s["directory"] for s in plan["sources"].values()]])
    ledger = _ledger(contract, records)
    report = {"schema_version": 1, "kind": "table1_source_report", "status": "running",
              "requested_path_ids": selected, "expected_path_ids": list(contract["experiment"].evaluation.expected_path_ids),
              "unrequested_path_ids": sorted(set(range(1, 10))-set(selected)),
              "complete_requested_scope": False, "complete_experiment_scope": False,
              "full_spatial_coverage": False, "tracking_executed": False, "settings": asdict(EvaluationConfig()),
              "units": {"length": "m", "row_errors": "explicit _m and _cm suffixes",
                        "selected_path_summary_errors": "m", "percentages": "percent"},
              "source_baseline": contract["experiment"].inference.source_baseline,
              "evaluation_source_fingerprint": source_fingerprint(), "policies": POLICIES,
              "experiment_manifest": contract["manifest"], "reference_manifest": contract["reference_manifest"],
              "prediction_exports": list(sources.values()), "mapping": file_fingerprint(mapping),
              "experiment_paths": ledger["paths"], "rows": [], "path_details": [],
              "selected_path_summary": {}, "failures": []}
    if history:
        report["historical"] = history
        shutil.copyfile(history["manifest"]["path"], output/"historical-snapshot.json")
        _pinned(output/"historical-snapshot.json", history["manifest"]["sha256"])
    _save(output, report)
    try:
        part_report = run_matched_evaluation(mapping, output/"parts", path_ids=selected,
                                             chunk_size=chunk_size, overlay=overlay, progress=progress)
    except (OSError, ValueError, ImportError) as error:
        report.update(status="failed", failures=[{"stage": "part_evaluation", "error": str(error)}])
        _save(output, report)
        return report
    report["part_report"] = file_fingerprint(output/"parts/report.json")
    report["overlays"] = part_report.get("overlays", [])
    if part_report["failures"]:
        report.update(status="failed", failures=part_report["failures"])
        _save(output, report)
        return report
    with TemporaryDirectory(prefix="railway-table1-") as temporary:
        pools = {model: _ResidualPool(Path(temporary)/model) for model in ("python", "original")}
        for assignment in sorted(plan["mappings"], key=lambda m: m["path_id"]):
            pid = assignment["path_id"]
            directory = output/f"path-{pid}"
            directory.mkdir()
            try:
                parts = _record_map(assignment, part_report)
                _write_json(directory/"reference-record-map.json", {
                    "identity": "path_record_id = record_begin + original zero-based LAS record ID", "parts": parts})
                files = next(item["model_files"] for item in part_report["mappings"] if item["path_id"] == pid)
                details = {"path_id": pid, "case_ids": assignment["case_ids"], "review_note": assignment["review_note"],
                           "record_map": file_fingerprint(directory/"reference-record-map.json"), "combined_models": {},
                           "run_statuses": [f["run_summary"]["status"] for f in assignment["fragments"]],
                           "limitations": sorted({"LIM-012"} | {lim for f in assignment["fragments"]
                                                   for lim in f["run_summary"].get("limitations_encountered", [])}),
                           "fragments": [{"source_id": f["source_id"], "run_id": f["run_id"], "layer": f["layer"],
                                          "run_manifest": f["run_manifest"], "vectors": f["vectors"],
                                          "run_summary": f["run_summary"]} for f in assignment["fragments"]]}
                report["path_details"].append(details)
                for model in ("python", "original"):
                    if progress:
                        progress(f"Path {pid}: recomputing combined-part {model} statistics")
                    rails = tuple(read_rail_segments(_pinned(files[model][side]["path"], files[model][side]["sha256"])["path"])
                                  for side in ("left", "right"))
                    result = evaluate_chunks(iter_path_reference(parts, chunk_size), *rails,
                                             point_callback=lambda rows, model=model, pid=pid: pools[model].append(pid, rows))
                    independent = [r for r in part_report["cases"] if r["path_id"] == pid and r["model"] == model]
                    for key in COUNTS+COVERAGE:
                        actual = result.counts[key] if key in COUNTS else result.coverage[key]
                        if actual != sum(r[key] for r in independent):
                            raise ValueError(f"Path {pid}: combined {key} differs from independent part totals")
                    _write_json(directory/f"{model}-combined.json", result.summary())
                    details["combined_models"][model] = file_fingerprint(directory/f"{model}-combined.json")
                    row = _metrics_row(pid, model, result, segment_length_xy(*rails))
                    row["prediction_status"] = ("partial_prediction" if any(s not in ("completed", "completed_with_limitations")
                                                                           for s in details["run_statuses"])
                                                  else "workflow_completed") if model == "python" else "saved_original_vectors"
                    row["native_tracking_comparability"] = all(f["run_summary"].get("reference_comparable", False)
                                                                 for f in assignment["fragments"]) if model == "python" else None
                    report["rows"].append(row)
                context = next(p for p in report["experiment_paths"] if p["path_id"] == pid)
                context.update(layer_review_status="reviewed", evaluation_status="evaluated")
                context.update({model: next(r for r in report["rows"] if r["path_id"] == pid and r["model"] == model)
                                for model in ("python", "original")})
            except (OSError, ValueError, ImportError) as error:
                report["failures"].append({"path_id": pid, "error": str(error)})
            _save(output, report)
        if not report["failures"]:
            for model, pool in pools.items():
                rows = [r for r in report["rows"] if r["model"] == model]
                totals = {key: sum(row[key] for row in rows) for key in COUNTS+COVERAGE}
                errors = source_error_statistics(pool.blocks, totals["assigned_positive"])
                with np.errstate(all="ignore"):
                    precision = float(np.divide(totals["nTP"], totals["nTP"]+totals["nFP"])*100)
                    recall = float(np.divide(totals["nTP"], totals["nTP"]+totals["nFN"])*100)
                report["selected_path_summary"][model] = {"scope_path_ids": selected, **totals,
                    "length_xy_m": sum(row["length_xy_m"] for row in rows), **errors,
                    "source_precision_percent": precision, "source_recall_percent": recall,
                    **coverage_diagnostics(totals, totals)}
    report["complete_requested_scope"] = not report["failures"] and len(report["rows"]) == 2*len(selected)
    report["complete_experiment_scope"] = report["complete_requested_scope"] and selected == list(range(1, 10))
    report["status"] = "completed" if report["complete_requested_scope"] else "failed"
    _save(output, report)
    return report
