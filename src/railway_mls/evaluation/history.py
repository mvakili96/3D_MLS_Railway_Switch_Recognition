"""Read historical evidence independently of reference labels and source metrics."""

from collections import Counter
from pathlib import Path
import math
from xml.etree import ElementTree as ET
from zipfile import ZipFile

from .io import file_fingerprint


def workbook_paths(path):
    """Read cached Vancouver rows of Evaluation.xlsx without Excel dependencies."""
    ns = {"x": "http://schemas.openxmlformats.org/spreadsheetml/2006/main"}
    with ZipFile(path) as archive:
        strings = []
        if "xl/sharedStrings.xml" in archive.namelist():
            for item in ET.fromstring(archive.read("xl/sharedStrings.xml")).findall("x:si", ns):
                strings.append("".join(item.itertext()))
        sheet = ET.fromstring(archive.read("xl/worksheets/sheet1.xml"))
        rows = {}
        for row in sheet.findall("x:sheetData/x:row", ns):
            number = int(row.attrib["r"])
            if not 5 <= number <= 13:
                continue
            cells = {}
            for cell in row.findall("x:c", ns):
                value = cell.find("x:v", ns)
                if value is not None:
                    text = value.text
                    if cell.attrib.get("t") == "s":
                        text = strings[int(text)]
                    cells["".join(c for c in cell.attrib["r"] if c.isalpha())] = text
            pid = int(cells["A"])
            rows[pid] = {"success": int(cells["G"]), "failure": int(cells["H"]),
                         "displayed_recall_percent": float(cells["I"]),
                         "mean_horizontal": float(cells["B"]),
                         "signed_mean_vertical": float(cells["D"]),
                         "length": float(cells["F"])}
    if set(rows) != set(range(1, 10)):
        raise ValueError("historical workbook must contain complete Vancouver rows 1–9")
    return rows


def saved_outcome_counts(path):
    """Historical XYZ + outcome 1/2. These codes are NOT raw reference labels."""
    counts = Counter()
    with Path(path).open() as stream:
        for number, line in enumerate(stream, 1):
            fields = line.split()
            if len(fields) != 4 or fields[-1] not in ("1", "2"):
                raise ValueError(f"{path}:{number}: expected XYZ and historical outcome 1/2")
            if not all(math.isfinite(float(value)) for value in fields[:3]):
                raise ValueError(f"{path}:{number}: nonfinite historical coordinates")
            counts[int(fields[-1])] += 1
    return {"success": counts[1], "failure": counts[2]}


def reconcile_history(results, workbook, outcomes_directory, *, expected_case_ids=None):
    saved = workbook_paths(workbook)
    by_path = []
    for pid in sorted({r["path_id"] for r in results}):
        cases = [r for r in results if r["path_id"] == pid]
        source_tp = sum(r["counts"]["nTP"] for r in cases)
        source_fp = sum(r["counts"]["nFP"] for r in cases)
        positives = sum(r["coverage"]["positive"] for r in cases)
        outcome_path = Path(outcomes_directory) / f"Path{pid}_TPFN.txt"
        outcomes = saved_outcome_counts(outcome_path)
        expected = saved[pid]
        by_path.append({"path_id": pid, "case_ids": [r["case_id"] for r in cases],
                        "complete_manifest_path": (set(r["case_id"] for r in cases)
                            == set(expected_case_ids[str(pid)])) if expected_case_ids else None,
                        "source_assigned_success": source_tp, "source_assigned_failure": source_fp,
                        "partwise_reference_positive": positives,
                        "historical": expected, "saved_outcomes": outcomes,
                        "outcome_fingerprint": file_fingerprint(outcome_path),
                        "saved_counts_match_workbook": all(outcomes[k] == expected[k] for k in outcomes),
                        "source_success_delta": source_tp-expected["success"],
                        "source_failure_delta": source_fp-expected["failure"],
                        "reference_denominator_delta": positives-expected["success"]-expected["failure"]})
    all_saved = list(saved.values())
    return {"workbook": file_fingerprint(workbook), "paths": by_path,
            "historical_nine_path_macro_displayed_recall_percent":
                sum(v["displayed_recall_percent"] for v in all_saved)/9,
            "historical_pooled_success_percent":
                100*sum(v["success"] for v in all_saved)/sum(v["success"]+v["failure"] for v in all_saved),
            "interpretation": "Historical evidence reconciliation, not a source-parity assertion. "
                "Source nFP counts assigned positive failures. Part-wise totals may include overlapping records. "
                "Historical counts cover full paths; deltas for incomplete case selections are not comparable. "
                "Thresholds, signed vertical statistics and artifact versions differ; see LIM-012."}
