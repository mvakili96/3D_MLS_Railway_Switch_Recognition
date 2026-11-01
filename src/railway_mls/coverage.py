"""Inspectable source dispositions and checks against the frozen local snapshot."""

from __future__ import annotations

from collections import Counter
from hashlib import sha256
from importlib import import_module, resources
import json
from pathlib import Path
import re


SOURCE_DIRECTORY = Path("ADE code/PCLProject/PCLProject/PCLProject")


def source_text_hash(text: str) -> str:
    """Hash source text with normalized newlines, avoiding Git CRLF differences."""
    return sha256(text.replace("\r\n", "\n").replace("\r", "\n").encode("utf-8")).hexdigest()


def cpp_definitions(text: str, owner: str) -> tuple[dict, ...]:
    """Find out-of-class definitions, excluding comment/string lookalikes.

    This intentionally inventories qualified definitions in two selected .cpp
    files, not arbitrary C++ syntax, preprocessing or a complete native call graph.
    """

    def blank(match):
        return re.sub(r"[^\n]", " ", match.group())

    stripped = re.sub(
        r'//[^\n]*|/\*[\s\S]*?\*/|"(?:\\.|[^"\\])*"|\'(?:\\.|[^\'\\])*\'', blank, text
    )
    pattern = re.compile(
        rf"(?m)^[ \t]*([^\n;{{}}]*?\b{re.escape(owner)}::(~?\w+)\s*\([^;{{}}]*\))\s*\{{"
    )
    counts, result = Counter(), []
    for match in pattern.finditer(stripped):
        name = match.group(2)
        counts[name] += 1
        result.append(
            {
                "id": f"{owner}::{name}#{counts[name]}",
                "name": name,
                "line": text.count("\n", 0, match.start()) + 1,
                "signature": " ".join(match.group(1).split()),
            }
        )
    return tuple(result)


def source_coverage() -> dict:
    """Read the packaged audit; it describes dispositions, not verified parity."""
    return json.loads(resources.files("railway_mls").joinpath("source_coverage.json").read_text())


def coverage_summary() -> dict:
    audit = source_coverage()
    entries = [entry for file in audit["definition_files"] for entry in file["definitions"]]
    counts = dict(sorted(Counter(entry["disposition"] for entry in entries).items()))
    return {
        "baseline_id": audit["baseline_id"],
        "definition_scope": audit["definition_scope"],
        "definition_count": len(entries),
        "dispositions": counts,
        "project_file_count": len(audit["files"]),
        "bundled_dependency_file_count": len(audit["bundled_dependencies"]),
        "pending_definitions": [
            {"source": file["path"], **entry}
            for file in audit["definition_files"]
            for entry in file["definitions"]
            if "pending" in entry["disposition"]
        ],
        "partially_ported_definitions": [
            {"source": file["path"], **entry}
            for file in audit["definition_files"]
            for entry in file["definitions"]
            if entry["disposition"] == "ported_reached_behavior"
        ],
        "pending_file_workflows": [
            item for item in audit["files"] if item["disposition"] == "io_or_conversion_pending"
        ],
        "complete_reconstruction": False,
    }


def check_source_coverage(repo: str | Path) -> tuple[str, ...]:
    """Read-only drift, classification and Python-target validation.

    Requires the original source checkout. Installed-wheel pipeline execution
    only needs the packaged manifest and never requires the native source.
    """
    audit, errors = source_coverage(), []
    root = Path(repo) / SOURCE_DIRECTORY
    for file in (*audit["files"], *audit["bundled_dependencies"]):
        path = root / file["path"]
        if not path.is_file():
            errors.append(f"missing source: {file['path']}")
        elif source_text_hash(path.read_text()) != file["sha256"]:
            errors.append(f"source changed: {file['path']}")
    actual_files = (
        {path.name for path in root.iterdir() if path.is_file() and path.suffix in (".cpp", ".h")}
        if root.is_dir()
        else set()
    )
    recorded_files = {file["path"] for file in audit["files"]}
    if actual_files != recorded_files:
        errors.append(f"file inventory differs: {sorted(actual_files ^ recorded_files)}")
    dependencies = {
        path.relative_to(root).as_posix()
        for directory in ("Jama", "etc")
        for path in (root / directory).rglob("*")
        if path.is_file() and path.suffix in (".cpp", ".h", ".inl")
    }
    recorded_dependencies = {file["path"] for file in audit["bundled_dependencies"]}
    if dependencies != recorded_dependencies:
        errors.append(
            f"dependency inventory differs: {sorted(dependencies ^ recorded_dependencies)}"
        )
    for file in audit["definition_files"]:
        path = root / file["path"]
        if not path.is_file():
            continue
        actual = cpp_definitions(path.read_text(), file["owner"])
        expected = file["definitions"]
        if [(x["id"], x["signature"]) for x in actual] != [
            (x["id"], x["signature"]) for x in expected
        ]:
            errors.append(f"definition inventory differs: {file['path']}")
        for entry in expected:
            if not entry.get("disposition") or not entry.get("note"):
                errors.append(f"unclassified: {entry['id']}")
            for target in entry["python"]:
                try:
                    module, _, attribute = target.rpartition(":")
                    obj = import_module(module)
                    for part in attribute.split("."):
                        obj = getattr(obj, part)
                except (ImportError, AttributeError):
                    errors.append(f"missing Python target: {target}")
    return tuple(errors)
