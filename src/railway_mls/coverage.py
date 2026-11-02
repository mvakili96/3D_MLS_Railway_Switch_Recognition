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


def _cpp_code(text: str) -> str:
    def blank(match):
        return re.sub(r"[^\n]", " ", match.group())

    return re.sub(r'//[^\n]*|/\*[\s\S]*?\*/|"(?:\\.|[^"\\])*"|\'(?:\\.|[^\'\\])*\'', blank, text)


def cpp_header_inventory(text: str) -> tuple[tuple[dict, ...], tuple[dict, ...]]:
    """Lexically inventory this header's inline bodies and named top-level types.

    Brace depth excludes calls/if blocks. Commented rotation helpers are excluded.
    This deliberately does not claim general C++ parsing or preprocessor analysis.
    """
    code = _cpp_code(text)
    depth, level, stack, closing = [], 0, [], {}
    for i, character in enumerate(code):
        depth.append(level)
        if character == "{":
            stack.append(i)
            level += 1
        elif character == "}":
            if not stack:
                raise ValueError("unbalanced source header braces")
            level -= 1
            closing[stack.pop()] = i
    if stack:
        raise ValueError("unbalanced source header braces")
    types = []
    for match in re.finditer(r"(?m)^(struct|class)\s+(\w+)\s*\{", code):
        opening = match.end() - 1
        if depth[opening] == 0:
            types.append(
                {
                    "name": match.group(2),
                    "kind": match.group(1),
                    "line": text.count("\n", 0, match.start()) + 1,
                    "opening": opening,
                    "closing": closing[opening],
                }
            )
    pattern = re.compile(
        r"(?m)^[ \t]*((?:[^\n;{}]*?\s+)?(operator\s*[=+*/-]|~?\w+)\s*\([^;{}]*\))\s*\{"
    )
    counts, definitions = Counter(), []
    for match in pattern.finditer(code):
        opening = match.end() - 1
        owner = next(
            (item["name"] for item in types if item["opening"] < opening < item["closing"]), None
        )
        if depth[opening] != (1 if owner else 0):
            continue
        signature = " ".join(match.group(1).split())
        if owner is None and not signature.startswith("inline "):
            continue
        owner = owner or "LIDARPT"
        name = "".join(match.group(2).split())
        counts[(owner, name)] += 1
        definitions.append(
            {
                "id": f"{owner}::{name}#{counts[(owner, name)]}",
                "owner": owner,
                "name": name,
                "signature": signature,
                "line": text.count("\n", 0, match.start()) + 1,
            }
        )
    return tuple(definitions), tuple(
        {k: v for k, v in item.items() if k not in ("opening", "closing")} for item in types
    )


def cpp_definitions(text: str, owner: str) -> tuple[dict, ...]:
    """Find out-of-class definitions, excluding comment/string lookalikes.

    This intentionally inventories qualified definitions in two selected .cpp
    files, not arbitrary C++ syntax, preprocessing or a complete native call graph.
    """

    stripped = _cpp_code(text)
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
        "engine_ui_definition_count": sum(
            len(file["definitions"])
            for file in audit["definition_files"]
            if file.get("kind") != "inline_header"
        ),
        "header_definition_count": sum(
            len(file["definitions"])
            for file in audit["definition_files"]
            if file.get("kind") == "inline_header"
        ),
        "header_type_count": len(audit.get("header_types", [])),
        "excluded_header_definitions": audit.get("excluded_header_definitions", []),
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
        try:
            actual = (
                cpp_header_inventory(path.read_text())[0]
                if file.get("kind") == "inline_header"
                else cpp_definitions(path.read_text(), file["owner"])
            )
        except ValueError:
            errors.append(f"cannot inventory malformed source: {file['path']}")
            continue
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
    for item in audit.get("header_types", []):
        for target in item["python"]:
            try:
                module, _, attribute = target.rpartition(":")
                obj = import_module(module)
                for part in attribute.split("."):
                    obj = getattr(obj, part)
            except (ImportError, AttributeError):
                errors.append(f"missing Python target: {target}")
    header = root / "LIDARPT.h"
    if header.is_file() and "header_types" in audit:
        try:
            _, actual_types = cpp_header_inventory(header.read_text())
        except ValueError:
            return tuple(errors)
        if [(item["name"], item["kind"]) for item in actual_types] != [
            (item["name"], item["kind"]) for item in audit["header_types"]
        ]:
            errors.append("type inventory differs: LIDARPT.h")
    return tuple(errors)
