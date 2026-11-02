"""Review evidence for the agreed computational scope, distinct from parity."""

from __future__ import annotations

from collections import Counter
from importlib import import_module, resources
import json
from pathlib import Path
import re

from .coverage import SOURCE_DIRECTORY, cpp_function_calls, source_coverage


def reconstruction_review() -> dict:
    """Packaged review decisions, call-site witnesses and explicit exclusions."""
    return json.loads(resources.files("railway_mls").joinpath("reconstruction_audit.json").read_text())


def check_reconstruction_review(repo: str | Path) -> tuple[str, ...]:
    """Check review consistency; this command does not execute tests or prove parity."""
    review, coverage, errors = reconstruction_review(), source_coverage(), []
    repo = Path(repo)
    root = repo / SOURCE_DIRECTORY
    if review["baseline_id"] != coverage["baseline_id"]:
        errors.append("reconstruction review belongs to a different source baseline")
    expected_files = {item["path"] for item in coverage["files"]}
    reviewed_files = [path for item in review["file_reviews"] for path in item["files"]]
    if set(reviewed_files) != expected_files or len(reviewed_files) != len(expected_files):
        errors.append("reconstruction file reviews must cover each project source file once")
    definitions = {
        entry["id"]: (file, entry)
        for file in coverage["definition_files"] for entry in file["definitions"]
    }
    recorded = review["call_inventory"]
    if ({item["definition"] for item in recorded} != set(definitions)
            or len(recorded) != len(definitions)):
        errors.append("reconstruction lexical call inventory differs from definition inventory")
    names = {entry["name"] for _, entry in definitions.values()}
    for item in recorded:
        if item["definition"] not in definitions:
            continue
        file, entry = definitions[item["definition"]]
        path = root / file["path"]
        if not path.is_file():
            continue  # Source coverage reports missing files; witnesses check entry points too.
        try:
            tokens = cpp_function_calls(path.read_text(), entry)
        except ValueError:
            errors.append(f"cannot inventory reviewed function: {entry['id']}")
            continue
        actual = dict(Counter(name for name in tokens if name in names))
        if actual != item["project_named_call_tokens"]:
            errors.append(f"reviewed call tokens changed: {entry['id']}")
    for workflow in review["workflows"]:
        label = workflow["id"]
        if not workflow.get("note") or not workflow.get("tests") or not workflow.get("python"):
            errors.append(f"incomplete workflow review: {label}")
        for definition in workflow["definitions"]:
            if definition not in definitions:
                errors.append(f"unknown workflow definition: {label}: {definition}")
        for target in workflow["python"]:
            try:
                module, _, name = target.rpartition(":")
                obj = import_module(module)
                for part in name.split("."):
                    obj = getattr(obj, part)
            except (ImportError, AttributeError):
                errors.append(f"missing workflow Python target: {target}")
        for path in workflow["tests"]:
            if not (repo / path).is_file():
                errors.append(f"missing review test source: {path}")
    for witness in review["call_sites"]:
        caller = witness["caller"]
        if caller not in definitions:
            errors.append(f"unknown reviewed caller: {caller}")
            continue
        file, entry = definitions[caller]
        path = root / file["path"]
        if not path.is_file():
            errors.append(f"missing reviewed caller source: {file['path']}")
            continue
        try:
            calls = cpp_function_calls(path.read_text(), entry)
        except ValueError as error:
            errors.append(f"cannot check reviewed caller {caller}: {error}")
            continue
        for name in witness.get("must_call", []):
            if name not in calls:
                errors.append(f"missing reviewed call: {caller} -> {name}")
        for name in witness.get("must_not_call", []):
            if name in calls:
                errors.append(f"inactive reviewed call was enabled: {caller} -> {name}")
    register = repo / "docs/LIMITATIONS.md"
    if not register.is_file():
        errors.append("missing limitations register")
    elif set(re.findall(r"\bLIM-\d{3}\b", register.read_text())) != set(review["open_limitations"]):
        errors.append("reconstruction review and limitations register IDs differ")
    return tuple(errors)


def reconstruction_summary() -> dict:
    review = reconstruction_review()
    return {
        "scope": review["scope"],
        "review_status": review["review_status"],
        "workflow_count": len(review["workflows"]),
        "reviewed_project_file_count": sum(len(item["files"]) for item in review["file_reviews"]),
        "call_site_count": len(review["call_sites"]),
        "open_limitations": review["open_limitations"],
        "whole_application_parity_verified": False,
        "real_data_validation_complete": False,
        "limitations_review_complete": False,
    }
