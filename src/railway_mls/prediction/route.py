"""Freeze one explicit lineage from an automatic branch run for route review."""

import json
import re
from dataclasses import asdict, dataclass
from math import ceil, floor, isfinite
from pathlib import Path

import numpy as np

from ..evaluation.io import file_fingerprint, read_python_vectors
from ..evaluation.matched import _json, _pinned
from ..geometry import distance_2d
from ..points import Point3D
from .benchmark import _contract, _ledger, _mapping_template, _new_output, _save
from .catalog import write_json_new
from .runner import _layer_from_rail_centers, _write_geometry, _write_report


def _read_vertices(path, layer):
    rails = read_python_vectors(path, layer=layer)
    if not all(len(rail) for rail in rails) or len(rails[0]) != len(rails[1]):
        raise ValueError(f"{path}: expected two rails with the same vertex count")
    return np.stack([
        np.concatenate((rail.endpoints[:1, 0], rail.endpoints[:, 1]))
        for rail in rails
    ], axis=1)


@dataclass(frozen=True)
class _Span:
    route: str
    first: int
    last: int
    reverse: bool = False


class _TrackingGeometry:
    def __init__(self, directory, report):
        self.directory = directory
        self.source = report["source"]
        self.sources = {row["route"]: row for row in self.source["journal_routes"]}
        self.branches = {row["route"]: row for row in report["branches"]}
        if (len(self.sources) != len(self.source["journal_routes"])
                or len(self.branches) != len(report["branches"])
                or any(not re.fullmatch(r"source-[0-9]+", name) for name in self.sources)
                or any(not re.fullmatch(r"branch-[0-9]+", name) for name in self.branches)):
            raise ValueError("automatic run has invalid or duplicate route IDs")
        self.inputs = {}
        self._vertices = {}

    def _record(self, directory, row, filename):
        path = directory / filename
        artifacts = row.get("artifacts")
        if artifacts is not None:
            pin = artifacts[filename]
            fingerprint = _pinned(path, pin["sha256"])
            if fingerprint["bytes"] != pin["bytes"]:
                raise ValueError(f"{path}: size differs from the tracking report")
        else:
            fingerprint = file_fingerprint(path)
        self.inputs[str(path)] = fingerprint
        return path

    def _legacy_source(self, directory, row):
        """Recover the seed and accepted vertices only after an exact journal match."""
        journal = self._record(directory, self.source, "steps.jsonl")
        events = []
        with journal.open(encoding="utf-8") as stream:
            for line in stream:
                event = json.loads(line)
                if (event["invocation"] == row["invocation"] and event["layer"] == row["layer"]
                        and event["accepted"] and event["source_status"] == "accepted"):
                    events.append(event)
        if not events or [e["number"] for e in events] != list(range(1, row["accepted_steps"] + 1)):
            raise ValueError("legacy source journal and accepted-window counts disagree")
        vectors = _read_vertices(self._record(directory, self.source, "vectors.csv"), row["layer"])
        centers = (vectors[:, 0] + vectors[:, 1]) * 0.5
        ordered = np.array([event["center"] for event in events])
        matches = [
            start for start in np.flatnonzero(np.all(centers == ordered[0], axis=1))
            if start > 0 and np.array_equal(centers[start:start + len(events)], ordered)
        ]
        if len(matches) != 1:
            raise ValueError("legacy source traversal cannot be recovered unambiguously from its vectors")
        start = matches[0]
        return vectors[start - 1:start + len(events)]

    def count(self, route):
        if route in self.sources:
            return self.sources[route]["accepted_steps"]
        row = self.branches.get(route)
        if row is None or row["confirmed_at"] is None:
            raise ValueError(f"{route}: expected a confirmed route from the automatic run")
        return row["exported_steps"]

    def vertices(self, route):
        if route not in self._vertices:
            count = self.count(route)
            if route in self.sources:
                row = self.sources[route]
                directory = self.directory / "source"
                if "accepted_layer" in row:
                    path = self._record(directory, self.source, "accepted/vectors.csv")
                    vertices = _read_vertices(path, row["accepted_layer"])
                else:
                    vertices = self._legacy_source(directory, row)
            else:
                row = self.branches[route]
                path = self._record(self.directory / route, row, "vectors.csv")
                vertices = _read_vertices(path, 0)
            if vertices.shape != (count + 1, 2, 3):
                raise ValueError(f"{route}: accepted-window count differs from the exported geometry")
            self._vertices[route] = vertices
        return self._vertices[route]

    def lineage(self, branch, follow_rejoins):
        if branch not in self.branches:
            raise ValueError("select a confirmed branch ID from the automatic run")
        spans, seen = [], set()
        route, end = branch, self.count(branch)
        while route in self.branches:
            if route in seen:
                raise ValueError("branch ancestry contains a cycle")
            seen.add(route)
            spans.append(_Span(route, 0, end))
            row = self.branches[route]
            route, end = row["parent"], row["parent_window"] - 1
        if route not in self.sources:
            raise ValueError("branch ancestry does not lead to a source traversal")
        spans.append(_Span(route, 0, end))
        spans.reverse()
        route = branch
        joined = {branch}
        while follow_rejoins and route in self.branches:
            join = self.branches[route].get("rejoin")
            if join is None:
                break
            direction, station = join["direction"], join["station"]
            route = join["route"]
            count = self.count(route)
            if direction not in (-1, 1) or not isfinite(station) or not 0 <= station <= count:
                raise ValueError("invalid reported rejoin station")
            # Whole recorded segments on the known route start after the join.
            if direction == -1:
                spans.append(_Span(route, 0, floor(station), reverse=True))
                break
            if route in joined:
                raise ValueError("selected rejoins would repeat a branch")
            joined.add(route)
            spans.append(_Span(route, ceil(station), count))
        return spans

    def layers(self, spans):
        layers, fragments, previous = [], [], None
        used = {}
        for span in spans:
            first, last = span.first, span.last
            if not 0 <= first <= last <= self.count(span.route):
                raise ValueError("selected range lies outside its accepted traversal")
            if any(max(a, first) < min(b, last) for a, b in used.setdefault(span.route, [])):
                raise ValueError("selected route would repeat a recorded segment")
            used[span.route].append((first, last))
            if first == last:
                continue
            rows = self.vertices(span.route)[first:last + 1]
            if span.reverse:
                rows = rows[::-1, ::-1]
            layer = _layer_from_rail_centers([
                tuple(Point3D(*point) for point in rails) for rails in rows
            ])
            fragments.append({
                "layer": len(layers), "route": span.route, "first_window": first,
                "last_window": last, "reverse": span.reverse,
                "gap_before_m": None if previous is None else
                distance_2d(previous, layer.center_vector[0]),
            })
            layers.append(layer)
            previous = layer.center_vector[-1]
        if not layers:
            raise ValueError("selected route contains no paired rail segments")
        return layers, fragments


def export_branch_route(manifest, tracking, output, *, branch, follow_rejoins=False):
    """Export measured fragments and an unreviewed mapping template without scoring."""
    contract = _contract(manifest)
    tracking = Path(tracking).resolve()
    raw, raw_pin = _json(tracking / "report.json")
    if (raw.get("kind") != "automatic_branch_tracking" or raw.get("schema_version") != 1
            or raw.get("status") in (None, "running") or raw.get("source") is None
            or raw.get("reference_files_read") is not False
            or raw.get("inference_manifest_sha256") != contract["manifest"]["sha256"]):
        raise ValueError("expected a finished automatic run from the supplied experiment")
    spec = contract["experiment"].inference
    run = next((run for run in spec.runs if run.run_id == raw["run_id"]), None)
    if (run is None or run.status != "ready" or raw["source_baseline"] != spec.source_baseline
            or raw["seed"] != {**asdict(run.seed), "xyz": list(run.seed.xyz)}
            or raw["direction"] != run.direction or raw["random_seed"] != run.random_seed
            or raw["config"] != asdict(spec.config)):
        raise ValueError("automatic run provenance conflicts with the experiment")
    geometry = _TrackingGeometry(tracking, raw)
    spans = geometry.lineage(branch, follow_rejoins)
    layers, fragments = geometry.layers(spans)
    output = _new_output(output, contract, [tracking])
    predictions = output / "predictions"
    directory = predictions / run.run_id
    directory.mkdir(parents=True)
    geometry_metadata = _write_geometry(directory, layers)
    terminal_route = spans[-1].route
    stop = (
        geometry.branches[terminal_route]["status"] if terminal_route in geometry.branches
        else raw["source"]["status"]
    )
    metadata = {
        "schema_version": 1, "run_id": run.run_id, "status": stop,
        "branch_policy": "automatic", "seed": raw["seed"], "seed_provenance": raw["seed_provenance"],
        "direction": run.direction, "config": raw["config"], "source_baseline": spec.source_baseline,
        "cache_manifest_sha256": raw["cache_manifest_sha256"],
        "random": {"generator": "MSVC", "seed": run.random_seed},
        "automatic_run": raw_pin, "automatic_run_status": raw["status"],
        "selected_branch": branch, "follow_rejoins": follow_rejoins,
        "route_fragments": fragments, "route_inputs": list(geometry.inputs.values()),
        "layers": geometry_metadata, "limitations_encountered": ["LIM-020", "LIM-021"],
        "reference_files_read": False, "evaluation_performed": False, "reference_comparable": False,
        "artifacts": {name: file_fingerprint(directory / name)
                      for name in ("vectors.csv", "segments.csv")},
    }
    _write_report(directory / "manifest.json", metadata)
    root = {
        "schema_version": 1, "kind": "automatic_branch_route_export", "status": "completed",
        "manifest_sha256": contract["manifest"]["sha256"], "reference_files_read": False,
        "evaluation_performed": False, "runs": [metadata],
    }
    _write_report(predictions / "manifest.json", root)
    prediction = file_fingerprint(predictions / "manifest.json")
    run_pin = file_fingerprint(directory / "manifest.json")
    sources = [{
        "source_id": "route", "directory": str(predictions),
        "manifest_sha256": prediction["sha256"],
        "inference_manifest": contract["manifest"]["path"],
    }]
    path_id = contract["path_by_run"][run.run_id]
    write_json_new(output / "mapping-template.json", _mapping_template(contract, sources))
    write_json_new(output / "route-candidates.json", {
        "path_id": path_id, "branch": branch, "status": "requires_explicit_review",
        "fragments": [
            {"source_id": "route", "run_id": run.run_id, "layer": fragment["layer"],
             "run_manifest_sha256": run_pin["sha256"]}
            for fragment in fragments
        ],
        "route_fragments": fragments,
    })
    report = _ledger(contract, [metadata])
    report.update(stage="prediction", prediction_exports=[prediction],
                  reference_point_records_read=False, selected_branch=branch,
                  route_fragments=fragments, automatic_run=raw_pin)
    _save(output, report)
    return report
