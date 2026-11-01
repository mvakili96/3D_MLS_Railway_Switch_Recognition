"""Source vertical grade segmentation, curve formulas and computational GWAD export."""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass, replace
from math import pi
from operator import index

import numpy as np

from .geometry import Line3D, distance_2d, distance_3d
from .models import GradeSection, TrackLayer
from .points import Point3D


@dataclass(frozen=True)
class GradeSplit:
    start: int
    end: int
    vertex: int
    distance: float


@dataclass(frozen=True)
class GradeDivision:
    marked_indices: tuple[int, ...]
    new_indices: tuple[int, ...]
    splits: tuple[GradeSplit, ...]


def _division(before: Sequence[int], points: Sequence[Point3D], splits) -> GradeDivision:
    marked = tuple(i for i, point in enumerate(points) if point.point_type == 1)
    return GradeDivision(marked, tuple(i for i in marked if before[i] != 1), tuple(splits))


def _distances(xy: np.ndarray, line: Line3D) -> np.ndarray:
    with np.errstate(over="ignore", divide="ignore", invalid="ignore"):
        return np.abs(
            (line.a * xy[:, 0] + line.b * xy[:, 1] + line.c)
            / np.sqrt(line.a * line.a + line.b * line.b)
        )


def douglas_peucker_vertical(
    vertices: list[Point3D], start: int, end: int, threshold: float
) -> GradeDivision:
    """DouglasPeuckerVertical: X/Z distances and left-first strict-max splits.

    Iterative spans preserve recursive visitation order without copying the whole
    vector or exhausting Python's recursion depth. Existing tags are not cleared.
    """
    start, end = index(start), index(end)
    before = tuple(point.point_type for point in vertices)
    if end <= start:
        return _division(before, vertices, ())
    if start < 0 or end >= len(vertices):
        raise ValueError("grade simplification span must be within the vertex vector")
    xyz = np.array([point.as_array() for point in vertices])
    xz = xyz[:, (0, 2)]
    pending, splits = [(start, end)], []
    while pending:
        first, last = pending.pop()
        if last <= first:
            continue
        line = Line3D(Point3D(*xyz[first, (0, 2, 1)]), Point3D(*xyz[last, (0, 2, 1)]))
        distances = _distances(xz[first + 1 : last], line)
        # Native max starts at -1e100; NaNs fail its strict > comparison.
        eligible = np.flatnonzero(distances > -1e100)
        if not len(eligible):
            continue
        maximum = float(np.max(distances[eligible]))
        selected = int(eligible[np.flatnonzero(distances[eligible] == maximum)[0]]) + first + 1
        if selected > 0 and maximum > threshold:
            vertices[selected] = replace(vertices[selected], point_type=1)
            splits.append(GradeSplit(first, last, selected, maximum))
            pending.extend(((selected, last), (first, selected)))
    return _division(before, vertices, splits)


def determine_grade_sections(vertices: list[Point3D], threshold: float = 1.0) -> GradeDivision:
    """DetermineGradeSection: mark endpoints, then simplify without clearing tags."""
    before = tuple(point.point_type for point in vertices)
    if not vertices:
        return _division(before, vertices, ())
    vertices[0] = replace(vertices[0], point_type=1)
    vertices[-1] = replace(vertices[-1], point_type=1)
    result = douglas_peucker_vertical(vertices, 0, len(vertices) - 1, threshold)
    return _division(before, vertices, result.splits)


@dataclass(frozen=True)
class GradeParameters:
    sections: tuple[GradeSection, ...]
    maximum_offsets: tuple[float, ...]
    status: str


def compute_grade_parameters(
    vertices: Sequence[Point3D], *, sections: list[GradeSection] | None = None
) -> GradeParameters:
    """ComputeGradeParameters, including shared endpoints and source curve algebra.

    Clear the supplied output list first. All input XYZ/types are retained by
    value; endpoint tagging belongs to DetermineGradeSection, not this function.
    Defined IEEE NaN/Inf values are returned without geometric replacements.
    """
    output = sections if sections is not None else []
    output.clear()
    members = []
    for point in vertices:
        members.append(point)
        if point.point_type == 1:
            if len(members) > 1:
                output.append(GradeSection(member_vertices=list(members)))
            members = [point]
    if len(members) > 1:
        output.append(GradeSection(member_vertices=list(members)))
    maximum_offsets = []
    for section in output:
        # POINT3D::operator= copies XYZ only; initialized endpoint tags stay zero.
        section.begin = Point3D(*section.member_vertices[0].as_array())
        section.end = Point3D(*section.member_vertices[-1].as_array())
        xyz = np.array([point.as_array() for point in section.member_vertices])
        section.chord = distance_2d(section.begin, section.end)
        line = Line3D(section.begin, section.end)
        with np.errstate(over="ignore", divide="ignore", invalid="ignore"):
            delta = xyz[:-1, :2] - xyz[1:, :2]
            lengths = np.sqrt(delta[:, 0] * delta[:, 0] + delta[:, 1] * delta[:, 1])
            section.arc = float(np.cumsum(lengths)[-1])
            distances = _distances(xyz[:-1, :2], line)  # Source excludes the final vertex.
            maximum = -1e100
            eligible = distances[distances > maximum]
            if len(eligible):
                maximum = float(np.max(eligible))
            maximum_offsets.append(maximum)
            section.height = section.end.z - section.begin.z
            section.grade = float(np.divide(section.height, section.arc) * 100)
            section.radius = section.chord * section.chord - 8 * maximum + 4 * maximum * maximum
            section.degree_of_curve = float(np.divide(360 * section.arc, 2 * pi * section.radius))
            delta_grade = 0.04 * section.degree_of_curve
            section.grade_with_curve = (
                section.grade + delta_grade if delta_grade > 0 else section.grade - delta_grade
            )
    finite = all(
        np.isfinite(
            [s.arc, s.chord, s.height, s.grade, s.radius, s.degree_of_curve, s.grade_with_curve]
        ).all()
        for s in output
    )
    status = "no_sections" if not output else "ok" if finite else "nonfinite_grade_parameters"
    return GradeParameters(tuple(output), tuple(maximum_offsets), status)


@dataclass(frozen=True)
class GradeReportRow:
    point: Point3D
    distance_from_start: float
    grade_begin: float | None
    grade_end: float | None
    grade_difference: float


def _report(sections: Sequence[GradeSection]) -> tuple[GradeReportRow, ...]:
    if not sections:
        return ()
    rows = [GradeReportRow(sections[0].begin, 0.0, None, sections[0].grade, sections[0].grade)]
    total = sections[0].arc
    for previous, current in zip(sections, sections[1:]):
        rows.append(
            GradeReportRow(
                current.begin, total, previous.grade, current.grade, current.grade - previous.grade
            )
        )
        total += current.arc
    rows.append(
        GradeReportRow(sections[-1].end, total, sections[-1].grade, None, -sections[-1].grade)
    )
    return tuple(rows)


@dataclass(frozen=True)
class GradeAnalysis:
    layer: TrackLayer
    division: GradeDivision
    parameters: GradeParameters
    report: tuple[GradeReportRow, ...]

    @property
    def sections(self) -> tuple[GradeSection, ...]:
        return self.parameters.sections

    @property
    def status(self) -> str:
        return self.parameters.status


def compute_grade(layer: TrackLayer) -> GradeAnalysis:
    """ComputeGrade, explicitly invoked on a layer; its source threshold is 1."""
    division = determine_grade_sections(layer.center_vector, threshold=1.0)
    parameters = compute_grade_parameters(layer.center_vector)
    return GradeAnalysis(layer, division, parameters, _report(parameters.sections))


def compute_layer_grades(layers: Sequence[TrackLayer]) -> tuple[GradeAnalysis, ...]:
    """The compute-grade menu's ordered loop over ALL layers, including empty ones."""
    return tuple(compute_grade(layer) for layer in layers)


@dataclass(frozen=True)
class GWADRow:
    point: Point3D
    layer_name: str
    chainage: float
    element: str
    parameters: tuple[float | None, ...]


@dataclass(frozen=True)
class GWADExport:
    analyses: tuple[GradeAnalysis, ...]
    rows: tuple[GWADRow, ...]


def build_gwad_rows(layers: Sequence[TrackLayer], layer_names: Sequence[str]) -> GWADExport:
    """GWAD UI computation: run grades, 3D chainage, tags 1/2 and all matches.

    Match with POINT3D::IsEqual, preserving begin-before-end and variable parameter
    counts. Formatting is separate; no file is written by this computational API.
    """
    if len(layers) != len(layer_names) or any(not isinstance(name, str) for name in layer_names):
        raise ValueError("provide one string name per layer before exporting GWAD rows")
    analyses, rows = [], []
    for layer, name in zip(layers, layer_names):
        analysis = compute_grade(layer)
        analyses.append(analysis)
        distance = 0.0
        for i, point in enumerate(layer.center_vector):
            if i != 0:
                distance += distance_3d(layer.center_vector[i - 1], point)
            if point.point_type not in (1, 2):
                continue
            parameters = []
            if point.point_type == 1:
                for section in analysis.sections:
                    if section.begin.is_equal(point):
                        parameters.extend((None, section.grade))
                    elif section.end.is_equal(point):
                        parameters.extend((section.grade, None))
            rows.append(
                GWADRow(
                    point,
                    name,
                    distance,
                    "GR" if point.point_type == 1 else "SW",
                    tuple(parameters),
                )
            )
    return GWADExport(tuple(analyses), tuple(rows))


def format_gwad(result: GWADExport) -> str:
    """Legacy fixed-six-decimal TSV, including variable/trailing parameter fields.

    Nonfinite values retain Python's nan/inf text; native NaN spelling/sign is
    platform-dependent. Finite formatting is checked against the original UI loop.
    """
    lines = ["X\tY\tZ\tChainage\tElement\tParm1\tParm2"]
    for row in result.rows:
        fields = [
            *(f"{value:.6f}" for value in row.point.as_array()),
            f"{row.layer_name}+{row.chainage:.6f}",
            row.element,
            *("" if value is None else f"{value:.6f}" for value in row.parameters),
        ]
        lines.append("\t".join(fields))
    return "\n".join(lines) + "\n"
