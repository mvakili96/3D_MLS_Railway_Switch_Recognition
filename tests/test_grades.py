from dataclasses import asdict
import json
import subprocess
import sys
import unittest

import numpy as np
from numpy.testing import assert_allclose, assert_array_equal

from railway_mls import (
    GradeSection,
    GridIndex,
    Point3D,
    TrackLayer,
    TrackingEngine,
    build_gwad_rows,
    compute_grade,
    compute_grade_parameters,
    compute_layer_grades,
    determine_grade_sections,
    douglas_peucker_vertical,
    format_gwad,
    tag_rail_vector,
)
from railway_mls.synthetic import grade_scene, tracking_scene
from tests.cpp_reference import CppReference


def vector_state(points):
    return [len(points), *(v for p in points for v in (*p.as_array(), p.point_type))]


def section_state(section):
    return [
        *section.begin.as_array(),
        section.begin.point_type,
        *section.end.as_array(),
        section.end.point_type,
        *vector_state(section.member_vertices),
        section.arc,
        section.chord,
        section.height,
        section.grade,
        section.radius,
        section.degree_of_curve,
        section.grade_with_curve,
    ]


def report_state(row):
    return [
        *row.point.as_array(),
        row.distance_from_start,
        row.grade_begin if row.grade_begin is not None else float("nan"),
        row.grade_end if row.grade_end is not None else float("nan"),
        row.grade_difference,
    ]


def marked(points):
    return tuple(i for i, p in enumerate(points) if p.point_type == 1)


class GradeTests(unittest.TestCase):
    def test_struct_defaults_and_reset_clear_types_members_and_parameters(self):
        section = GradeSection(
            begin=Point3D(1, 2, 3, 7),
            end=Point3D(4, 5, 6, 9),
            member_vertices=[Point3D(8, 9, 10)],
            arc=99,
        )
        members = section.member_vertices
        section.reset()
        self.assertIs(section.member_vertices, members)
        self.assertEqual(asdict(section), asdict(GradeSection()))

    def test_strict_threshold_endpoint_tags_and_new_indices(self):
        points = [Point3D(0, 0, 0, 2), Point3D(1, 0, 1), Point3D(2, 0, 0, 2)]
        result = determine_grade_sections(points, 1)
        self.assertEqual(result.marked_indices, (0, 2))
        self.assertEqual(result.new_indices, (0, 2))
        result = determine_grade_sections(points, np.nextafter(1.0, 0.0))
        self.assertEqual(result.new_indices, (1,))

    def test_equal_maximum_ties_keep_first_and_split_depth_first(self):
        points = [Point3D(i, 0, z) for i, z in enumerate((0, 2, 2, 0))]
        result = determine_grade_sections(points, 0.5)
        self.assertEqual([s.vertex for s in result.splits], [1, 2])
        self.assertEqual([(s.start, s.end) for s in result.splits], [(0, 3), (1, 3)])

    def test_vertical_simplification_uses_x_not_along_track_distance(self):
        east = [Point3D(i, 0, z) for i, z in enumerate((0, 2, 0))]
        north = [Point3D(0, i, z) for i, z in enumerate((0, 2, 0))]
        determine_grade_sections(east)
        determine_grade_sections(north)
        self.assertEqual(marked(east), (0, 1, 2))
        self.assertEqual(marked(north), (0, 2))

    def test_existing_grade_tags_survive_and_switch_tags_can_be_overwritten(self):
        points = [
            Point3D(0, 0, 0, 2),
            Point3D(1, 0, 0, 1),
            Point3D(2, 0, 3, 2),
            Point3D(3, 0, 0, 2),
        ]
        result = determine_grade_sections(points)
        self.assertEqual(result.marked_indices, (0, 1, 2, 3))
        self.assertEqual(result.new_indices, (0, 2, 3))
        determine_grade_sections(points, 100)
        self.assertEqual(marked(points), (0, 1, 2, 3))

    def test_direct_subspan_does_not_mark_endpoints_or_other_points(self):
        points = [Point3D(i, 0, z) for i, z in enumerate((20, 0, 2, 0, 20))]
        result = douglas_peucker_vertical(points, 1, 3, 1)
        self.assertEqual(result.marked_indices, (2,))
        with self.assertRaises(ValueError):
            douglas_peucker_vertical(points, -1, 3, 1)
        self.assertEqual(douglas_peucker_vertical(points, 10, 0, 1).splits, ())

    def test_iterative_spans_handle_deep_first_winner_chain(self):
        points = [Point3D(i, 0, 0) for i in range(1500)]
        result = determine_grade_sections(points, -1)
        self.assertEqual(len(result.marked_indices), 1500)
        self.assertEqual([s.vertex for s in result.splits], list(range(1, 1499)))

    def test_membership_includes_leading_trailing_sections_and_shared_grade_vertex(self):
        points = [
            Point3D(0, 0, 0),
            Point3D(1, 0, 0.1, 2),
            Point3D(2, 0, 0.2, 1),
            Point3D(3, 0, 0.1),
        ]
        old = [GradeSection(arc=99)]
        result = compute_grade_parameters(points, sections=old)
        self.assertEqual(len(old), 2)
        self.assertEqual(result.sections[0].member_vertices, points[:3])
        self.assertEqual(result.sections[1].member_vertices, points[2:])
        self.assertTrue(
            all(s.begin.point_type == 0 and s.end.point_type == 0 for s in result.sections)
        )
        self.assertEqual([p.point_type for p in points], [0, 2, 1, 0])

    def test_straight_section_retains_source_squared_chord_radius_and_compensation(self):
        section = compute_grade_parameters([Point3D(0, 0, 0), Point3D(3, 4, 1)]).sections[0]
        self.assertEqual(
            (section.arc, section.chord, section.height, section.grade, section.radius),
            (5, 5, 1, 20, 25),
        )
        self.assertAlmostEqual(section.degree_of_curve, 360 * 5 / (2 * np.pi * 25))
        self.assertGreater(section.grade_with_curve, section.grade)

    def test_negative_radius_and_down_grade_keep_source_compensation_branch(self):
        section = compute_grade_parameters(
            [Point3D(0, 0, 0), Point3D(0.5, 1, 0), Point3D(1, 0, 0)]
        ).sections[0]
        self.assertEqual(section.radius, -3)
        self.assertLess(section.degree_of_curve, 0)
        self.assertGreater(section.grade_with_curve, 0)
        down = compute_grade_parameters([Point3D(0, 0, 1), Point3D(3, 4, 0)]).sections[0]
        self.assertLess(down.grade, 0)
        self.assertGreater(down.grade_with_curve, down.grade)

    def test_zero_radius_and_zero_xy_arc_retain_ieee_values(self):
        zero_radius = compute_grade_parameters(
            [Point3D(0, 0, 0), Point3D(1, 1, 0), Point3D(2, 0, 0)]
        )
        self.assertEqual(zero_radius.status, "nonfinite_grade_parameters")
        self.assertEqual(zero_radius.sections[0].radius, 0)
        self.assertTrue(np.isinf(zero_radius.sections[0].degree_of_curve))
        zero_arc = compute_grade_parameters([Point3D(0, 0, 0), Point3D(0, 0, 1)])
        self.assertTrue(np.isinf(zero_arc.sections[0].grade))
        self.assertTrue(np.isnan(zero_arc.sections[0].grade_with_curve))

    def test_empty_singleton_and_report_distances_have_defined_results(self):
        self.assertEqual(compute_grade(TrackLayer()).status, "no_sections")
        one = TrackLayer(center_vector=[Point3D(1, 2, 3, 2)])
        result = compute_grade(one)
        self.assertEqual(result.sections, ())
        self.assertEqual(one.center_vector[0].point_type, 1)
        layer = TrackLayer(center_vector=[Point3D(0, 0, 0), Point3D(1, 0, 2), Point3D(2, 0, 0)])
        result = compute_grade(layer)
        self.assertEqual([row.distance_from_start for row in result.report], [0, 1, 2])
        self.assertIsNone(result.report[0].grade_begin)
        self.assertIsNone(result.report[-1].grade_end)
        self.assertEqual(result.report[1].grade_difference, -400)


class GWADTests(unittest.TestCase):
    def test_export_uses_3d_chainage_and_keeps_variable_parameter_columns(self):
        layer = grade_scene()
        result = build_gwad_rows([layer], ["Main line"])
        self.assertEqual([r.element for r in result.rows], ["GR", "SW", "GR", "GR", "SW", "GR"])
        self.assertGreater(
            result.rows[-1].chainage, result.analyses[0].report[-1].distance_from_start
        )
        lines = format_gwad(result).splitlines()
        self.assertEqual(len(lines[0].split("\t")), 7)
        self.assertEqual(len(lines[2].split("\t")), 5)
        self.assertEqual(len(lines[3].split("\t")), 9)
        self.assertTrue(lines[-1].endswith("\t"))

    def test_closed_path_matches_multiple_sections_without_collapsing_parameters(self):
        points = [Point3D(0, 0, 0, 1), Point3D(1, 0, 0, 1), Point3D(0, 0, 0, 1)]
        result = build_gwad_rows([TrackLayer(center_vector=points)], ["Loop"])
        self.assertEqual(len(result.rows[0].parameters), 4)
        self.assertEqual(result.rows[0].parameters, result.rows[-1].parameters)

    def test_begin_match_wins_when_both_section_endpoints_coincide(self):
        result = build_gwad_rows(
            [TrackLayer(center_vector=[Point3D(0, 0, 0), Point3D(0, 0, 0)])], ["Zero"]
        )
        self.assertIsNone(result.rows[-1].parameters[0])
        self.assertTrue(np.isnan(result.rows[-1].parameters[1]))

    def test_names_validate_before_mutating_and_empty_layers_produce_header_only(self):
        layer = TrackLayer(center_vector=[Point3D(0, 0, 0, 2)])
        with self.assertRaises(ValueError):
            build_gwad_rows([layer], [])
        self.assertEqual(layer.center_vector[0].point_type, 2)
        result = build_gwad_rows([TrackLayer()], ["Empty"])
        self.assertEqual(result.rows, ())
        self.assertEqual(format_gwad(result), "X\tY\tZ\tChainage\tElement\tParm1\tParm2\n")


class GradeCppTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.reference = CppReference()

    @classmethod
    def tearDownClass(cls):
        cls.reference.close()

    def test_struct_reset_matches_original(self):
        section = GradeSection(
            begin=Point3D(1, 2, 3, 7),
            end=Point3D(4, 5, 6, 9),
            member_vertices=[Point3D(8, 9, 10)],
            arc=99,
        )
        section.reset()
        assert_array_equal([1, *section_state(section)], self.reference.run("grade_reset"))

    def compare_division(self, vertices, threshold=1, span=None):
        values = [*vector_state(vertices), threshold]
        if span is not None:
            values.extend(span)
        native = self.reference.run("grade_mark" if span is None else "grade_dp", *values)
        result = (
            determine_grade_sections(vertices, threshold)
            if span is None
            else douglas_peucker_vertical(vertices, *span, threshold)
        )
        expected = [
            *vector_state(vertices),
            len(result.splits),
            *(
                v
                for split in result.splits
                for v in (split.start, split.end, split.vertex, split.distance)
            ),
        ]
        assert_allclose(expected, native, rtol=1e-12, atol=2e-10, equal_nan=True)
        return result

    def test_threshold_ties_first_winner_and_endpoint_markers_match_original(self):
        for threshold in (1, np.nextafter(1.0, 0.0), -1, float("nan")):
            self.compare_division(
                [Point3D(0, 0, 0, 2), Point3D(1, 0, 1), Point3D(2, 0, 0, 2)], threshold
            )
        self.compare_division([Point3D(i, 0, z) for i, z in enumerate((0, 2, 2, 0))], 0.5)
        self.compare_division([])
        self.compare_division([Point3D(1, 2, 3, 2)])

    def test_subspans_and_existing_tags_match_original(self):
        self.compare_division(
            [Point3D(i, 0, z) for i, z in enumerate((20, 0, 2, 0, 20))], 1, (1, 3)
        )
        self.compare_division(
            [Point3D(0, 0, 0, 2), Point3D(1, 0, 0, 1), Point3D(2, 0, 3, 2), Point3D(3, 0, 0)], 100
        )

    def compare_parameters(self, vertices, full=False):
        native = self.reference.run(
            "grade" if full else "grade_parameters", *vector_state(vertices)
        )
        if full:
            analysis = compute_grade(TrackLayer(center_vector=vertices))
            result, report = analysis.parameters, analysis.report
        else:
            result, report = compute_grade_parameters(vertices, sections=[GradeSection(arc=99)]), ()
        expected = [
            *vector_state(vertices),
            len(result.sections),
            *(v for s in result.sections for v in section_state(s)),
            len(result.maximum_offsets),
            *result.maximum_offsets,
            len(report),
            *(v for row in report for v in report_state(row)),
        ]
        assert_allclose(expected, native, rtol=1e-12, atol=2e-9, equal_nan=True)
        return result

    def test_straight_curved_negative_and_zero_radius_parameters_match_original(self):
        for xyz in (
            ((0, 0, 0), (3, 4, 1)),
            ((0, 0, 1), (3, 4, 0)),
            ((0, 0, 0), (0.5, 1, 0), (1, 0, 0)),
            ((0, 0, 0), (1, 1, 0), (2, 0, 0)),
        ):
            self.compare_parameters([Point3D(*p) for p in xyz])

    def test_degenerate_ieee_and_nonfinite_values_match_original(self):
        self.compare_parameters([Point3D(0, 0, 0), Point3D(0, 0, 1)])
        self.compare_parameters([Point3D(0, 0, 0), Point3D(0, 0, 0)])
        self.compare_parameters([Point3D(0, 0, float("nan")), Point3D(1, 1, 1)], True)

    def test_membership_output_clear_and_empty_singleton_inputs_match_original(self):
        self.compare_parameters([])
        self.compare_parameters([Point3D(1, 2, 3, 2)])
        self.compare_parameters(
            [Point3D(0, 0, 0), Point3D(1, 0, 0.1, 2), Point3D(2, 0, 0.2, 1), Point3D(3, 0, 0.1)]
        )

    def test_full_grade_report_x_only_projection_and_large_origin_match_original(self):
        self.compare_parameters(grade_scene().center_vector, True)
        self.compare_parameters([Point3D(0, i, z) for i, z in enumerate((0, 20, 0))], True)
        self.compare_parameters([Point3D(1e6 + i, 2e6, z) for i, z in enumerate((0, 2, 0))], True)

    def test_ui_all_layer_action_includes_empty_layers_and_preserves_order(self):
        layers = [grade_scene(), TrackLayer(), TrackLayer(center_vector=[Point3D(1, 2, 3, 2)])]
        native = self.reference.run(
            "grade_layers",
            len(layers),
            *(v for layer in layers for v in vector_state(layer.center_vector)),
        )
        result = compute_layer_grades(layers)
        self.assertEqual(len(result), 3)
        assert_array_equal(
            [len(layers), *(v for layer in layers for v in vector_state(layer.center_vector))],
            native,
        )

    def compare_gwad(self, layers, names):
        values = [len(layers)]
        for layer, name in zip(layers, names):
            values.extend([json.dumps(name), *vector_state(layer.center_vector)])
        native = self.reference.run_text("gwad", *values)
        # The C++ protocol separator adds one newline after the actual TSV.
        actual = format_gwad(build_gwad_rows(layers, names))
        self.assertEqual(actual, native.removesuffix("\n"))

    def test_gwad_finite_ui_text_fields_chainage_precision_and_names_match_original(self):
        self.compare_gwad(
            [grade_scene(), TrackLayer(), TrackLayer(center_vector=[Point3D(1, 2, 3, 2)])],
            ["Main line", "Empty", "One"],
        )

    def test_gwad_repeated_coordinates_and_variable_fields_match_original(self):
        layer = TrackLayer(
            center_vector=[Point3D(0, 0, 0, 1), Point3D(1, 0, 0, 1), Point3D(0, 0, 0, 1)]
        )
        self.compare_gwad([layer], ["Loop +1"])

    def test_marked_vector_insertion_capacity_gap_is_explicit(self):
        # Inspect actual capacity: no compiler's vector growth policy is assumed.
        for count in (2, 3):
            points = [Point3D(i * 2, i * 2, i, 10 + i) for i in range(count)]
            state = self.reference.run("tag_vector_state", *vector_state(points), 1, 1.5, 9, 2)
            capacity, native = int(state[0]), state[1:]
            tagged = tag_rail_vector(points, Point3D(1, 1.5, 9))
            if count == 2:
                assert_allclose(vector_state(points), native)
                self.assertTrue(tagged.reference_comparable)
            else:
                self.assertFalse(tagged.reference_comparable)
                self.assertEqual(native[1 + 4 * 2 + 3], 12 if capacity > count else 11)
                self.assertEqual(points[2].point_type, 11)


class GradeIntegrationTests(unittest.TestCase):
    def test_actual_tracking_grade_action_changes_only_center_types_without_rng_draws(self):
        scene = tracking_scene(length=5, curvature=0.01, grade=0.005)
        engine = TrackingEngine(GridIndex(scene.cloud))
        engine.initialize(scene.seed)
        result = engine.run_pass()
        self.assertEqual(result.status, "completed")
        layer = engine.layers[0]
        tag_rail_vector(layer.center_vector, layer.center_vector[2], 2)
        coordinates = np.array([p.as_array() for p in layer.center_vector])
        labels = scene.cloud.class_codes.copy()
        draws = engine.rng.draw_count
        segments = list(layer.track_segments)
        layer_index = engine.current_layer_index
        analyses = engine.compute_grades()
        self.assertEqual(len(analyses[0].sections), 1)
        self.assertEqual(layer.center_vector[2].point_type, 2)
        assert_array_equal(coordinates, [p.as_array() for p in layer.center_vector])
        assert_array_equal(scene.cloud.class_codes, labels)
        self.assertEqual(draws, engine.rng.draw_count)
        self.assertEqual(engine.current_layer_index, layer_index)
        self.assertTrue(all(a is b for a, b in zip(segments, layer.track_segments)))
        export = build_gwad_rows(engine.layers, ["Tracked main"])
        self.assertEqual([r.element for r in export.rows], ["GR", "SW", "GR"])

    def test_grade_cli_reports_three_sections_and_source_gwad_structure(self):
        completed = subprocess.run(
            [sys.executable, "-m", "railway_mls", "grade-demo"],
            text=True,
            capture_output=True,
            check=True,
        )
        result = json.loads(completed.stdout)
        self.assertEqual(result["status"], "ok")
        self.assertEqual(result["grade_vertex_indices"], [0, 6, 10, 16])
        self.assertEqual(result["section_count"], 3)
        self.assertEqual(len(result["gwad_rows"]), 6)
        self.assertEqual(len(result["gwad_rows"][2]["parameters"]), 4)


if __name__ == "__main__":
    unittest.main()
