from dataclasses import asdict, replace
import json
import subprocess
import sys
import unittest

import numpy as np
from numpy.testing import assert_allclose

from railway_mls.config import ExtractionConfig
from railway_mls.models import BinaryString, TrackStack
from railway_mls.recognition import (
    define_templates,
    recognize_stacks,
    recognize_turnout,
    stack_signature,
    template_matching,
)
from railway_mls.synthetic import signature_stack
from railway_mls.tracking import SequentialTracker
from tests.cpp_reference import CppReference
from tests.test_tracking import prepared


def binary_state(value):
    return [
        value.type,
        value.score,
        value.position_label,
        value.direction_label,
        value.turnout_label,
        value.toe_label,
        len(value.bits),
        *value.bits,
    ]


class RecognitionTests(unittest.TestCase):
    def test_catalogue_has_twenty_original_entries_and_independent_subsets(self):
        catalogue = define_templates()
        self.assertEqual([template.type for template in catalogue.all], list(range(20)))
        self.assertEqual(
            [template.type for template in catalogue.begin], [0, 1, 2, 3, 4, 5, 12, 13, 16, 17]
        )
        self.assertEqual(
            [template.type for template in catalogue.end], [6, 7, 8, 9, 10, 11, 14, 15, 18, 19]
        )
        self.assertEqual(catalogue.all[0].bits, tuple(map(int, "0100010001010101")))
        for template in catalogue.all:
            self.assertEqual(
                (
                    template.position_label,
                    template.direction_label,
                    template.turnout_label,
                    template.toe_label,
                ),
                (-1, -1, -1, -1),
            )
        catalogue.all[0].type = 77
        self.assertEqual(catalogue.begin[0].type, 0)
        self.assertEqual(define_templates().all[0].type, 0)

    def test_duplicates_are_ambiguous_even_with_perfect_similarity(self):
        catalogue = define_templates()
        for type_id, tied in ((2, (2, 8)), (3, (3, 9)), (8, (2, 8)), (9, (3, 9))):
            observed = BinaryString(bits=catalogue.all[type_id].bits)
            result = recognize_turnout(observed)
            self.assertEqual(result.status, "ambiguous")
            self.assertEqual(observed.type, 999)
            self.assertEqual(observed.score, 1)
            self.assertEqual(result.best_indices, tied)
            self.assertEqual(
                (
                    observed.position_label,
                    observed.direction_label,
                    observed.turnout_label,
                    observed.toe_label,
                ),
                (1, 1, 1, -1),
            )

    def test_all_other_exact_patterns_are_unique(self):
        for template in define_templates().all:
            if template.type in (2, 3, 8, 9):
                continue
            result = recognize_turnout(BinaryString(bits=template.bits))
            self.assertEqual(result.status, "matched")
            self.assertEqual(result.binary.type, template.type)
            self.assertEqual(result.binary.score, 1)

    def test_context_is_computed_but_does_not_break_unary_ties(self):
        observed = BinaryString(bits=(0,) * 16)
        templates = (
            BinaryString(
                type=30, bits=observed.bits, position_label=0, direction_label=0, turnout_label=0
            ),
            BinaryString(
                type=31, bits=observed.bits, position_label=1, direction_label=1, turnout_label=1
            ),
        )
        previous = BinaryString(position_label=0, direction_label=0, turnout_label=0)
        result = recognize_turnout(observed, previous, templates=templates)
        self.assertEqual(result.context_scores, (2, 1))
        self.assertEqual(result.scores, (1, 1))
        self.assertEqual(result.binary.type, 999)

    def test_labels_do_not_assign_type_and_bits_do_not_reset_metadata(self):
        value = BinaryString(type=42, score=0.25)
        value.set_labels(12)
        self.assertEqual(value.type, 42)
        self.assertEqual(
            (value.position_label, value.direction_label, value.turnout_label, value.toe_label),
            (0, 0, 1, 2),
        )
        before = binary_state(value)[:6]
        value.set_bits([2] * 16)
        self.assertEqual(binary_state(value)[:6], before)
        self.assertEqual(value.bits, (2,) * 16)
        with self.assertRaises(ValueError):
            value.set_bits([0] * 15)
        self.assertEqual(value.bits, (2,) * 16)

    def test_no_equal_bits_is_zero_score_ambiguity_without_rejection_threshold(self):
        result = recognize_turnout(BinaryString(bits=(2,) * 16))
        self.assertEqual(result.binary.type, 999)
        self.assertEqual(result.binary.score, 0)
        self.assertEqual(len(result.best_indices), 20)

    def test_empty_catalogue_preserves_score_and_active_labels_use_unknown_else(self):
        value = BinaryString(type=7, score=0.75, position_label=0, direction_label=0)
        result = recognize_turnout(value, templates=())
        self.assertEqual(result.status, "no_templates")
        self.assertEqual(value.type, -1)
        self.assertEqual(value.score, 0.75)
        self.assertEqual(
            (value.position_label, value.direction_label, value.turnout_label, value.toe_label),
            (1, 1, 1, -1),
        )

    def test_raw_helper_uses_counts_first_tied_id_and_leaves_labels_untouched(self):
        value = BinaryString(
            bits=define_templates().all[8].bits, position_label=77, direction_label=88
        )
        result = template_matching(value)
        self.assertEqual(value.type, 2)
        self.assertEqual(value.score, 16)
        self.assertEqual((value.position_label, value.direction_label), (77, 88))
        self.assertEqual(result.best_indices, (2, 8))

    def test_raw_negative_template_ids_keep_source_less_than_zero_guard(self):
        bits = (0,) * 16
        value = BinaryString(bits=bits)
        templates = (
            BinaryString(type=-3, bits=bits),
            BinaryString(type=-5, bits=bits),
            BinaryString(type=7, bits=bits),
        )
        self.assertEqual(template_matching(value, templates=templates).binary.type, 7)
        self.assertEqual(template_matching(value, templates=templates[:2]).binary.type, -5)

    def test_native_vector_lengths_keep_denominator_sixteen_and_ignore_extra_observed_bits(self):
        observed = BinaryString(bits=(0, 1, 9))
        result = recognize_turnout(observed, templates=(BinaryString(type=3, bits=(0, 1)),))
        self.assertEqual(result.binary.score, 2 / 16)
        with self.assertRaises(ValueError):
            recognize_turnout(observed)

    def test_signature_row_and_column_order_ignores_middle_segments(self):
        bits = tuple(range(16))
        stack = signature_stack(bits, 7)
        for segment in stack.track_segments[2:-2]:
            segment.track1.left_tag = segment.track2.right_tag = 999
        self.assertEqual(stack_signature(stack), bits)
        with self.assertRaises(ValueError):
            stack_signature(TrackStack(stack.track_segments[:4]))

    def test_short_stack_keeps_existing_binary_state(self):
        value = BinaryString(type=6, score=0.875, bits=(1,) * 16, position_label=1, toe_label=0)
        stack = TrackStack(signature_stack((0,) * 16).track_segments[:4], value)
        before = asdict(value)
        result = recognize_stacks([stack])
        self.assertEqual(result.entries[0].status, "too_short")
        self.assertEqual(asdict(stack.binary), before)

    def test_previous_is_immediate_skipped_stack_not_last_recognized_stack(self):
        template = define_templates().all[0]
        stacks = [signature_stack(template.bits), TrackStack(), signature_stack(template.bits)]
        result = recognize_stacks(stacks)
        self.assertEqual(result.entries[0].match.previous.position_label, 1)
        self.assertEqual(result.entries[2].match.previous.position_label, -1)
        self.assertEqual(result.entries[0].match.context_scores, (1,) * 20)
        self.assertEqual(result.entries[2].match.context_scores, (0,) * 20)
        self.assertEqual(stacks[2].binary.type, 0)

    def test_batch_main_track_gate_and_empty_batch(self):
        stack = signature_stack(define_templates().all[0].bits)
        batch = recognize_stacks([stack], main_track=False)
        self.assertEqual(batch.entries[0].status, "main_track_disabled")
        self.assertEqual(stack.binary, BinaryString())
        self.assertEqual(recognize_stacks([]).entries, ())

    def test_result_binary_is_a_value_snapshot(self):
        value = BinaryString(bits=define_templates().all[0].bits)
        first = recognize_turnout(value)
        value.set_bits(define_templates().all[1].bits)
        recognize_turnout(value)
        self.assertEqual(first.binary.type, 0)
        self.assertEqual(value.type, 1)


class RecognitionCppTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.reference = CppReference()

    @classmethod
    def tearDownClass(cls):
        cls.reference.close()

    def compare_match(self, observed, previous=None, templates=None, raw=False):
        prior = previous if previous is not None else BinaryString(position_label=1)
        inputs = [*binary_state(observed), *binary_state(prior), int(templates is not None)]
        if templates is not None:
            inputs.append(len(templates))
            for template in templates:
                inputs.extend(binary_state(template))
        native = self.reference.run("match_raw" if raw else "recognize", *inputs)
        result = (
            template_matching(observed, templates=templates)
            if raw
            else recognize_turnout(observed, prior, templates=templates)
        )
        expected = binary_state(observed)
        for scores in (result.unary_scores, result.context_scores, result.scores):
            expected.extend([len(scores), *scores])
        assert_allclose(expected, native, rtol=0, atol=0, equal_nan=True)
        return result

    def test_all_catalogue_values_and_subset_orders_match_original(self):
        catalogue = define_templates()
        expected = []
        for group in (catalogue.all, catalogue.begin, catalogue.end):
            expected.append(len(group))
            for template in group:
                expected.extend(binary_state(template))
        assert_allclose(expected, self.reference.run("templates"), rtol=0, atol=0)

    def test_all_source_label_mappings_and_unknown_else_match_original(self):
        types = [*range(20), -1, 999, 42]
        expected = []
        for type_id in types:
            value = BinaryString()
            value.set_labels(type_id)
            expected.extend(binary_state(value))
        assert_allclose(
            expected, self.reference.run("binary_labels", len(types), *types), rtol=0, atol=0
        )

    def test_every_exact_template_and_single_bit_flip_matches_original(self):
        for template in define_templates().all:
            self.compare_match(BinaryString(bits=template.bits))
            for changed in range(16):
                bits = list(template.bits)
                bits[changed] = 1 - bits[changed]
                self.compare_match(BinaryString(bits=bits))

    def test_random_noisy_and_nonbinary_signatures_match_original(self):
        rng = np.random.default_rng(314)
        for _ in range(64):
            self.compare_match(BinaryString(bits=tuple(rng.integers(0, 2, size=16))))
        for bits in ((0,) * 16, (1,) * 16, (2,) * 16, tuple(range(16))):
            self.compare_match(BinaryString(bits=bits))

    def test_context_diagnostics_with_default_and_custom_template_labels_match_original(self):
        templates = define_templates().all
        for previous in (
            BinaryString(),
            BinaryString(position_label=1),
            BinaryString(position_label=0, direction_label=0, turnout_label=1),
        ):
            self.compare_match(BinaryString(bits=templates[0].bits), previous)
            custom = tuple(replace(template) for template in templates)
            for template in custom:
                template.set_labels(template.type)
            self.compare_match(BinaryString(bits=templates[0].bits), previous, templates=custom)

    def test_raw_matching_and_source_tie_guard_match_original(self):
        for template in define_templates().all:
            self.compare_match(
                BinaryString(bits=template.bits, position_label=77, direction_label=88), raw=True
            )
        bits = (0,) * 16
        templates = (
            BinaryString(type=-3, bits=bits),
            BinaryString(type=-7, bits=bits),
            BinaryString(type=9, bits=bits),
        )
        self.compare_match(BinaryString(bits=bits), templates=templates, raw=True)
        self.compare_match(BinaryString(bits=bits), templates=templates[:2], raw=True)

    def test_empty_catalogue_and_defined_short_vector_cases_match_original(self):
        for raw in (False, True):
            self.compare_match(
                BinaryString(type=6, score=0.75, position_label=77, toe_label=88),
                templates=(),
                raw=raw,
            )
            self.compare_match(
                BinaryString(bits=(0, 1, 9)),
                templates=(BinaryString(type=42, bits=(0, 1)),),
                raw=raw,
            )
            self.compare_match(BinaryString(bits=(0,) * 17), raw=raw)

    def compare_stacks(self, stacks, main_track=True):
        inputs = [int(main_track), len(stacks)]
        for stack in stacks:
            inputs.extend([*binary_state(stack.binary), len(stack.track_segments)])
            for segment in stack.track_segments:
                inputs.extend(
                    [
                        segment.track2.left_tag,
                        segment.track2.right_tag,
                        segment.track1.left_tag,
                        segment.track1.right_tag,
                    ]
                )
        native = self.reference.run("stack_recognition", *inputs)
        result = recognize_stacks(stacks, main_track=main_track)
        expected = [len(stacks)]
        for stack in stacks:
            expected.extend(binary_state(stack.binary))
        assert_allclose(expected, native, rtol=0, atol=0)
        return result

    def test_all_twenty_stack_patterns_match_original_postpass(self):
        self.compare_stacks([signature_stack(template.bits) for template in define_templates().all])

    def test_short_skipped_predecessor_and_main_flag_match_original_postpass(self):
        template = define_templates().all[0]
        for main_track in (False, True):
            stacks = [signature_stack(template.bits), TrackStack(), signature_stack(template.bits)]
            for length in range(5):
                stacks.insert(
                    1,
                    TrackStack(
                        signature_stack(template.bits).track_segments[:length],
                        BinaryString(type=7, score=0.25),
                    ),
                )
            self.compare_stacks(stacks, main_track=main_track)


class RecognitionIntegrationTests(unittest.TestCase):
    def test_terminal_tracker_recognizes_completed_stacks_and_preserves_snapshot_metadata(self):
        _, grid, rng, initialized = prepared(length=3)
        stack = signature_stack(define_templates().all[0].bits)
        tracker = SequentialTracker(
            initialized.layer, grid, initialized.initial_heights, rng=rng, completed_stacks=[stack]
        )
        limited = tracker.run(max_steps=1)
        self.assertIsNone(limited.recognition)
        result = tracker.run()
        self.assertEqual(result.status, "insufficient_window_points")
        self.assertEqual(result.recognition.entries[0].binary.type, 0)
        self.assertEqual(result.completed_stacks[0].binary.type, 0)
        self.assertEqual(stack.binary.type, 0)
        self.assertEqual(limited.completed_stacks[0].binary.type, -1)
        self.assertIs(tracker.run().recognition, result.recognition)

    def test_non_main_run_keeps_stack_metadata_and_active_stack_is_excluded(self):
        _, grid, rng, initialized = prepared(length=2)
        stack = signature_stack(define_templates().all[0].bits)
        tracker = SequentialTracker(
            initialized.layer,
            grid,
            initialized.initial_heights,
            rng=rng,
            main_track=False,
            completed_stacks=[stack],
        )
        result = tracker.run()
        self.assertEqual(result.recognition.entries[0].status, "main_track_disabled")
        self.assertEqual(stack.binary.type, -1)
        # A qualifying active stack is still excluded after a real count stop.
        config = ExtractionConfig(min_points=1000000)
        _, grid, rng, initialized = prepared(length=2, config=config)
        tracker = SequentialTracker(
            initialized.layer, grid, initialized.initial_heights, config=config, rng=rng
        )
        tracker.active_stack = signature_stack(define_templates().all[0].bits)
        result = tracker.run()
        self.assertEqual(result.status, "insufficient_window_points")
        self.assertEqual(result.recognition.entries, ())
        self.assertEqual(len(result.active_stack.track_segments), 6)
        self.assertEqual(result.active_stack.binary.type, -1)

    def test_recognition_demo_reports_all_exact_templates_and_duplicate_ambiguity(self):
        completed = subprocess.run(
            [sys.executable, "-m", "railway_mls", "recognition-demo"],
            text=True,
            capture_output=True,
            check=True,
        )
        result = json.loads(completed.stdout)
        self.assertEqual(result["template_count"], 20)
        self.assertEqual(result["ambiguous_count"], 4)
        self.assertEqual(result["matched_count"], 16)
        self.assertFalse(result["toe_detection_implemented"])


if __name__ == "__main__":
    unittest.main()
