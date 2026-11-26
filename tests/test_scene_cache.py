from dataclasses import replace
from hashlib import sha256
import importlib.util
import json
from pathlib import Path
import struct
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

import numpy as np
from numpy.testing import assert_array_equal

from railway_mls import Bounds3D, GridIndex, Point3D, PointCloud, MsvcRand
from railway_mls.legacy_io import LegacyLasReader
from railway_mls.orchestration import TrackingEngine
from railway_mls.prediction import (build_scene_cache, load_inference_spec, open_scene_cache,
                                    ResourceLimitError, SceneCacheError)
from railway_mls.prediction.manifest import TileSpec
from railway_mls.prediction.cache import grid_dimensions
from railway_mls.synthetic import tracking_scene
from railway_mls.tracking import SequentialTracker


ROOT = Path(__file__).resolve().parents[1]


@unittest.skipUnless(importlib.util.find_spec("laspy"), "optional LAS support")
class SceneCacheTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        rng = np.random.default_rng(5)
        xyz = rng.uniform([0, 0, 0], [4, 4, 2], (33, 3))
        xyz[0], xyz[-1] = [0, 0, 0], [4, 4, 2]
        self.write_las(self.root/"a.las", xyz)
        self.write_las(self.root/"b.las", xyz[:17]+[2, .1, 0])
        self.spec = self.make_spec()

    def write_las(self, path, xyz):
        import laspy
        header = laspy.LasHeader(version="1.2", point_format=1)
        header.scales = [.001]*3
        data = laspy.LasData(header)
        data.x, data.y, data.z = np.asarray(xyz).T
        data.intensity = (np.arange(len(xyz))*1001 % 65536).astype(np.uint16)
        data.classification = np.full(len(xyz), 3, dtype=np.uint8)
        data.write(path)

    def make_spec(self, paths=None, **kwargs):
        paths = paths if paths is not None else [self.root/"a.las", self.root/"b.las"]
        base = load_inference_spec(ROOT/"configs/prediction-vancouver-pilot.json")
        return replace(base, data_root=self.root, tiles=tuple(
            TileSpec(p, sha256(p.read_bytes()).hexdigest()) for p in paths), **kwargs)

    def reference(self, cache):
        loaded = [LegacyLasReader().read(tile.path) for tile in self.spec.tiles]
        cloud = PointCloud(np.concatenate([item.cloud.xyz for item in loaded]),
                           np.concatenate([item.cloud.intensity for item in loaded]))
        bounds = cache.metadata["bounds"]
        grid = GridIndex(cloud, self.spec.config.grid_size, bounds=Bounds3D(
            Point3D(*bounds["minimum"]), Point3D(*bounds["maximum"])))
        return cloud, grid, loaded

    def test_world_coordinates_intensity_identity_order_and_per_file_odd_parity(self):
        before = [tile.path.read_bytes() for tile in self.spec.tiles]
        cache = build_scene_cache(self.spec, self.root/"cache", chunk_size=3)
        cloud, _, loaded = self.reference(cache)
        with cache.new_run(self.root/"run") as scene:
            self.assertIsInstance(scene.cloud, PointCloud)
            self.assertIsInstance(scene.grid, GridIndex)
            assert_array_equal(scene.cloud.xyz, cloud.xyz)
            assert_array_equal(scene.cloud.intensity, cloud.intensity)
            assert_array_equal(scene.cloud.class_codes, 0)
            assert_array_equal(scene.cloud.segment_ids, -1)
            tiles, records = scene.cloud.source_records(np.arange(len(cloud)))
            assert_array_equal(tiles, np.repeat([0, 1], [len(item.cloud) for item in loaded]))
            assert_array_equal(records, np.concatenate([item.record_indices for item in loaded]))
            self.assertFalse(scene.cloud.xyz.flags.writeable)
            self.assertFalse(scene.cloud.intensity.flags.writeable)
        self.assertEqual(before, [tile.path.read_bytes() for tile in self.spec.tiles])

    def test_grid_matches_cell_members_and_every_neighborhood_order_and_sentinel(self):
        cache = build_scene_cache(self.spec, self.root/"cache", chunk_size=5)
        _, reference, _ = self.reference(cache)
        with cache.new_run(self.root/"run") as scene:
            grid = scene.grid
            self.assertEqual(grid.occupied_cells, reference.occupied_cells)
            for cell in range(grid.num_cells):
                assert_array_equal(grid.members(cell).indices, reference.members(cell).indices)
                self.assertEqual(grid.cell_bounds(cell), reference.cell_bounds(cell))
            for x, y in ((-.1, -.1), (0, 0), (2, 2), (6, 4), (6.001, 5)):
                for size in (0, 1, 2, 5):
                    self.assertEqual(grid.neighbor_indices(x, y, size), reference.neighbor_indices(x, y, size))
                    assert_array_equal(grid.neighbor_points(x, y, size).indices,
                                       reference.neighbor_points(x, y, size).indices)
            for cell in (-1, grid.num_cells):
                with self.assertRaises(IndexError):
                    grid.members(cell)

    def test_nonunit_source_floor_before_division_and_maximum_clamping(self):
        for size in (.5, 2.0):
            with self.subTest(size=size):
                self.spec = replace(self.spec, config=replace(self.spec.config, grid_size=size))
                cache = build_scene_cache(self.spec, self.root/f"cache{size}", chunk_size=7)
                _, ref, _ = self.reference(cache)
                with cache.new_run(self.root/f"run{size}") as scene:
                    for cell in range(ref.num_cells):
                        assert_array_equal(scene.grid.members(cell).indices, ref.members(cell).indices)

    def test_sparse_index_does_not_allocate_by_empty_grid_extent(self):
        self.write_las(self.root/"a.las", [[0, 0, 0], [0, 0, 0], [1e6, 1e6, 0]])
        self.spec = self.make_spec([self.root/"a.las"])
        cache = build_scene_cache(self.spec, self.root/"cache", chunk_size=1)
        self.assertEqual(cache.metadata["grid"]["num_cells"], 10**12)
        self.assertEqual(cache.metadata["grid"]["occupied_cells"], 2)
        self.assertLess(sum(p.stat().st_size for p in cache.path.iterdir()), 20_000)

    def test_overlapping_tiles_keep_duplicate_points_and_distinct_record_identities(self):
        self.write_las(self.root/"b.las", [[0, 0, 0]]*3)
        self.spec = self.make_spec()
        cache = build_scene_cache(self.spec, self.root/"cache", chunk_size=1)
        with cache.new_run(self.root/"run") as scene:
            end = len(scene.cloud)
            assert_array_equal(scene.cloud.select([0, end-2, end-1]).xyz, [[0, 0, 0]]*3)
            tiles, records = scene.cloud.source_records([0, end-2, end-1])
            assert_array_equal(tiles, [0, 1, 1])
            assert_array_equal(records, [0, 0, 2])

    def test_payloads_and_membership_are_independent_of_decode_chunk_size(self):
        one = build_scene_cache(self.spec, self.root/"one", chunk_size=1)
        two = build_scene_cache(self.spec, self.root/"two", chunk_size=16)
        self.assertEqual(one.metadata["compatibility_fingerprint"], two.metadata["compatibility_fingerprint"])
        self.assertEqual(one.metadata["files"], two.metadata["files"])

    def test_label_updates_share_within_run_and_fresh_runs_are_independent(self):
        cache = build_scene_cache(self.spec, self.root/"cache")
        with cache.new_run(self.root/"one") as one, cache.new_run(self.root/"two") as two:
            one.cloud.select([0, 2, 2]).set_class_code(7)
            one.cloud.select([2]).set_segment_id(9)
            assert_array_equal(one.cloud.select([2, 0]).cloud.class_codes[[2, 0]], [7, 7])
            self.assertEqual(one.cloud.segment_ids[2], 9)
            assert_array_equal(two.cloud.class_codes, 0)
            assert_array_equal(two.cloud.segment_ids, -1)
            one.cloud.release_pages()
            self.assertEqual(one.cloud.class_codes[2], 7)
        self.assertEqual(open_scene_cache(cache.path).metadata["files"], cache.metadata["files"])

    def test_private_trials_share_the_cache_with_independent_live_labels(self):
        cache = build_scene_cache(self.spec, self.root / "cache")
        with (
            cache.new_run(self.root / "template") as template,
            template.private_run() as first,
            template.private_run() as second,
        ):
            first.cloud.select([2]).set_class_code(7)
            first.cloud.select([2]).set_segment_id(9)
            first.cloud.release_pages()
            self.assertEqual((first.cloud.class_codes[2], first.cloud.segment_ids[2]), (7, 9))
            self.assertEqual((second.cloud.class_codes[2], second.cloud.segment_ids[2]), (0, -1))
            self.assertEqual((template.cloud.class_codes[2], template.cloud.segment_ids[2]), (0, -1))
            assert_array_equal(first.grid.members(0).xyz, second.grid.members(0).xyz)
        assert_array_equal(np.fromfile(template.path / "class_codes.bin", dtype="<i4"), 0)
        assert_array_equal(np.fromfile(template.path / "segment_ids.bin", dtype="<i4"), -1)

    def test_repeated_existing_cache_or_run_paths_never_overwrite(self):
        cache = build_scene_cache(self.spec, self.root/"cache")
        with self.assertRaises(FileExistsError):
            build_scene_cache(self.spec, self.root/"cache")
        with cache.new_run(self.root/"run"):
            pass
        with self.assertRaises(FileExistsError):
            cache.new_run(self.root/"run")

    def test_closed_selection_fails_without_accessing_unmapped_storage(self):
        cache = build_scene_cache(self.spec, self.root/"cache")
        with cache.new_run(self.root/"run") as scene:
            selection = scene.cloud.select([0])
        with self.assertRaises(RuntimeError):
            _ = selection.xyz
        with self.assertRaises(RuntimeError):
            selection.set_class_code(8)

    def test_payload_and_manifest_corruption_are_detected(self):
        cache = build_scene_cache(self.spec, self.root/"cache")
        path = cache.path/"members.bin"
        payload = path.read_bytes()
        path.write_bytes(bytes([payload[0]^1])+payload[1:])
        with self.assertRaisesRegex(SceneCacheError, "payload SHA256"):
            open_scene_cache(cache.path)
        path.write_bytes(payload+b"extra")
        with self.assertRaisesRegex(SceneCacheError, "length mismatch"):
            open_scene_cache(cache.path)
        path.write_bytes(payload)
        manifest = cache.path/"cache.json"
        text = json.loads(manifest.read_text())
        text["point_count"] += 1
        manifest.write_text(json.dumps(text))
        with self.assertRaisesRegex(SceneCacheError, "manifest SHA256"):
            open_scene_cache(cache.path)

    def test_policy_grid_and_order_changes_are_incompatible_but_fitting_and_seed_changes_are_not(self):
        cache = build_scene_cache(self.spec, self.root/"cache")
        invalid = [replace(self.spec, tiles=tuple(reversed(self.spec.tiles))),
                   replace(self.spec, header_roundoff="reject_outside"),
                   replace(self.spec, config=replace(self.spec.config, grid_size=.5))]
        for spec in invalid:
            with self.subTest(spec=spec), self.assertRaisesRegex(SceneCacheError, "incompatible"):
                open_scene_cache(cache.path, inference=spec)
        valid = replace(self.spec, config=replace(self.spec.config, mdl_weight=.7), runs=())
        self.assertEqual(open_scene_cache(cache.path, inference=valid).metadata["point_count"],
                         cache.metadata["point_count"])

    def test_cached_snapshot_can_be_verified_without_access_to_original_las(self):
        cache = build_scene_cache(self.spec, self.root/"cache")
        for tile in self.spec.tiles:
            tile.path.unlink()
        self.assertEqual(open_scene_cache(cache.path, inference=self.spec).metadata["point_count"], 26)

    def patch_min_x(self, value):
        with (self.root/"a.las").open("r+b") as stream:
            stream.seek(187)  # LAS 1.2 Min X header field, without rewriting points.
            stream.write(struct.pack("<d", value))
        self.spec = self.make_spec([self.root/"a.las"])

    def test_named_ulp_policy_omits_only_outside_records_and_keeps_original_ids(self):
        self.patch_min_x(np.nextafter(0., np.inf))
        cache = build_scene_cache(self.spec, self.root/"cache", chunk_size=3)
        self.assertEqual(cache.metadata["build"]["omitted_header_roundoff"], 1)
        self.assertEqual(cache.metadata["bounds"]["minimum"][0], np.nextafter(0., np.inf))
        with cache.new_run(self.root/"run") as scene:
            _, ids = scene.cloud.source_records(np.arange(len(scene.cloud)))
            assert_array_equal(ids, np.arange(2, 33, 2))

    def test_strict_bounds_or_larger_errors_fail_and_remove_partial_payloads(self):
        self.patch_min_x(np.nextafter(0., np.inf))
        with self.assertRaisesRegex(SceneCacheError, "outside declared"):
            build_scene_cache(replace(self.spec, header_roundoff="reject_outside"), self.root/"strict")
        self.assertFalse((self.root/"strict/cache.json").exists())
        self.assertTrue((self.root/"strict/failure.json").exists())
        self.assertFalse(list((self.root/"strict").glob("*.bin")))
        self.patch_min_x(1e-5)
        with self.assertRaisesRegex(SceneCacheError, "four-ULP"):
            build_scene_cache(self.spec, self.root/"outside")

    def test_hash_drift_fails_before_creating_cache(self):
        with self.spec.tiles[0].path.open("ab") as stream:
            stream.write(b"new bytes")
        with self.assertRaisesRegex(SceneCacheError, "SHA256 mismatch"):
            build_scene_cache(self.spec, self.root/"cache")
        self.assertFalse((self.root/"cache").exists())

    def test_memory_and_disk_limits_fail_without_thinning_or_creating_cache(self):
        for spec in (replace(self.spec, memory_bytes=1), replace(self.spec, cache_disk_bytes=1)):
            with self.subTest(spec=spec), self.assertRaises(ResourceLimitError):
                build_scene_cache(spec, self.root/"cache")
            self.assertFalse((self.root/"cache").exists())

    def test_query_and_fresh_state_respect_budget_failure(self):
        cache = build_scene_cache(self.spec, self.root/"cache")
        with cache.new_run(self.root/"run") as scene:
            scene.cloud.memory.limit = 1
            with self.assertRaises(ResourceLimitError):
                scene.cloud.select([0])
        cache.disk_bytes = 1
        with self.assertRaises(ResourceLimitError):
            cache.new_run(self.root/"too-large")
        self.assertFalse((self.root/"too-large").exists())

    def test_unsafe_int64_grid_extent_is_explicit(self):
        with self.assertRaisesRegex(SceneCacheError, "int64"):
            grid_dimensions({"minimum": [0, 0, 0], "maximum": [1e15, 1e15, 0]}, 1.)

    def test_public_builder_rejects_unsupported_policies_before_claiming_cached_behavior(self):
        for spec in (replace(self.spec, record_selection="all_records"),
                     replace(self.spec, header_roundoff="clamp"),
                     replace(self.spec, source_baseline="2018")):
            with self.subTest(spec=spec), self.assertRaisesRegex(SceneCacheError, "unsupported"):
                build_scene_cache(spec, self.root/"cache")
            self.assertFalse((self.root/"cache").exists())

    def test_invalid_cache_schema_reports_an_explicit_failure(self):
        directory = self.root/"badcache"
        directory.mkdir()
        (directory/"cache.json").write_text("[]")
        with self.assertRaisesRegex(SceneCacheError, "supported scene cache"):
            open_scene_cache(directory)

    def test_tracking_prefix_matches_reference_backend_vectors_labels_and_rng(self):
        scene = tracking_scene()
        # Duplicate each record so source even-record selection reconstructs the
        # same generated input. Split it into two files, retaining global order.
        middle = len(scene.cloud)//2
        for name, values in (("a.las", scene.cloud.xyz[:middle]), ("b.las", scene.cloud.xyz[middle:])):
            self.write_las(self.root/name, np.repeat(values, 2, axis=0))
        self.spec = self.make_spec(config=scene.config)
        cache = build_scene_cache(self.spec, self.root/"cache", chunk_size=37)
        reference, grid, _ = self.reference(cache)
        with cache.new_run(self.root/"run") as mapped:
            results, randoms = [], []
            for backend in (grid, mapped.grid):
                random = MsvcRand(1)
                engine = TrackingEngine(backend, config=scene.config, rng=random)
                init = engine.initialize(scene.seed)
                self.assertTrue(init.success, init.status)
                result = SequentialTracker(engine.current_layer, backend, init.initial_heights,
                                           config=scene.config, rng=random).run(max_steps=3)
                results.append(result)
                randoms.append((random.state, random.draw_count))
            a, b = results
            self.assertGreater(a.accepted_steps, 0)
            self.assertEqual((a.status, a.accepted_steps), (b.status, b.accepted_steps))
            for name in ("rail1_vector", "rail2_vector", "center_vector"):
                assert_array_equal([p.as_array() for p in getattr(a.layer, name)],
                                   [p.as_array() for p in getattr(b.layer, name)])
            assert_array_equal(reference.class_codes, mapped.cloud.class_codes)
            assert_array_equal(reference.segment_ids, mapped.cloud.segment_ids)
            self.assertEqual(randoms[0], randoms[1])

    def test_cache_cli_build_audit_and_incomplete_cache_failure(self):
        document = json.loads((ROOT/"configs/prediction-vancouver-pilot.json").read_text())
        document["inference"]["data_root"] = str(self.root)
        document["inference"]["tiles"] = [{"path": t.path.name, "sha256": t.sha256} for t in self.spec.tiles]
        path = self.root/"experiment.json"
        path.write_text(json.dumps(document))
        base = [sys.executable, "-m", "railway_mls"]
        result = subprocess.run([*base, "build-scene-cache", "--manifest", str(path),
                                 "--output", str(self.root/"cache"), "--chunk-size", "3"],
                                capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertFalse(json.loads(result.stdout)["tracking_executed"])
        result = subprocess.run([*base, "scene-cache-audit", str(self.root/"cache"), "--manifest", str(path)],
                                capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stderr)
        (self.root/"incomplete").mkdir()
        result = subprocess.run([*base, "scene-cache-audit", str(self.root/"incomplete")],
                                capture_output=True, text=True)
        self.assertEqual(result.returncode, 1)


if __name__ == "__main__":
    unittest.main()
