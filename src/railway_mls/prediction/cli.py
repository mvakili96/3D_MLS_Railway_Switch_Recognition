"""Headless scene inventory and experiment validation; prediction comes later."""

import json
from pathlib import Path
import sys

from .catalog import catalog_scene, write_json_new
from .manifest import audit_prediction_manifest, load_inference_spec


def add_cli(commands):
    catalog = commands.add_parser("scene-catalog", help="inventory extracted LAS/LAZ headers and SHA256 hashes")
    catalog.add_argument("sources", nargs="+", help="files/directories, in declared input order")
    catalog.add_argument("--root", default=".", help="root for catalog-relative file paths")
    catalog.add_argument("--output", required=True, help="new catalog JSON filename")
    audit = commands.add_parser("prediction-audit", help="validate a prediction experiment without running tracking")
    audit.add_argument("--manifest", required=True)
    audit.add_argument("--data-root", help="override only the unannotated inference data root")
    audit.add_argument("--output", help="optional new audit JSON filename")
    build = commands.add_parser("build-scene-cache", help="build immutable multi-tile points and an ordered disk grid")
    build.add_argument("--manifest", required=True)
    build.add_argument("--data-root")
    build.add_argument("--output", required=True, help="new cache directory")
    build.add_argument("--chunk-size", type=int, default=250_000)
    cache = commands.add_parser("scene-cache-audit", help="verify cached payload hashes and optional inference compatibility")
    cache.add_argument("cache")
    cache.add_argument("--manifest")
    cache.add_argument("--data-root")


def run_cli(args):
    try:
        output_path = getattr(args, "output", None)
        if output_path and Path(output_path).exists():
            raise FileExistsError(f"output already exists: {args.output}")
        progress = lambda message: print(message, file=sys.stderr, flush=True)
        if args.command in ("build-scene-cache", "scene-cache-audit"):
            from .cache import build_scene_cache, open_scene_cache
            if args.data_root and not args.manifest:
                raise ValueError("--data-root requires --manifest")
            spec = load_inference_spec(args.manifest, data_root=args.data_root) if args.manifest else None
            cache = (build_scene_cache(spec, args.output, chunk_size=args.chunk_size, progress=progress)
                     if args.command == "build-scene-cache" else open_scene_cache(args.cache, inference=spec))
            metadata = cache.metadata
            print(json.dumps({"status": "ok", "cache": str(cache.path), "point_count": metadata["point_count"],
                              "cache_manifest_sha256": metadata["manifest_sha256"], "grid": metadata["grid"],
                              "build": metadata["build"], "tracking_executed": False}, indent=2, allow_nan=False))
            return 0
        if args.command == "scene-catalog":
            result = catalog_scene(args.sources, root=args.root, progress=progress)
        else:
            result = audit_prediction_manifest(args.manifest, data_root=args.data_root, progress=progress)
        if args.output:
            write_json_new(args.output, result)
        if args.command == "scene-catalog":
            output = {key: result[key] for key in ("status", "scene_fingerprint", "totals", "issues", "footprints")}
        else:
            output = {key: result[key] for key in ("status", "experiment_id", "readiness", "errors")}
        if args.output:
            output["output"] = str(Path(args.output))
        print(json.dumps(output, indent=2, allow_nan=False))
        return 0 if result["status"] in ("ok", "valid", "valid_with_pending") else 1
    except (OSError, ValueError, ImportError, KeyError, TypeError) as error:
        print(json.dumps({"status": getattr(error, "status", "failed"), "error": str(error)}), file=sys.stderr)
        return 1
