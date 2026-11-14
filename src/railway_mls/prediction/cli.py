"""Headless scene inventory and experiment validation; prediction comes later."""

import json
from pathlib import Path
import sys

from .catalog import catalog_scene, write_json_new
from .manifest import audit_prediction_manifest


def add_cli(commands):
    catalog = commands.add_parser("scene-catalog", help="inventory extracted LAS/LAZ headers and SHA256 hashes")
    catalog.add_argument("sources", nargs="+", help="files/directories, in declared input order")
    catalog.add_argument("--root", default=".", help="root for catalog-relative file paths")
    catalog.add_argument("--output", required=True, help="new catalog JSON filename")
    audit = commands.add_parser("prediction-audit", help="validate a prediction experiment without running tracking")
    audit.add_argument("--manifest", required=True)
    audit.add_argument("--data-root", help="override only the unannotated inference data root")
    audit.add_argument("--output", help="optional new audit JSON filename")


def run_cli(args):
    try:
        if args.output and Path(args.output).exists():
            raise FileExistsError(f"output already exists: {args.output}")
        progress = lambda message: print(message, file=sys.stderr, flush=True)
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
        print(json.dumps({"status": "failed", "error": str(error)}), file=sys.stderr)
        return 1
