"""Scene input, source prediction and explicit benchmark workflow commands."""

import json
from pathlib import Path
import sys

from .catalog import catalog_scene, write_json_new
from .manifest import audit_prediction_manifest, load_inference_spec


def add_cli(commands):
    preview = commands.add_parser("benchmark-preview", help="plot frozen layers against original routes for review without calculating scores")
    preview.add_argument("--manifest", required=True)
    preview.add_argument("--predictions", action="append", required=True)
    preview.add_argument("--output", required=True)
    seed = commands.add_parser("prepare-benchmark-seed", help="apply a saved manual starting pose to a new experiment definition")
    seed.add_argument("--manifest", required=True)
    seed.add_argument("--run", required=True)
    seed.add_argument("--capture", required=True)
    seed.add_argument("--provenance", required=True)
    seed.add_argument("--output", required=True)
    for name, help_text in (("benchmark-plan", "account for all nine paths without reading point payloads"),
                            ("benchmark-rails", "run declared ready starts and preserve missing-path accounting"),
                            ("benchmark-evaluate", "evaluate reviewed benchmark layers and retain unavailable path scores")):
        parser = commands.add_parser(name, help=help_text)
        parser.add_argument("--manifest", required=True)
        parser.add_argument("--output", required=True)
        if name == "benchmark-rails":
            parser.add_argument("--cache", required=True)
            parser.add_argument("--run", action="append")
            parser.add_argument("--max-steps", type=int)
            parser.add_argument("--retain-history", action="store_true")
        if name == "benchmark-evaluate":
            parser.add_argument("--predictions", action="append", required=True, help="benchmark batch output; repeat for disjoint run batches")
            parser.add_argument("--mapping", required=True, help="explicit reviewed matched-evaluation manifest")
            parser.add_argument("--chunk-size", type=int, default=250_000)
            parser.add_argument("--point-outcomes", action="store_true")
            parser.add_argument("--overlay", action="store_true")
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
    predict = commands.add_parser("predict-rails", help="run source tracking over a verified multi-tile cache")
    predict.add_argument("--manifest", required=True)
    predict.add_argument("--cache", required=True)
    predict.add_argument("--output", required=True, help="new prediction output directory")
    predict.add_argument("--run", action="append", help="select a ready run ID; default all ready runs")
    predict.add_argument("--data-root")
    predict.add_argument("--max-steps", type=int, help="explicit diagnostic bound per source invocation")
    predict.add_argument("--retain-history", action="store_true", help="retain full diagnostic step objects")
    probe = commands.add_parser(
        "probe-switch", help="trace separate rail-pair candidates at a selected tracking window"
    )
    probe.add_argument("--manifest", required=True)
    probe.add_argument("--cache", required=True)
    probe.add_argument("--run", required=True, help="ready source run ID")
    probe.add_argument("--split-step", type=int, required=True, help="one-based forward window")
    probe.add_argument("--lookahead-steps", type=int, required=True)
    probe.add_argument("--output", required=True, help="new directory for the probe")
    probe.add_argument("--data-root")
    branches = commands.add_parser(
        "track-branches", help="discover supported switch continuations during scene tracking"
    )
    branches.add_argument("--manifest", required=True)
    branches.add_argument("--cache", required=True)
    branches.add_argument("--run", required=True, help="ready source run ID")
    branches.add_argument("--output", required=True, help="new directory for the branch run")
    branches.add_argument("--data-root")
    branches.add_argument("--max-steps", type=int, help="bound along each route from its initial seed")
    branches.add_argument("--max-branch-attempts", type=int)


def run_cli(args):
    try:
        output_path = getattr(args, "output", None)
        if output_path and Path(output_path).exists():
            raise FileExistsError(f"output already exists: {args.output}")
        progress = lambda message: print(message, file=sys.stderr, flush=True)
        if args.command == "benchmark-preview":
            from .review import preview_benchmark
            report = preview_benchmark(args.manifest, args.predictions, args.output)
            print(json.dumps({"status": report["status"], "output": str(Path(args.output)/"review.json"),
                              "plots": len(report["plots"]), "evaluation_performed": False}, indent=2))
            return 0
        if args.command == "prepare-benchmark-seed":
            from .seeds import prepare_seed_manifest
            prepare_seed_manifest(args.manifest, args.run, args.capture, args.output, provenance=args.provenance)
            print(json.dumps({"status": "ready", "run_id": args.run, "new_manifest": args.output,
                              "prediction_executed": False}, indent=2))
            return 0
        if args.command.startswith("benchmark-"):
            from .benchmark import plan_benchmark, run_benchmark, evaluate_benchmark
            if args.command == "benchmark-plan":
                report = plan_benchmark(args.manifest, args.output)
            elif args.command == "benchmark-rails":
                report = run_benchmark(args.manifest, args.cache, args.output, run_ids=args.run,
                                       max_steps=args.max_steps, retain_history=args.retain_history, progress=progress)
            else:
                report = evaluate_benchmark(args.manifest, args.predictions, args.mapping, args.output,
                                            chunk_size=args.chunk_size, point_outcomes=args.point_outcomes,
                                            overlay=args.overlay, progress=progress)
            print(json.dumps({"status": report["status"], "output": str(Path(args.output)/"report.json"),
                              "accounted_paths": report["accounted_path_count"], "accounted_cases": report["accounted_case_count"],
                              "all_paths_have_seeds": report["all_paths_have_seeds"],
                              "all_reference_cases_evaluated": report["all_reference_cases_evaluated"],
                              "paths": [{k: row[k] for k in ("path_id", "prediction_status", "evaluation_status")}
                                        for row in report["paths"]]}, indent=2, allow_nan=False))
            if report["status"] == "interrupted" or any(r["status"] == "interrupted" for r in report["run_summaries"]):
                return 130
            return 1 if report["status"] == "failed" else 0 if report["status"] == "evaluated" else 2
        if args.command == "track-branches":
            from .branching import track_branches
            try:
                report = track_branches(
                    args.manifest, args.cache, args.output, run_id=args.run,
                    max_steps=args.max_steps, max_branch_attempts=args.max_branch_attempts,
                    data_root=args.data_root, progress=progress,
                )
            except KeyboardInterrupt:
                return 130
            if "error" in report:
                print(json.dumps({"status": report["status"], **report["error"]}), file=sys.stderr)
                return 1
            print(json.dumps({
                "status": report["status"],
                "output": args.output,
                "source_stop": report["source"]["status"],
                "branch_attempts": len(report["branches"]),
                "confirmed_branches": [
                    {key: branch[key] for key in
                     ("route", "parent", "parent_window", "status", "accepted_steps", "length_xy_m")}
                    for branch in report["branches"] if branch["confirmed_at"] is not None
                ],
                "pending_candidates": report["pending_candidates"],
            }, indent=2, allow_nan=False))
            return 0 if report["status"] == "queue_drained" else 2
        if args.command == "probe-switch":
            from .switch_probe import probe_switch
            try:
                report = probe_switch(
                    args.manifest, args.cache, args.output, run_id=args.run,
                    split_step=args.split_step, lookahead_steps=args.lookahead_steps,
                    data_root=args.data_root, progress=progress,
                )
            except KeyboardInterrupt:
                return 130
            if "error" in report:
                print(json.dumps({"status": report["status"], **report["error"]}), file=sys.stderr)
                return 1
            print(json.dumps({
                "status": report["status"],
                "output": args.output,
                "candidate_pairs": len(report["pairs"]),
                "branches": [
                    {key: branch[key] for key in
                     ("pair_indices", "status", "accepted_steps", "length_xy_m")}
                    for branch in report["branches"]
                ],
            }, indent=2, allow_nan=False))
            return 0 if report["status"] == "lookahead_complete" else 2
        if args.command == "predict-rails":
            from .runner import run_predictions
            result = run_predictions(args.manifest, args.cache, args.output, run_ids=args.run,
                                     data_root=args.data_root, retain_history=args.retain_history,
                                     max_steps=args.max_steps, progress=progress)
            print(json.dumps({"status": result["status"], "output": str(Path(args.output)/"manifest.json"),
                              "runs": [{"run_id": r["run_id"], "status": r["status"],
                                        "coverage": r.get("coverage"), "failure": r.get("failure")}
                                       for r in result["runs"]],
                              "pending_seed_runs": result["pending_seed_runs"],
                              "nine_path_complete": False}, indent=2, allow_nan=False))
            statuses = [r["status"] for r in result["runs"]]
            return (130 if "interrupted" in statuses else 0 if result["status"] != "partial"
                    else 2 if all(s in ("step_limit", "completed", "completed_with_limitations") for s in statuses) else 1)
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
