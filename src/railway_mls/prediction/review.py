"""Post-run route inspection without calculating or selecting evaluation scores."""

from pathlib import Path

import numpy as np

from ..evaluation.io import file_fingerprint, read_rail_segments
from ..evaluation.matched import _fragment, _pinned, _source
from ..evaluation.overlay import require_plotting
from .benchmark import _contract, _exports, _new_output
from .catalog import write_json_new


def preview_benchmark(manifest, prediction_outputs, output):
    contract = _contract(manifest)
    exports, _ = _exports(contract, prediction_outputs)
    matplotlib = require_plotting()
    matplotlib.use("Agg")
    from matplotlib import pyplot as plt
    from matplotlib.collections import LineCollection
    sources, fragments, originals = {}, [], {}
    for i, fingerprint in enumerate(exports.values()):
        sid, source = _source({"source_id": f"batch-{i}", "directory": str(Path(fingerprint["path"]).parent),
                               "manifest_sha256": fingerprint["sha256"],
                               "inference_manifest": contract["manifest"]["path"]}, Path.cwd())
        sources[sid] = source
        for rid, run in source["runs"].items():
            leaf = source["directory"]/rid/"manifest.json"
            for layer in run.get("layers", []):
                if not all(layer.get("vertices", {}).get(side, 0) > 1 for side in ("rail1", "rail2")):
                    continue
                fragments.append(_fragment({"source_id": sid, "run_id": rid, "layer": layer["layer"],
                                            "run_manifest_sha256": file_fingerprint(leaf)["sha256"]}, sources))
    if not fragments:
        raise ValueError("no exported layers with paired rail segments to preview")
    for pid in range(1, 10):
        case = next(case for case in contract["cases"] if case["path_id"] == pid)
        originals[pid] = []
        for side in ("left", "right"):
            _pinned(case[side], case["sha256"][side])
            originals[pid].append(read_rail_segments(case[side]))
    output = _new_output(output, contract, prediction_outputs)
    plots = []
    for fragment in fragments:
        rid, layer = fragment["run_id"], fragment["layer"]
        seed = np.array(fragment["run_summary"]["seed"]["xyz"][:2])
        xy = np.concatenate([rail.endpoints[:, :, :2].reshape(-1, 2) for rail in fragment["rails"]])
        fig, axes = plt.subplots(3, 1, figsize=(13, 10), constrained_layout=True)
        try:
            for ax in axes:
                for pid, rails in originals.items():
                    for side, rail in enumerate(rails):
                        ax.add_collection(LineCollection(rail.endpoints[:, :, :2]-seed, colors=[f"C{pid-1}"],
                                                         linewidths=1.2, label=f"Original path {pid}" if side == 0 else None))
                for side, rail in enumerate(fragment["rails"]):
                    ax.add_collection(LineCollection(rail.endpoints[:, :, :2]-seed, colors="black", linewidths=2,
                                                     label=f"Python layer {layer}" if side == 0 else None))
                ax.scatter([0], [0], marker="*", s=70, c="black", label="Supplied seed")
                ax.autoscale()
                ax.set_aspect("equal", adjustable="box")
                ax.set_xlabel(f"Easting - {seed[0]:.3f} (m)")
                ax.set_ylabel(f"Northing - {seed[1]:.3f} (m)")
            axes[0].set_title("All nine original model routes and frozen Python output")
            for axis, limits, title in ((axes[1], (xy.min(axis=0)-seed-5, xy.max(axis=0)-seed+5), "Prediction extent"),
                                        (axes[2], ([-20, -20], [70, 40]), "Starting region (inspection only)")):
                axis.set_xlim(limits[0][0], limits[1][0])
                axis.set_ylim(limits[0][1], limits[1][1])
                axis.set_title(title)
            axes[2].legend(loc="best", ncol=3, fontsize=8)
            fig.suptitle(f"{rid}, layer {layer}: explicit route review; no scores calculated")
            filename = output/f"{fragment['source_id']}-{rid}-layer-{layer}.png"
            fig.savefig(filename, dpi=140)
        finally:
            plt.close(fig)
        plots.append({**file_fingerprint(filename), "run_id": rid, "layer": layer,
                      "intended_path_id": contract["path_by_run"][rid]})
    report = {"schema_version": 1, "kind": "benchmark_layer_review", "status": "requires_review",
              "prediction_exports": list(exports.values()), "plots": plots,
              "evaluation_performed": False, "reference_point_records_read": False,
              "matching_policy": "Human review of frozen route identity; never copy reference geometry into seeds/fitting or select by scores"}
    write_json_new(output/"review.json", report)
    return report
