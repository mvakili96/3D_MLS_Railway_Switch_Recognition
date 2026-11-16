"""Static inspection plots. Display sampling never enters the scoring path."""

import json
from pathlib import Path

import numpy as np

from .io import file_fingerprint, iter_reference_las, read_rail_segments


def require_plotting():
    try:
        import matplotlib
    except ImportError as error:
        raise ImportError("Evaluation overlays require: pip install -e '.[evaluation]'") from error
    return matplotlib


def _verified_file(fingerprint):
    path = Path(fingerprint["path"])
    if file_fingerprint(path)["sha256"] != fingerprint["sha256"]:
        raise ValueError(f"{path}: SHA256 mismatch before overlay")
    return path


def plot_matched_evaluation(report_path, *, max_points=200_000):
    """Plot complete path scope plus a pilot zoom, one new PNG per reviewed path.

    Grey is the reference LAS cloud; orange is raw classification byte 3. These
    are display samples only. All records were scored in the paired reports.
    """
    if isinstance(max_points, bool) or not isinstance(max_points, int) or max_points <= 0:
        raise ValueError("max_points must be a positive integer")
    matplotlib = require_plotting()
    matplotlib.use("Agg")
    from matplotlib import pyplot as plt
    from matplotlib.collections import LineCollection
    report_path = Path(report_path).resolve()
    report = json.loads(report_path.read_text())
    if report.get("kind") != "matched_prediction_evaluation" or report.get("status") != "completed":
        raise ValueError("overlay requires a completed matched evaluation report")
    original = json.loads(_verified_file(report["models"]["original"]["report"]).read_text())
    outputs = []
    destinations = [report_path.parent/f"path-{m['path_id']}-overlay.png" for m in report["mappings"]]
    if any(path.exists() for path in destinations):
        raise FileExistsError("overlay already exists; use a new evaluation output directory")
    for mapping, destination in zip(report["mappings"], destinations):
        pid = mapping["path_id"]
        cases = [c for c in original["cases"] if c["case_id"] in mapping["case_ids"]]
        stride = max(1, (sum(c["coverage"]["total"] for c in cases)+max_points-1)//max_points)
        samples, labels, offset = [], [], 0
        for case in cases:
            reference = _verified_file(case["inputs"]["las"])
            for chunk in iter_reference_las(reference):
                keep = (chunk.record_ids+offset) % stride == 0
                samples.append(chunk.xyz[keep, :2])
                labels.append(chunk.raw_classification[keep] == 3)
            offset += case["coverage"]["total"]
        xy = np.concatenate(samples) if samples else np.empty((0, 2))
        positive = np.concatenate(labels) if labels else np.empty(0, dtype=bool)
        models = {name: [read_rail_segments(_verified_file(files[side])) for side in ("left", "right")]
                  for name, files in mapping["model_files"].items()}
        pilot = np.concatenate([rail.endpoints.reshape(-1, 3)[:, :2] for rail in models["python"]])
        origin = pilot[0]
        fig, axes = plt.subplots(1, 2, figsize=(15, 6), constrained_layout=True)
        try:
            for ax in axes:
                ax.scatter(*(xy[~positive]-origin).T, s=.5, c="0.75", rasterized=True,
                           label="Reference cloud (display sample)")
                ax.scatter(*(xy[positive]-origin).T, s=1.5, c="#d98b00", rasterized=True,
                           label="Reference positives (byte 3)")
                for name, color in (("original", "#377eb8"), ("python", "#151515")):
                    for side, rail in enumerate(models[name]):
                        ax.add_collection(LineCollection(rail.endpoints[:, :, :2]-origin, colors=color,
                                                         linewidths=1.3 if name == "original" else 1.8,
                                                         label=f"{name.title()} predictions" if side == 0 else None))
                ax.autoscale()
                ax.set_aspect("equal", adjustable="box")
                ax.set_xlabel(f"Easting - {origin[0]:.3f} (m)")
                ax.set_ylabel(f"Northing - {origin[1]:.3f} (m)")
            axes[0].set_title(f"Path {pid}: all {len(cases)} reference parts")
            lower, upper = pilot.min(axis=0)-5, pilot.max(axis=0)+5
            axes[1].set_xlim(lower[0]-origin[0], upper[0]-origin[0])
            axes[1].set_ylim(lower[1]-origin[1], upper[1]-origin[1])
            axes[1].set_title("Frozen Python prediction extent (zoom only)")
            axes[1].legend(loc="best", fontsize=8)
            fig.suptitle("Post-run inspection; full unsampled reference scope used for scoring")
            fig.savefig(destination, dpi=160)
        finally:
            plt.close(fig)
        outputs.append({**file_fingerprint(destination), "path_id": pid, "display_stride": stride,
                        "display_records": len(xy), "evaluation_sampling": "none"})
    return outputs
