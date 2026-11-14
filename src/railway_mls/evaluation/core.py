"""Fresh-session evaluation with disk-backed, source-ordered error reductions."""

from pathlib import Path
from tempfile import TemporaryDirectory

import numpy as np

from .geometry import assign_points, prepare_line, residuals
from .types import EvaluationConfig, EvaluationResult, POINT_DTYPE, ReferencePoints, RailSegments


def _ordered_add(total, values):
    if len(values):
        # cumsum, unlike sum, retains the source's sequential additions. Including
        # the previous total preserves that order across input/spool chunk sizes.
        return float(np.cumsum(np.concatenate(([total], values)))[-1])
    return total


def _read_residuals(path, block_size=100_000):
    with path.open("rb") as stream:
        while True:
            block = np.fromfile(stream, dtype=np.float64, count=2*block_size)
            if not len(block):
                break
            yield block.reshape(-1, 2)


def evaluate_chunks(chunks, left: RailSegments, right: RailSegments, *,
                    config=None, use_index=True, collect_points=False, point_callback=None):
    """Consume increasing original record IDs; never alter reference annotations.

    Temporary files contain assigned positive residuals per segment, in original
    record order. This bounds RAM while retaining the source's segment-first,
    point-second arithmetic in both statistics passes.
    """
    config = EvaluationConfig() if config is None else config
    if not isinstance(config, EvaluationConfig):
        raise TypeError("config must be EvaluationConfig")
    lines = [prepare_line(segment, config.candidate_buffer)
             for rails in (left, right) for segment in rails.endpoints]
    counts = dict(nTP=0, nFP=0, nTN=0, nFN=0)
    coverage = dict(total=0, positive=0, background=0, assigned_positive=0,
                    assigned_background=0, unassigned_positive=0, unassigned_background=0)
    collected, active = [], set()
    last_id = -1
    with TemporaryDirectory(prefix="railway-mls-evaluation-") as temp:
        directory = Path(temp)
        for chunk in chunks:
            if not isinstance(chunk, ReferencePoints):
                raise TypeError("chunks must contain ReferencePoints")
            if len(chunk) and chunk.record_ids[0] <= last_id:
                raise ValueError("record IDs must increase across chunks")
            if len(chunk):
                last_id = int(chunk.record_ids[-1])
            assignment = assign_points(chunk.xyz, lines, use_index=use_index)
            positive = chunk.raw_classification == 3
            assigned = assignment >= 0
            coverage["total"] += len(chunk)
            for name, mask in (("positive", positive), ("background", ~positive),
                               ("assigned_positive", assigned & positive),
                               ("assigned_background", assigned & ~positive),
                               ("unassigned_positive", ~assigned & positive),
                               ("unassigned_background", ~assigned & ~positive)):
                coverage[name] += int(np.count_nonzero(mask))
            output = np.empty(len(chunk), dtype=POINT_DTYPE)
            output["record_id"], output["segment_id"] = chunk.record_ids, assignment
            output["raw_classification"] = chunk.raw_classification
            output["outcome"] = 0  # native ClassCodeRe is uninitialized when unassigned
            output["horizontal"] = output["signed_vertical"] = np.nan
            indices = np.flatnonzero(assigned)
            indices = indices[np.argsort(assignment[indices], kind="stable")]
            boundaries = np.r_[0, np.flatnonzero(np.diff(assignment[indices]))+1, len(indices)]
            for start, end in zip(boundaries[:-1], boundaries[1:]):
                ids = indices[start:end]
                if not len(ids):
                    continue
                segment = int(assignment[ids[0]])
                horizontal, vertical = residuals(chunk.xyz[ids], lines[segment])
                output["horizontal"][ids], output["signed_vertical"][ids] = horizontal, vertical
                is_head = positive[ids]
                height = np.abs(vertical)
                head_pass = (horizontal <= config.horizontal_threshold) & (height <= config.vertical_threshold)
                background_pass = (horizontal < config.horizontal_threshold) & (height < config.vertical_threshold)
                codes = np.where(is_head, np.where(head_pass, 1, 3), np.where(background_pass, 4, 2))
                output["outcome"][ids] = codes
                for code, name in ((1, "nTP"), (3, "nFP"), (4, "nFN"), (2, "nTN")):
                    counts[name] += int(np.count_nonzero(codes == code))
                if np.any(is_head):
                    active.add(segment)
                    with (directory / str(segment)).open("ab") as stream:
                        np.column_stack((horizontal[is_head], vertical[is_head])).tofile(stream)
            if point_callback is not None:
                point_callback(output)
            if collect_points:
                collected.append(output)
        total_h, total_v = 0.0, 0.0
        for segment in sorted(active):
            for block in _read_residuals(directory / str(segment)):
                total_h = _ordered_add(total_h, block[:, 0])
                total_v = _ordered_add(total_v, np.abs(block[:, 1]))
        count = coverage["assigned_positive"]
        with np.errstate(all="ignore"):
            mean_h, mean_v = float(np.divide(total_h, count)), float(np.divide(total_v, count))
        variance_h, variance_v = 0.0, 0.0
        for segment in sorted(active):
            for block in _read_residuals(directory / str(segment)):
                variance_h = _ordered_add(variance_h, (mean_h-block[:, 0])*(mean_h-block[:, 0]))
                # Intentionally SIGNED residual around the ABSOLUTE vertical mean.
                variance_v = _ordered_add(variance_v, (mean_v-block[:, 1])*(mean_v-block[:, 1]))
        with np.errstate(all="ignore"):
            metrics = dict(
                source_precision_percent=float(np.divide(counts["nTP"], counts["nTP"]+counts["nFP"])*100),
                source_recall_percent=float(np.divide(counts["nTP"], counts["nTP"]+counts["nFN"])*100),
                mean_horizontal=mean_h, mean_absolute_vertical=mean_v,
                std_horizontal=float(np.sqrt(np.divide(variance_h, count-1))),
                std_vertical_source=float(np.sqrt(np.divide(variance_v, count-1))),
            )
    diagnostics = []
    if coverage["unassigned_positive"]:
        diagnostics.append("unassigned_positive_points_excluded_from_source_metrics")
    if any(not np.isfinite(value) for value in metrics.values()):
        diagnostics.append("source_nonfinite_statistics")
    if any(np.array_equal(line.begin[:2], line.end[:2]) for line in lines):
        diagnostics.append("zero_length_xy_segment_retains_source_geometry")
    points = np.concatenate(collected) if collected else np.empty(0, dtype=POINT_DTYPE) if collect_points else None
    return EvaluationResult("nonfinite" if "source_nonfinite_statistics" in diagnostics else "ok",
                            counts, coverage, metrics, {"left": len(left), "right": len(right)},
                            diagnostics, points)


def evaluate_rails(reference: ReferencePoints, left: RailSegments, right: RailSegments, **kwargs):
    """Evaluate one fresh in-memory case with the same kernel used by LAS batches."""
    return evaluate_chunks((reference,), left, right, **kwargs)
