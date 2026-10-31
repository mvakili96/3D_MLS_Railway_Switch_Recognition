"""Small source-comparison fixtures with distinct heights to avoid sort ties."""

from math import pi, tan

import numpy as np

from railway_mls.points import PointCloud
from railway_mls.sampling import MsvcRand


def parallel_cloud(counts=(100, 90, 80), angle=45):
    slope = tan(angle * pi / 180)
    rows = []
    for rail, count in enumerate(counts):
        for position in range(count):
            x = position * 0.01
            rows.append((x, slope * x + rail * 0.3, 0.2 + len(rows) * 1e-6))
    return PointCloud(rows)


def invalid_second_valid_third_cloud():
    rows = [(i * 0.01, i * 0.01, 0.3 + i * 1e-6) for i in range(180)]
    rows += [(i * 0.01, i * 0.01 + 0.3, 0.5 + i * 1e-6) for i in range(50)]
    rows += [(3 + i * 0.01, 3 + i * 0.01 + 0.3, 0.2 + i * 1e-6) for i in range(90)]
    return PointCloud(rows)


def draws(count, seed=1):
    random = MsvcRand(seed)
    return [random.rand() for _ in range(count)]


def component_cloud(counts):
    rows = [
        (i * 0.01, group * 10, i * 0.1) for group, count in enumerate(counts) for i in range(count)
    ]
    return PointCloud(rows, class_codes=np.full(len(rows), 9), segment_ids=np.full(len(rows), 7))
