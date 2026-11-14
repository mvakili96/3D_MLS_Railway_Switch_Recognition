"""Reference-preserving input adapters; deliberately separate from tracking I/O."""

import csv
from hashlib import sha256
from pathlib import Path

import numpy as np

from .types import RailSegments, ReferencePoints


def file_fingerprint(path):
    path = Path(path)
    digest = sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(4*1024*1024), b""):
            digest.update(block)
    return {"path": str(path), "bytes": path.stat().st_size, "sha256": digest.hexdigest()}


def read_rail_segments(path):
    """Count-prefixed six-field endpoint format used by both native menu loaders.

    Missing/malformed/nonfinite fields are rejected instead of emulating unsafe
    strtok(NULL)/uninitialized reads. Finite canonical files retain every segment.
    """
    path = Path(path)
    with path.open(encoding="utf-8-sig") as stream:
        first = stream.readline().strip()
        try:
            count = int(first)
        except ValueError as error:
            raise ValueError(f"{path}: invalid segment count") from error
        if count < 0:
            raise ValueError(f"{path}: negative segment count")
        segments = []
        for index in range(count):
            fields = stream.readline().strip().split(",")
            if len(fields) != 6:
                raise ValueError(f"{path}: segment {index} requires six coordinates")
            try:
                segments.append([float(value) for value in fields])
            except ValueError as error:
                raise ValueError(f"{path}: invalid segment {index}") from error
        if stream.read().strip():
            raise ValueError(f"{path}: content beyond declared segment count")
    return RailSegments(np.asarray(segments).reshape(count, 2, 3),
                        {"format": "native_counted_segments", **file_fingerprint(path)})


def iter_reference_las(path, chunk_size=250_000):
    """All legacy LAS records, original byte labels and float64 world coordinates."""
    if chunk_size <= 0:
        raise ValueError("chunk_size must be positive")
    try:
        import laspy
    except ImportError as error:
        raise ImportError("Reference LAS loading requires: pip install -e '.[las]'") from error
    path = Path(path)
    with laspy.open(path) as reader:
        header = reader.header
        if header.point_format.id > 5 or header.version.minor > 3:
            raise ValueError("source2018 supports legacy LAS layouts through 1.3, formats 0–5")
        if not header.are_points_compressed:
            expected = header.offset_to_point_data + header.point_count*header.point_format.size
            if path.stat().st_size < expected:
                raise ValueError(f"{path}: truncated LAS point payload")
        loaded = 0
        for points in reader.chunk_iterator(chunk_size):
            xyz = np.column_stack((np.asarray(points.X, dtype=np.float64)*header.scales[0]+header.offsets[0],
                                   np.asarray(points.Y, dtype=np.float64)*header.scales[1]+header.offsets[1],
                                   np.asarray(points.Z, dtype=np.float64)*header.scales[2]+header.offsets[2]))
            # laspy.classification masks legacy flag bits; the C++ byte read does not.
            yield ReferencePoints(xyz, np.asarray(points.raw_classification),
                                  np.arange(loaded, loaded+len(points), dtype=np.int64))
            loaded += len(points)
        if loaded != header.point_count:
            raise ValueError(f"{path}: expected {header.point_count} records, read {loaded}")


def rails_from_layer(layer):
    """Adapt one explicit track layer; do not connect separate layers or branches."""
    result = []
    for name in ("rail1_vector", "rail2_vector"):
        vector = getattr(layer, name)
        if any(point is None for point in vector):
            raise ValueError(f"{name}: missing vertex")
        xyz = np.array([point.as_array() for point in vector], dtype=np.float64).reshape(-1, 3)
        endpoints = np.stack((xyz[:-1], xyz[1:]), axis=1) if len(xyz) > 1 else np.empty((0, 2, 3))
        result.append(RailSegments(endpoints, {"format": "python_track_layer", "vector": name}))
    return tuple(result)


def read_python_vectors(path, *, layer):
    """Read one explicit layer from pipeline vectors.csv; IDs must be contiguous."""
    groups = {"rail1": {}, "rail2": {}}
    found_layer = False
    with Path(path).open(newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        required = {"layer", "vector", "vertex", "x", "y", "z"}
        if not required <= set(reader.fieldnames or ()):
            raise ValueError("Python vectors require the pipeline vectors.csv schema")
        for row in reader:
            if int(row["layer"]) != layer:
                continue
            found_layer = True
            if row["vector"] not in groups:
                continue
            vertices = groups[row["vector"]]
            index = int(row["vertex"])
            if index < 0 or index in vertices:
                raise ValueError("duplicate/negative vertex ID in selected layer")
            vertices[index] = [float(row[key]) for key in ("x", "y", "z")]
    if not found_layer:
        raise ValueError(f"Python vectors have no layer {layer}")
    result = []
    fingerprint = file_fingerprint(path)
    for name, vertices in groups.items():
        if sorted(vertices) != list(range(len(vertices))):
            raise ValueError(f"{name}: vertex IDs must be contiguous from zero")
        xyz = np.array([vertices[i] for i in range(len(vertices))], dtype=np.float64).reshape(-1, 3)
        endpoints = np.stack((xyz[:-1], xyz[1:]), axis=1) if len(xyz) > 1 else np.empty((0, 2, 3))
        result.append(RailSegments(endpoints, {"format": "python_pipeline_vectors", "layer": layer,
                                               "vector": name, **fingerprint}))
    return tuple(result)
