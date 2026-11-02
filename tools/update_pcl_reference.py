"""Freeze the small PCL numerical subset reached by the two LAS reader helpers.

Reads the locally bundled installations; never modifies them. Frozen C++ bodies
and hashes ship with Python so source auditing does not require large installers.
The native comparison harness separately requires the bundled Eigen headers.
"""

import json
from pathlib import Path

from railway_mls.coverage import source_text_hash


ROOT = Path(__file__).resolve().parents[1]
PCL = ROOT / "ADE code/Installations/Installations/PCL 1.8.0/include/pcl-1.8/pcl"
EIGEN = ROOT / "ADE code/Installations/Installations/PCL 1.8.0/3rdParty/Eigen/eigen3/Eigen"


def block(text, signature):
    start = text.index(signature)
    opening = text.index("{", start)
    depth = 0
    for pos in range(opening, len(text)):
        depth += (text[pos] == "{") - (text[pos] == "}")
        if depth == 0:
            return text[start : pos + 1]
    raise ValueError(signature)


items = (
    (
        "voxel_apply",
        "filters/impl/voxel_grid.hpp",
        "pcl::VoxelGrid<PointT>::applyFilter (PointCloud &output)",
    ),
    ("voxel_index", "filters/impl/voxel_grid.hpp", "struct cloud_point_index_idx"),
    ("voxel_constructor", "filters/voxel_grid.h", "VoxelGrid () :"),
    ("voxel_leaf", "filters/voxel_grid.h", "setLeafSize (float lx, float ly, float lz)"),
    (
        "bounds",
        "common/impl/common.hpp",
        "pcl::getMinMax3D (const pcl::PointCloud<PointT> &cloud, const std::vector<int> &indices,",
    ),
    (
        "centroid",
        "common/impl/centroid.hpp",
        "pcl::compute3DCentroid (const pcl::PointCloud<PointT> &cloud, \n",
    ),
    (
        "transform",
        "common/impl/transforms.hpp",
        "pcl::transformPointCloud (const pcl::PointCloud<PointT> &cloud_in, \n",
    ),
    ("accumulator_xyz", "common/impl/accumulators.hpp", "struct AccumulatorXYZ"),
    ("add_point", "common/impl/accumulators.hpp", "struct AddPoint"),
    ("get_point", "common/impl/accumulators.hpp", "struct GetPoint"),
    ("centroid_point", "common/centroid.h", "class CentroidPoint"),
    ("filter", "filters/filter.h", "filter (PointCloud &output)"),
    ("init_compute", "impl/pcl_base.hpp", "pcl::PCLBase<PointT>::initCompute ()"),
    ("deinit_compute", "impl/pcl_base.hpp", "pcl::PCLBase<PointT>::deinitCompute ()"),
)
extracts, sources, licenses = {}, {}, []
for name, relative, signature in items:
    path = PCL / relative
    text = path.read_text()
    extracts[name] = {
        "source": path.relative_to(ROOT).as_posix(),
        "signature": signature,
        "code": block(text, signature),
    }
    sources[path] = text
    license = text[: text.index("*/") + 2]
    if license not in licenses:
        licenses.append(license)
for relative in (
    "point_cloud.h",
    "PCLHeader.h",
    "point_types.h",
    "impl/point_types.hpp",
    "common/point_tests.h",
):
    path = PCL / relative
    sources[path] = path.read_text()
for relative in (
    "Core",
    "Geometry",
    "src/Core/Functors.h",
    "src/Core/SelfCwiseBinaryOp.h",
    "src/Core/arch/SSE/PacketMath.h",
):
    path = EIGEN / relative
    sources[path] = path.read_text()
payload = {
    "schema_version": 1,
    "scope": "PCL 1.8.0 PointXYZ wrappers; default fixed filter settings; Eigen float/double expressions",
    "sources": [
        {"path": p.relative_to(ROOT).as_posix(), "sha256": source_text_hash(t)}
        for p, t in sorted(sources.items())
    ],
    "extracts": extracts,
}
destination = ROOT / "src/railway_mls/pcl_reference.json"
destination.write_text(json.dumps(payload, indent=2) + "\n")
(destination.parent / "PCL_LICENSE.txt").write_text(
    "PCL notices retained for the numerical subset ported in pcl_helpers.py.\n"
    "Original notices from the bundled source files follow.\n\n" + "\n\n".join(licenses) + "\n"
)
print(f"Froze {len(extracts)} PCL extracts and {len(sources)} dependency hashes.")
