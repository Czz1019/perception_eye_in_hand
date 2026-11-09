# Native module provenance — sparse interface v2

The executable derives from NVIDIA `nvblox::NvbloxNode`, linking the installed
`nvblox_ros` library from the user-selected prefix. It reuses NITROS conversion,
image-time TF, MultiMapper, TSDF/3D ESDF propagation and official layer streaming.

The v1 application exporter reused the dense AABB conversion template. V2 removes
that conversion from real-time pointcloud publication. `block_export.cu` traverses
`EsdfLayer::getAllBlockIndices()` and GPU block pointers, rejects `!observed`, and
compacts only requested outputs. The occupied predicate follows the definition
of `EsdfVoxelFilter` in NVIDIA `layer_publishing.cpp` (`observed && is_inside`).
In this reference revision that filter is defined but not wired to its own 3D
publisher; the reference's active layer outputs are Mesh/TSDF/color/etc., and its
static_esdf_pointcloud is emitted by the 2D slicing branch. V2 does not pretend
to copy a nonexistent default 3D occupied topic.

Signed conversion retains NVIDIA's observed, is_inside,
sqrt(squared_distance_vox)*voxel_size semantics. The dense AABB conversion is
still inherited for on-demand ESDF service queries. No TSDF or ESDF propagation
algorithm is reimplemented.

Mesh retains the official incremental `nvblox_msgs/Mesh` publisher. The standard
MarkerArray view expands indexed triangles and handles empty blocks/deletions;
join/reset snapshots use NVIDIA `layer_streamers().serializeAllBlocks` and
`conversions::markerMessageFromSerializedMesh`. Native vertex coordinates are
already global and must not be offset by the block origin again.

Reference checkout: /home/czz/workspaces/isaac_ros-dev/src/isaac_ros_nvblox
HEAD used in previous implementation: bc181942202b24fd47b2f3db8f77c3c82d484566
(local YAML modifications exist). The dependency retains its original
Apache-2.0 license and NVIDIA notices. Project additions carry SPDX identifiers.

`camera_path` uses robot FK/hand-eye TF at depth-image
timestamps. NVIDIA visual_slam's optimized SLAM path is a different source;
this module does not reproduce or claim visual odometry/loop closure.

V4 delegates the main processing order directly to NvbloxNode::tick, rather than
maintaining an application token budget. MeshView follows the reference's
separation of layer publication from the input adapter; its snapshots serialize
the already updated mesh without forcing a full-map remesh. GpuTimings is an
application measurement pool; it does not alter NVIDIA's integration kernels.
