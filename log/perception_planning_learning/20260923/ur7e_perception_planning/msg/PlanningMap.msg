# One atomic snapshot. All distances in metres, positive in free space.
# Arrays use C order [z,y,x]; dimensions are [nx,ny,nz]. Unknown is NEVER free.
std_msgs/Header header
uint64 epoch
uint64 sequence
geometry_msgs/Point origin
float32 resolution
uint32[3] dimensions
float32 unknown_distance
float32[] nvblox_distance
# Conservative distance to either an obstacle OR unobserved space.
float32[] planning_distance
# 0 unknown/occupied, 1 depth-observed free, 2 commissioned initial clearance.
uint8[] free_source
# If nonempty, the three arrays above are empty. Lossless zlib level-1 payload:
# n little-endian float32 native distances, n float32 planning distances, n uint8
# sources, where n=nx*ny*nz. Geometry/version/stamp remain explicit ROS fields.
uint8[] compressed_data
