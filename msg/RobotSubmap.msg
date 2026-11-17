# header frame is the robot's LIO reference; stamp is the submap keyframe time.
std_msgs/Header header
string session_id
uint8 robot_id
uint32 epoch
uint32 submap_id
# T_reference_key: maps keyframe-local points into header.frame_id.
geometry_msgs/Pose pose_in_lio_map
# R_gravity_key: maps cloud_local into a gravity-aligned frame at the same origin.
geometry_msgs/Quaternion gravity_alignment
# Points are in a unique keyframe-local frame, NOT header.frame_id.
sensor_msgs/PointCloud2 cloud_local
bool partial
string pcd_path
