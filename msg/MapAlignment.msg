# Maps UAV reference-map points into the UGV reference map; no TF is broadcast.
std_msgs/Header header
bool valid
uint64 revision
string reference_frame
string query_frame
string reference_session_id
string query_session_id
uint32 reference_epoch
uint32 query_epoch
geometry_msgs/Transform map_transform
uint32 reference_submap_id
uint32 query_submap_id
string reason
