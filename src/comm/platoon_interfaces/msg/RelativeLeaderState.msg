std_msgs/Header header

uint8 SOURCE_UNKNOWN=0
uint8 SOURCE_V2V_GPS=1
uint8 SOURCE_SYNTHETIC=2
uint8 SOURCE_V2V_ARUCO=3
uint8 SOURCE_LIDAR_WHEELS=4
uint8 SOURCE_FUSED=5

uint8 source
string target_frame

bool valid
bool pose_valid
bool motion_valid
bool stop_required

geometry_msgs/PoseWithCovariance relative_pose

float32 longitudinal_distance_m
float32 lateral_offset_m
float32 range_m

float32 leader_speed_mps
float32 leader_acceleration_mps2
float32 leader_curvature_1pm
float32 leader_yaw_rate_radps

float32 reference_age_sec

string status
