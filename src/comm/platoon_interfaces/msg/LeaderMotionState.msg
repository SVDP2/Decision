std_msgs/Header header

uint8 MODE_UNKNOWN=0
uint8 MODE_MANUAL_RC=1
uint8 MODE_AUTONOMOUS=2
uint8 MODE_ESTOP=3

uint8 SOURCE_UNKNOWN=0
uint8 SOURCE_RC=1
uint8 SOURCE_AUTONOMOUS_CMD=2
uint8 SOURCE_OUTPUT_ESTIMATE=3
uint8 SOURCE_SIMULATED=4

uint32 seq
uint32 boot_id
uint8 drive_mode
uint8 source

bool motion_valid
bool stop_required

float32 speed_mps
float32 acceleration_mps2
float32 steering_angle_rad
float32 curvature_1pm
float32 yaw_rate_radps
float32 throttle_norm

uint32 source_time_ms
