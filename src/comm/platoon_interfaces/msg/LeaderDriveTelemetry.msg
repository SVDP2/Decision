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

bool rc_valid
bool rc_failsafe
bool auto_valid
bool auto_failsafe
bool stop_required

uint16 rc_throttle_us
uint16 rc_steer_us
uint16 rc_mode_us

float32 throttle_norm
float32 throttle_cmd_pwm
float32 throttle_output_pwm
float32 steering_target_adc
float32 steering_current_adc
float32 steering_angle_rad
float32 speed_estimate_mps
float32 auto_speed_cmd_mps
float32 auto_steering_cmd_rad

uint32 arduino_time_ms
uint32 bridge_rx_ok_count
uint32 bridge_crc_fail_count
