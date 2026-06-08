std_msgs/Header header

string health_level      # OK / WARN / ERROR
string health_message    # Human-readable one-line summary.
string fix_mode          # RTK_FIXED / RTK_FLOAT / DGNSS / GNSS / NO_GNSS / UNKNOWN
string solution_status   # SOL_COMPUTED / NO_FIX / receiver-native solution status.

string position_source   # BESTNAV / GGA / UNKNOWN
float64 latitude
float64 longitude
float64 altitude_msl_m
float64 altitude_ellipsoid_m
float64 differential_age_s

string accuracy_source   # BESTNAV / PVTSLN / GST / UNKNOWN
float64 accuracy_age_s
float64 horizontal_accuracy_m
float64 vertical_accuracy_m
float64 hdop

uint32 satellites_visible
uint32 satellites_used

float64 speed_mps
float64 course_deg
float64 east_velocity_mps
float64 north_velocity_mps
float64 up_velocity_mps

string base_station_name
string ntrip_host
string ntrip_mountpoint
bool ntrip_connected
string ntrip_last_error
float64 ntrip_gga_age_s
float64 rtcm_hz
float64 rtcm_bytes_per_sec
float64 rtcm_age_s
uint32 rtcm_base_station_id
uint32 rtcm_last_message_id
uint32[] rtcm_message_ids
uint32[] rtcm_message_counts

string serial_port
uint32 serial_baud
string configured_profile
float64 solution_rate_hz
float64 publish_period_s

uint32 checksum_error_count
uint32 parse_drop_count
uint64 unknown_line_count
uint64 serial_line_count
string last_error
string last_unknown_sentence_type
