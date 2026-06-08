std_msgs/Header header

string native_position_type   # Receiver-native position type, e.g. NARROW_INT / L1_FLOAT / SINGLE.
string native_velocity_type   # Receiver-native velocity type, e.g. DOPPLER_VELOCITY.
string velocity_solution_status
float64 velocity_latency_s
float64 velocity_age_s

string latest_raw_gga
string latest_raw_rmc
string latest_raw_gsa
string latest_raw_gsv
string latest_raw_gst
string latest_raw_bestnav
string latest_raw_pvtsln
string latest_raw_rtkstatus
string latest_raw_rtcmstatus
string latest_rtcm_frame_hex

string[] recent_raw_lines
string last_error_line
string last_unknown_line

string[] sentence_rate_names
float64[] sentence_rate_hz

uint32 satellites_tracked
uint32 bestnav_satellites_used
uint32 pvtsln_satellites_used
uint32 gsv_satellites_in_view
uint32 gsv_max_snr_db
