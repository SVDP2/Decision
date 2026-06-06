std_msgs/Header header

uint8 ROLE_UNKNOWN=0
uint8 ROLE_LEADER=1
uint8 ROLE_FOLLOWER=2

uint8 role
uint32 seq
uint32 boot_id
uint32 last_motion_seq
bool system_ok
string status
