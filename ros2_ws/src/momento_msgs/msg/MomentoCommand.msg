uint8 SOURCE_UNKNOWN=0
uint8 SOURCE_XBOX=1
uint8 SOURCE_AUTO=2
uint8 SOURCE_ASSIST=3
uint8 SOURCE_ESTOP=4

uint8 YAW_ABSOLUTE=0
uint8 YAW_RATE=1

std_msgs/Header header
float32 linear
float32 yaw
float32 roll
float32 pitch
float32 leg
bool start
uint8 yaw_mode
uint8 source
