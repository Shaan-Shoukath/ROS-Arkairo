# Mission command message for controlling spray operations
# Sent from GCS operator interface

std_msgs/Header header

# Command type
uint8 CMD_START = 0
uint8 CMD_PAUSE = 1
uint8 CMD_RESUME = 2
uint8 CMD_ABORT = 3
uint8 CMD_RTL = 4
uint8 command

# Target (if applicable)
uint32 target_id
geographic_msgs/GeoPoint target_position

# Parameters
float32 spray_duration_sec
float32 approach_altitude_m
