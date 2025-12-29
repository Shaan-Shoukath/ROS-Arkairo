# Mission state message
# Complete state of current mission

std_msgs/Header header

# Mission identification
uint32 mission_id
uint32 target_id

# State machine
uint8 STATE_IDLE = 0
uint8 STATE_NAVIGATING = 1
uint8 STATE_ARRIVED = 2
uint8 STATE_DETECTING = 3
uint8 STATE_CENTERING = 4
uint8 STATE_SPRAY_READY = 5
uint8 STATE_SPRAYING = 6
uint8 STATE_COMPLETE = 7
uint8 STATE_FAILED = 8
uint8 STATE_RETURNING = 9
uint8 current_state

# Progress
float32 distance_to_target_m
float32 centering_error_pixels
float32 spray_progress_percent

# Timing
builtin_interfaces/Duration elapsed_time
builtin_interfaces/Duration estimated_remaining

# Error tracking
string last_error
uint8 retry_count
