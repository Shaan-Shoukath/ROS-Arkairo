# LaneSegment message for survey lane definition
# Represents a single survey lane with start and end waypoints

# Header with timestamp
std_msgs/Header header

# Lane identification
uint32 lane_id
string lane_name

# Start and end waypoints of this lane
drone1_msgs/Waypoint start_waypoint
drone1_msgs/Waypoint end_waypoint

# Lane parameters
float64 lane_width      # Meters - swath width for survey
float64 overlap_percent # Percentage overlap with adjacent lanes

# Lane status
uint8 STATUS_PENDING = 0
uint8 STATUS_IN_PROGRESS = 1
uint8 STATUS_COMPLETED = 2
uint8 STATUS_SKIPPED = 3
uint8 status 0
