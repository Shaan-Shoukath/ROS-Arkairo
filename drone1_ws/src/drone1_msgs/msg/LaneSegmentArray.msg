# LaneSegmentArray message for complete mission plan
# Contains all survey lanes for a mission

# Header with timestamp
std_msgs/Header header

# Mission identification
string mission_id
string kml_source_file

# Array of lane segments
drone1_msgs/LaneSegment[] lanes

# Mission metadata
uint32 total_lanes
float64 total_distance_m     # Estimated total flight distance
float64 estimated_duration_s # Estimated mission duration
