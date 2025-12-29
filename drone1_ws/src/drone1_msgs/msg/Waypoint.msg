# Waypoint message for survey mission planning
# Represents a single GPS coordinate for navigation

# Header with timestamp
std_msgs/Header header

# GPS coordinates
float64 latitude   # Degrees
float64 longitude  # Degrees
float64 altitude   # Meters (AMSL or relative, defined by altitude_mode)

# Altitude reference mode
uint8 ALTITUDE_AMSL = 0
uint8 ALTITUDE_RELATIVE = 1
uint8 altitude_mode 0

# Optional waypoint identifier
uint32 waypoint_id
