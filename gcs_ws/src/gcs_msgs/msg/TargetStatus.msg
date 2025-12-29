# Target status message for tracking spray mission progress
# Sent from GCS to monitoring systems

std_msgs/Header header

# Target identification
uint32 target_id
geographic_msgs/GeoPoint position

# Status
uint8 STATUS_PENDING = 0
uint8 STATUS_DISPATCHED = 1
uint8 STATUS_IN_PROGRESS = 2
uint8 STATUS_COMPLETED = 3
uint8 STATUS_FAILED = 4
uint8 STATUS_CANCELLED = 5
uint8 status

# Drone assignment
string assigned_drone

# Timestamps
builtin_interfaces/Time received_time
builtin_interfaces/Time dispatched_time
builtin_interfaces/Time completed_time
