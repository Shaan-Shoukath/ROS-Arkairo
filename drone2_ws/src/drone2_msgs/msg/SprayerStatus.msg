# Sprayer status message
# Reports current state of spray system

std_msgs/Header header

# Sprayer state
bool is_active
bool tank_empty
float32 tank_level_percent      # 0-100

# PWM output
uint16 current_pwm
uint16 target_pwm

# Flow monitoring
float32 flow_rate_ml_per_sec
float32 total_sprayed_ml

# Safety
bool safety_enabled
bool in_safe_zone
