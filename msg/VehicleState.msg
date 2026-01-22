std_msgs/Header header

# Vehicle Position
float64 x
float64 y
float64 yaw

# Vehicle Dynamics
float64 velocity
float64 steering_angle

# Transmission State
uint8 GEAR_P = 0
uint8 GEAR_R = 1
uint8 GEAR_N = 2
uint8 GEAR_D = 3

# Current Gear
uint8 gear

