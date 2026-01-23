std_msgs/Header header

# Vehicle Position
float64 x
float64 y
float64 yaw

# Vehicle Mode
uint8 MODE_MANUAL = 0
uint8 MODE_AUTO = 1
uint8 MODE_ESTOP = 2

# Transmission State
uint8 GEAR_P = 0
uint8 GEAR_R = 1
uint8 GEAR_N = 2
uint8 GEAR_D = 3

# Current state
uint8 gear
uint8 mode 
float32 battery

# Vehicle Dynamics
float64 velocity
float64 steering_angle


