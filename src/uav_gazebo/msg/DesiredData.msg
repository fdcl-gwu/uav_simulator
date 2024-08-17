std_msgs/Header header

float64[3] position
float64[3] velocity
float64[3] acceleration
float64[3] jerk
float64[3] snap

float64[9] attitude
float64[3] angular_velocity
float64[3] angular_acceleration
float64[3] angular_jerk

float64[3] b1c

float64[3] b1
float64[3] b1_dot
float64[3] b1_2dot

float64[3] b3
float64[3] b3_dot
float64[3] b3_2dot

# Decoupled-yaw controller
float64 wc3
float64 wc3_dot