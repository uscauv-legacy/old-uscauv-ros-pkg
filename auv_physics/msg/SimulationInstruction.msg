# Data used to begin or control the high-level state of the physics simulator

uint8 START=0
uint8 STOP=1
uint8 RESTART=2

Header header # Parent frame and time stamp must be set. Simulation is relative to time in header

int32 type

geometry_msgs/Pose initial_pose
geometry_msgs/Twist initial_velocity