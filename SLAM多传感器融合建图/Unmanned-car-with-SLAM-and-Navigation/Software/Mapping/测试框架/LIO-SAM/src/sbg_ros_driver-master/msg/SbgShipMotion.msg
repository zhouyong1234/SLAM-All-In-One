# SBG Ellipse Messages
Header header

# Time since sensor is powered up us 
uint32 time_stamp

# Main heave period in seconds. s float 4 4
uint16 heave_period

# [Surge, Sway, Heave (positive down)] at main location (in m)
# Note : Surge & Sway are not fulfilled
geometry_msgs/Vector3 ship_motion

# [Longitudinal, Lateral, Vertical (positive down)] acceleration (in m/s2)
geometry_msgs/Vector3 acceleration

# [Longitudinal, Lateral, Vertical (positive down)] velocity (in m/s)
geometry_msgs/Vector3 velocity

# Ship motion output status
SbgShipMotionStatus status
