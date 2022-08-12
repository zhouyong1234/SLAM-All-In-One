# SBG Ellipse Messages

Header header

# Time since sensor is powered up [us]
uint32 time_stamp

# Quaternion parameter (ROS order X, Y, Z, W)
# The rotation definition depends on the driver NED/ENU configuration
# Please read the message SbgEkfEuler for more information
geometry_msgs/Quaternion quaternion

# Angle accuracy (Roll, Pitch, Yaw (heading)) (1 sigma) [rad]
geometry_msgs/Vector3 accuracy

#  Global solution status
SbgEkfStatus status
