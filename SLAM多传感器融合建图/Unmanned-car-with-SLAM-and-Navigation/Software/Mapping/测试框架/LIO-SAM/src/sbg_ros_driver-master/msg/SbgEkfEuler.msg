# SBG Ellipse Messages

Header header

# Time since sensor is powered up micro [us]
uint32 time_stamp

# Angle [Roll, Pitch, Yaw (heading)] [rad]
#
# The right-hand convention applies, where positive rotation is indicated by the direction in which
# the fingers on your right hand curl when your thumb is pointing in the direction of the axis of rotation.
#
# NED convention:
#   Roll: Rotation about the X axis
#   Pitch: Rotation about the Y axis
#   Yaw: Rotation about the down axis. Zero when the X axis is pointing North.
#
# ENU convention:
#   Roll: Rotation around X axis
#   Pitch: Rotation around Y axis (opposite sign compared to NED)
#   Yaw: Rotation about the up axis. Zero when the X axis is pointing East. (opposite sign compared to NED)
geometry_msgs/Vector3 angle

# Angle accuracy (Roll, Pitch, Yaw (heading)) (1 sigma) [rad]
geometry_msgs/Vector3 accuracy

#  Global solution status
SbgEkfStatus status
