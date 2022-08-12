# SBG Ellipse Messages
Header header

# Time since sensor is powered up (us)
uint32 time_stamp

# Magnetometer output
#
# NED convention:
#   x: X axis of the device frame
#   y: Y axis of the device frame
#   z: Z axis of the device frame
#
# ENU convention:
#   x: X axis of the device frame
#   y: -Y axis of the device frame
#   z: -Z axis of the device frame
geometry_msgs/Vector3 mag

# Accelerometer output  [m/s^2]
#
# NED convention:
#   x: X axis of the device frame
#   y: Y axis of the device frame
#   z: Z axis of the device frame
#
# ENU convention:
#   x: X axis of the device frame
#   y: -Y axis of the device frame
#   z: -Z axis of the device frame
geometry_msgs/Vector3 accel

# Status
SbgMagStatus status
