# SBG Ellipse Messages
Header header

# Time since sensor is powered up [us]
uint32 time_stamp

# IMU Status
SbgImuStatus imu_status

# Filtered Accelerometer [m/s^2]
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

# Filtered Gyroscope [rad/s]
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
geometry_msgs/Vector3 gyro

# Internal Temperature [°C]
float32 temp

# Sculling output [m/s2]
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
geometry_msgs/Vector3 delta_vel

# Coning output [rad/s]
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
geometry_msgs/Vector3 delta_angle
