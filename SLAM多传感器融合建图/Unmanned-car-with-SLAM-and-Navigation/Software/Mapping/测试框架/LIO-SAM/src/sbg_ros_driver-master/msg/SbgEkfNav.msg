# SBG Ellipse Messages

Header header

# Time since sensor is powered up [us]
uint32 time_stamp

# Velocity [m/s]
# In NED convention:
#   x: North
#   y: East
#   z: Down
# In ENU convention:
#   x: East
#   y: North
#   z: Up
geometry_msgs/Vector3 velocity

# Velocity accuracy (1 sigma) [m/s].
# In NED convention:
#   x: North
#   y: East
#   z: Vertical
# In ENU convention:
#   x: East
#   y: North
#   z: Vertical
geometry_msgs/Vector3 velocity_accuracy

# Latitude [degrees]. Positive is north of equator; negative is south
float64 latitude

# Longitude [degrees]. Positive is east of prime meridian; negative is west
float64 longitude

# Altitude [m]. Positive (above Mean Sea Level in meters)
float64 altitude

# Altitude difference between the geoid and the Ellipsoid (WGS-84 Altitude - MSL Altitude)
# (Height above Ellipsoid = altitude + undulation) [m]
float32 undulation

# Position accuracy (1 sigma) [m].
# In NED convention:
#   x: North
#   y: East
#   z: Vertical
# In ENU convention:
#   x: East
#   y: North
#   z: Vertical
geometry_msgs/Vector3 position_accuracy

#  Global solution status
SbgEkfStatus status
