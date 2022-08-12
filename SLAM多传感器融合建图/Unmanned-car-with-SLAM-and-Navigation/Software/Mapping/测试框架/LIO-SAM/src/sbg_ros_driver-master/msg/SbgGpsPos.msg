# SBG Ellipse Messages
Header header

#  Time since sensor is powered up us 
uint32 time_stamp

# GPS position fix and status bitmask
SbgGpsPosStatus status

# GPS Time of Week ms
uint32 gps_tow

# Latitude [degrees]; Positive is north of equator; negative is south
float64 latitude

# Longitude [degrees]; Positive is east of prime meridian; negative is west
float64 longitude

# Altitude [m]; Positive is above Mean Sea Level in meters
float64 altitude

# Altitude difference between the geoid and the Ellipsoid (WGS-84 Altitude - MSL Altitude)
# (Height above Ellipsoid = altitude + undulation)
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

# Number of space vehicles used in GNSS solution
uint8 num_sv_used

# ID of the DGPS/RTK base station in use
uint16 base_station_id

# Differential data age 0.01 s
uint16 diff_age
