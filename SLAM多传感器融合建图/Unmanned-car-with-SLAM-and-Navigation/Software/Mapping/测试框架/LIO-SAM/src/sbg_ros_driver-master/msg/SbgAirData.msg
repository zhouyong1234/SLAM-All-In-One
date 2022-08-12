# SBG Ellipse Messages
Header header

# Time since sensor is powered up micro s
uint32 time_stamp

# Airdata sensor status
SbgAirDataStatus status

# Raw absolute pressure measured by the barometer sensor in Pascals.
float64 pressure_abs

# Altitude computed from barometric altimeter in meters and positive upward.
float64 altitude

# Raw differential pressure measured by the pitot tube in Pascal.
float64 pressure_diff

# True airspeed measured by a pitot tube in m.s^-1 and positive forward.
float64 true_air_speed

# Outside air temperature in °C that could be used to compute true airspeed from differential pressure.
float64 air_temperature