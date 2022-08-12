# SBG Ellipse Messages

# True when a clock input can be used to synchronize the internal clock.
bool clock_stable

# Define the internal clock estimation status
# 0 An error has occurred on the clock estimation.
# 1 The clock is only based on the internal crystal.
# 2 A PPS has been detected and the clock is converging to it.
# 3 The clock has converged to the PPS and is within 500ns.
uint8 clock_status

# True if UTC time is synchronized with a PPS
bool clock_utc_sync

# UTC validity status
# 0 The UTC time is not known, we are just propagating the UTC time internally.
# 1 We have received valid UTC time information but we don't have the leap seconds information.
# 2 We have received valid UTC time data with valid leap seconds.
uint8 clock_utc_status