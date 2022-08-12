# SBG Ellipse Messages
Header header

# Time since sensor is powered up us 
uint32 time_stamp

# True if we have received events at a higher rate than 1 kHz.
bool overflow

# True if at least two events have been received.
bool offset_0_valid

# True if at least three events have been received.
bool offset_1_valid

# True if at least four events have been received.
bool offset_2_valid

# True if five events have been received.
bool offset_3_valid


# Time offset for the second received event. (us)
uint16 time_offset_0

# Time offset for the third received event. (us)
uint16 time_offset_1

# Time offset for the fourth received event. (us)
uint16  time_offset_2

# Time offset for the fifth received event. (us)
uint16  time_offset_3
