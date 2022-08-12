# SBG Ellipse Messages
Header header

# Time since sensor is powered up (us)
uint32 time_stamp

# General UTC time and clock sync status
SbgUtcTimeStatus clock_status

# Year
uint16 year

# Month in Year [1 ... 12]
uint8 month

# Day in Month [1 ... 31]
uint8 day

# Hour in day [0 ... 23]
uint8 hour

# Minute in hour [0 ... 59]
uint8 min

# Second in minute [0 ... 60], Note 60 is when a leap second is added.
uint8 sec

# Nanosecond of second.
uint32 nanosec

# GPS Time of week (ms)
uint32 gps_tow
