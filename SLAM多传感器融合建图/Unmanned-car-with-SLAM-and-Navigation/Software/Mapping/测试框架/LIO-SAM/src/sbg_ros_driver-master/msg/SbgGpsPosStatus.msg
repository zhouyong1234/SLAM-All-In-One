# SBG Ellipse Messages
# Submessage


# The raw GPS position status
# 0 SOL_COMPUTED		A valid solution has been computed.
# 1 INSUFFICIENT_OBS 	Not enough valid SV to compute a solution.
# 2 INTERNAL_ERROR 		An internal error has occurred.
# 3 HEIGHT_LIMIT 		The height limit has been exceeded.
uint8 status

# The raw GPS position type
# 0 NO_SOLUTION		No valid solution available.
# 1 UNKNOWN_TYPE	An unknown solution type has been computed.
# 2 SINGLE			Single point solution position.
# 3 PSRDIFF			Standard Pseudorange Differential Solution (DGPS).
# 4 SBAS			SBAS satellite used for differential corrections.
# 5 OMNISTAR		Omnistar VBS Position (L1 sub-meter).
# 6 RTK_FLOAT		Floating RTK ambiguity solution (20 cms RTK).
# 7 RTK_INT			Integer RTK ambiguity solution (2 cms RTK).
# 8 PPP_FLOAT		Precise Point Positioning with float ambiguities
# 9 PPP_INT			Precise Point Positioning with fixed ambiguities
# 10 FIXED			Fixed location solution position
uint8 type

# True if GPS L1 is used in the solution
bool gps_l1_used

# True if GPS L2 is used in the solution
bool gps_l2_used

# True if GPS L5 is used in the solution
bool gps_l5_used

# True if GLONASS L1 is used in the solution
bool glo_l1_used

# True if GLONASS L2 is used in the solution
bool glo_l2_used
