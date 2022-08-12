# SBG Ellipse Messages
# Submessage

# The raw GPS velocity status
# 0 SOL_COMPUTED		A valid solution has been computed.
# 1 INSUFFICIENT_OBS	Not enough valid SV to compute a solution.
# 2 INTERNAL_ERROR		An internal error has occurred.
# 3 LIMIT				Velocity limit exceeded.
uint8 vel_status

# The raw GPS velocity type
# 0 VEL_NO_SOLUTION		No valid velocity solution available.
# 1 VEL_UNKNOWN_TYPE	An unknown solution type has been computed.
# 2 VEL_DOPPLER			A Doppler velocity has been computed.
# 3 VEL_DIFFERENTIAL	A velocity has been computed between two positions.
uint8 vel_type