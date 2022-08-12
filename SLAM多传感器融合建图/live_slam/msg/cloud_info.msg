# Cloud Info
Header header 

int32[] startRingIndex
int32[] endRingIndex

int32[]  pointColInd     # point column index in range image
float32[] pointRange     # point range

int64 imuAvailable
int64 odomAvailable
int64 altitudeAvailable

# Attitude for LOAM initialization
float32 imuRollInit
float32 imuPitchInit
float32 imuYawInit

# Initial guess from imu pre-integration
float32 initialGuessX
float32 initialGuessY
float32 initialGuessZ
float32 initialGuessRoll
float32 initialGuessPitch
float32 initialGuessYaw

float64 altitude

# Point cloud messages
sensor_msgs/PointCloud2 cloud_deskewed         # original cloud deskewed
sensor_msgs/PointCloud2 cloud_lane_corner      # extracted lane cloud
sensor_msgs/PointCloud2 cloud_lane_surface     # extracted lane cloud
sensor_msgs/PointCloud2 cloud_corner           # extracted corner feature
sensor_msgs/PointCloud2 cloud_surface          # extracted surface feature