# Cloud Info
Header header 

int32[] startRingIndex # 对应线在点云（cloud_deskewed）中的起始索引
int32[] endRingIndex   # 对应线在点云（cloud_deskewed）中的结束索引

int32[]  pointColInd # point column index in range image
float32[] pointRange # point range 

int64 imuAvailable   # 是否包含 IMU 校正信息
int64 odomAvailable  # 是否包含 odom 校正信息

# Attitude for LOAM initialization -- 从 IMU 消息中获得的测量姿态
float32 imuRollInit 
float32 imuPitchInit
float32 imuYawInit

# Initial guess from imu pre-integration -- 从 imu 里程计中获得状态作为初值
float32 initialGuessX
float32 initialGuessY
float32 initialGuessZ
float32 initialGuessRoll
float32 initialGuessPitch
float32 initialGuessYaw

# Point cloud messages
sensor_msgs/PointCloud2 cloud_deskewed  # original cloud deskewed
sensor_msgs/PointCloud2 cloud_corner    # extracted corner feature
sensor_msgs/PointCloud2 cloud_surface   # extracted surface feature