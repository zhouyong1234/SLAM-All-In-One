// param.tpp
// parameters in the package
// Zhiang Chen, Aug 2020, zch@asu.edu

// GPSVIO
#define Queue_Size 10  // the queue size for gps and vio message synchronization
#define RATE 30  // odom output rate
#define CALIB_NM 10 // the number of gps odom needed for tracking camera calibration

// SWGraph
#define MAX_NODE 400 // maximum node number in sliding window

// GPS Covariance
#define GPS_COV false // GPSVIO will only use the covariance defined here if true
#define GPS_x 1
#define GPS_y 1
#define GPS_z 1
#define GPS_roll 100
#define GPS_pitch 100
#define GPS_yaw 100

// VIO Covariance
#define VIO_COV false // GPSVIO will only use the covariance defined here if true
#define VIO_x 1
#define VIO_y 1
#define VIO_z 1
#define VIO_roll 0.5
#define VIO_pitch 0.5
#define VIO_yaw 0.5
