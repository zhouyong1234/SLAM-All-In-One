#include "params.h"

int MIN_DIST;
int MAX_CNT;
int WINDOW_SIZE;
double BASELINE;

double fx;
double cx;
double fy;
double cy;

double ShiftOnYTHS;
double MIN_PARALLAX;
int MIN_TRACKED_FEATURES;

Eigen::Matrix<double, 3, 4> Left_Cam_Pose;
Eigen::Matrix<double, 3, 4> Right_Cam_Pose;

void setParams()
{
	MIN_DIST = 30;
	BASELINE = 0.537165718864418;
	MAX_CNT = 200;
	WINDOW_SIZE = 10;

	fx =  7.18856e+02;
	cx = 6.071928e+02;
	fy = 7.18856e+02;
	cy = 1.852157e+02;

	Left_Cam_Pose<<1,0,0,0,
					0,1,0,0,
					0,0,1,0;

	Right_Cam_Pose<<1,0,0,-0.537165718864418,
					0,1,0,0,
					0,0,1,0;

	ShiftOnYTHS = 1.5;

	MIN_TRACKED_FEATURES = 40;

	MIN_PARALLAX = 7.5;
}