#include <eigen3/Eigen/Dense>
using namespace Eigen;
extern int MIN_DIST;
extern int MAX_CNT;
extern double BASELINE;
extern int WINDOW_SIZE;
extern double fx;
extern double cx;
extern double fy;
extern double cy;
extern double ShiftOnYTHS;
extern int MIN_TRACKED_FEATURES;
extern double MIN_PARALLAX;

extern Eigen::Matrix<double, 3, 4> Left_Cam_Pose;
extern Eigen::Matrix<double, 3, 4> Right_Cam_Pose;
void setParams();