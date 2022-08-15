#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Dense>
#include <pangolin/pangolin.h>
#include <vector>
#include <iostream>

#include "ImageManager.h"

using namespace std;
using namespace cv;
using namespace Eigen;

struct _Pose{
	Quaterniond quat;
	Vector3d pos;
};

typedef struct _Pose Pose;

class StateEstimator{
public:
	ImageManager img_manager;
	vector<Pose> poses;
	vector<Vector3d> point_cloud;
public:
	StateEstimator();
	~StateEstimator();
	void initialize();
	//void trackStereoFrames(const Mat& leftImg, const Mat& rightImg);
};