#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <pangolin/pangolin.h>
#include <iostream>
#include <mutex>

#include "StateEstimator.h"

//#include "params.h"
using namespace std;
using namespace cv;

class OpticalSLAM{
public:
	StateEstimator state_estimator;
	bool is_first_img;
	bool ready_to_display;
	Mat img_to_display;
	mutex m_dis;
	mutex m_pose;
	int nFrames;

	vector<Matrix3d> Rp;//实际旋转和位移
  	vector<Vector3d> tp;
  	vector<Vector3d> trj;
public:
	OpticalSLAM();
	~OpticalSLAM();
	void initialize();
	void display();
	void trackStereoFrames(const Mat& leftImg, const Mat& rightImg);
	void setFirstFrame(const Mat& leftImg, const Mat& rightImg);
};