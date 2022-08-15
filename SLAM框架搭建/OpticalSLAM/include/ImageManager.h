#include <eigen3/Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <iostream>
#include <unordered_map>
#include <memory>

#include "params.h"

using namespace std;
using namespace cv;
using namespace Eigen;
// struct Feature{
// 	int frameID;
// 	Vector2d leftPoint, rightPoint;
// };
class LandMark{
public:
	bool state;
    unsigned long id;
    vector<pair<int,Point2f>> observation;//frameID, 归一化坐标
    //Vector3d puv;//归一化坐标
    Point3f pos;
    double position[3];
    double depth;
public:
	LandMark(){
		id = -1;
		state = false;
		depth = -1;
	}
};

class ImageManager
{
public:
	unsigned long frame_cnt;
	long int id_cnt;
	vector<long int> ids;
	unordered_map<unsigned long, shared_ptr<LandMark>> landmarks;
	//vector<pair<unsigned long, shared_ptr<LandMark>> landmarks;
	Mat cur_left_img, cur_right_img;
	Mat prev_left_img, prev_right_img;

	std::vector<cv::Mat> prev_left_pyramid_;
	std::vector<cv::Mat> prev_right_pyramid_;
  	std::vector<cv::Mat> cur_left_pyramid_;
  	std::vector<cv::Mat> cur_right_pyramid_;

  	vector<Point2f> prev_left_pts, cur_left_pts;
  	vector<Point2f> prev_right_pts, cur_right_pts;

  	vector<Point2f> pre, cur;

  	Mat mask;
  	bool is_first_track;

  	
  	vector<Matrix3d> Rp;//实际旋转和位移
  	vector<Vector3d> tp;
  	// Matrix3d Rp[WINDOW_SIZE];
  	// Vector3d tp[WINDOW_SIZE];

  	Vec4d cam_intrinsics;

  	int frameID0; //frameIDmid;
public:
	ImageManager();
	~ImageManager();
	void setFirstFrame(const Mat& leftImg, const Mat& rightImg);
	void stereoMatch(std::vector<cv::Point2f>& cam0_points,
      std::vector<cv::Point2f>& cam1_points,
      std::vector<unsigned char>& inlier_markers);
	void createImagePyramids();
	void trackStereoFrames(const Mat& leftImg, const Mat& rightImg);
	void buildCoarse3DStructure();
	void triangulatePoint(Eigen::Matrix<double, 3, 4> &Pose0, Eigen::Matrix<double, 3, 4> &Pose1,
                        Eigen::Vector2d &point0, Eigen::Vector2d &point1, Eigen::Vector3d &point_3d);
	double averageParallax(const vector<Point2f>& pts0, const vector<Point2f>& pts1, const vector<uchar>& status);
private:
	void setMask();
	void checkEpipolarConstraint(const vector<Point2f>& pts0, const vector<Point2f>& pts1, vector<uchar>& status, double threshold);
	void checkShiftsOnY(const vector<Point2f>& pts0, const vector<Point2f>& pts1, vector<uchar>& status, double threshold);
	void getPairsForPnP(int id0, int id1, vector<Point3f>& p3d, vector<Point2f>& p2d, int tid);
	int getTrackNumBetweenTwoFrame(int i, int j);
	bool solvePnP(const vector<Point3f>& p3d, const vector<Point2f>& p2d, Matrix3d &R_initial, Vector3d &P_initial, bool use_ransac);
};