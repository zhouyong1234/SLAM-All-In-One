#include "opencv2/opencv.hpp"
 
using namespace cv;
using namespace std;
 
int main(int argc,char** argv)
{
  if (argc != 3) {
    cout << "please input image 1 and image 2 file!" << endl;
    return -1;
  }
  string image1 = argv[1];
  string image2 = argv[2];
	// Read the images to be aligned 读取仿射图像
	//im1参考图像，im2要处理的图像
	Mat im1 = imread(image1,IMREAD_GRAYSCALE);
	Mat im2 = imread(image2,IMREAD_GRAYSCALE);
 
	// Convert images to gray scale 转换为灰度图像
	Mat im1_gray = im1.clone(), im2_gray = im2.clone();
	//cvtColor(im1, im1_gray, CV_BGR2GRAY);
	//cvtColor(im2, im2_gray, CV_BGR2GRAY);

	equalizeHist(im1,im1_gray);
  equalizeHist(im2,im2_gray);

	// Define the motion model 定义运动模型
	const int warp_mode = MOTION_EUCLIDEAN;
 
	// Set a 2x3 or 3x3 warp matrix depending on the motion model. 变换矩阵
	Mat warp_matrix;
 
	// Initialize the matrix to identity
	if (warp_mode == MOTION_HOMOGRAPHY)
	{
		warp_matrix = Mat::eye(3, 3, CV_32F);
	}
	else
	{
		warp_matrix = Mat::eye(2, 3, CV_32F);
	}
 
	// Specify the number of iterations. 算法迭代次数
	int number_of_iterations = 5000;
 
	// Specify the threshold of the increment
	// in the correlation coefficient between two iterations 设定阈值
	double termination_eps = 1e-10;
 
	// Define termination criteria 定义终止条件
	TermCriteria criteria(TermCriteria::COUNT + TermCriteria::EPS, number_of_iterations, termination_eps);
 
	// Run the ECC algorithm. The results are stored in warp_matrix. ECC算法
	findTransformECC
	(
		im1_gray,
		im2_gray,
		warp_matrix,
		warp_mode,
		criteria
	);
 
	// Storage for warped image.
	Mat im2_aligned;
 
	if (warp_mode != MOTION_HOMOGRAPHY)
	{
		// Use warpAffine for Translation, Euclidean and Affine
		warpAffine(im2, im2_aligned, warp_matrix, im1.size(), INTER_LINEAR + WARP_INVERSE_MAP);
	}
	else
	{
		// Use warpPerspective for Homography
		warpPerspective(im2, im2_aligned, warp_matrix, im1.size(), INTER_LINEAR + WARP_INVERSE_MAP);
	}
 
  cout << "Warp Matrix = " << warp_matrix << endl;

	// Show final result
	imshow("Image 1", im1);
	imshow("Image 2", im2);
	imshow("Image 2 Aligned", im2_aligned);
	waitKey(0);
 
	return 0;
}
