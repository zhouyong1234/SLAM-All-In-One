//#include "core/core.hpp"  
//#include "highgui/highgui.hpp" 
//#include "imgproc/imgproc.hpp"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <vector>
#include <opencv/cv.h>
	
using namespace cv;   
 
Mat image;
Mat imageGray;
int thresh=5;   //角点个数控制
int MaxThresh=255;
 
void Trackbar(int,void*);  
 
int main(int argc,char*argv[])  
{  
	image=imread(argv[1]);
	cvtColor(image,imageGray,CV_RGB2GRAY);
	GaussianBlur(imageGray,imageGray,Size(5,5),1); // 滤波
	namedWindow("Corner Detected");
	createTrackbar("threshold：","Corner Detected",&thresh,MaxThresh,Trackbar);
	//imshow("Corner Detected",image);
	Trackbar(0,0);
	waitKey();
	return 0;
}  
 
void Trackbar(int,void*)
{
	Mat dst,imageSource;
	dst=Mat::zeros(image.size(),CV_32FC1);  
	imageSource=image.clone();
	std::vector<Point2f>corners;  
	goodFeaturesToTrack(imageGray,corners,thresh,0.01,10,Mat());
	for(int i=0;i<corners.size();i++)
	{
		circle(imageSource,corners[i],2,Scalar(0,0,255),2);
	}
	imshow("Corner Detected",imageSource); 
}
