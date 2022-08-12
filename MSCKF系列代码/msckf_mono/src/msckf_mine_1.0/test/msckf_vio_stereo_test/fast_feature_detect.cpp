#include <iostream>
#include <opencv2/opencv.hpp>
#include "data_reader.h"
#include "types.h"
#include "common_include.h"
#include "config.h"

using namespace std;
using namespace cv;
using namespace MSCKF_MINE;

Mat ShowFeatures(Mat image, vector<KeyPoint>& kp)
{
    cvtColor(image,image,CV_GRAY2BGR);
    for(int i = 0; i < kp.size(); i++)
    {
        cv::circle( image, kp[i].pt, 3, Scalar(0,255,0), -1, 8, 0 );
    }
    return image;
}

int main(int argc, char *argv[])
{
    Config::setParameterFile("../config/config.yaml");
    DataReader data;
    vector<CAMERA> vCamera = data.mvCameraData;
    for(vector<CAMERA>::iterator iter_cam = vCamera.begin(); iter_cam!=vCamera.end(); iter_cam++)
    {
        Mat image = imread(iter_cam->img_name, CV_LOAD_IMAGE_GRAYSCALE);
        cv::Ptr<Feature2D> detector_ptr = FastFeatureDetector::create(10);
        vector<KeyPoint> new_features(0);
        detector_ptr->detect(image, new_features);
        Mat img_features = ShowFeatures(image.clone(), new_features);
        cout << new_features.size() << " features are detected in this frame." << endl;
        imshow("Features", img_features);
        waitKey(0);
    }

    return 0;
}
