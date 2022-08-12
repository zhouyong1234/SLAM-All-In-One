#include "common_include.h"
#include "camera.h"
#include "config.h"
#include "msckf.h"
#include "data_reader.h"
#include "ORBextractor.h"
#include <vector>

using namespace MSCKF_MINE;



cv::Mat showFeatures(const cv::Mat &mImage, const vector<KeyPoint> &mvKeys)
{
    cv::Mat imageWithFeatures = mImage.clone();
    cv::cvtColor(imageWithFeatures, imageWithFeatures, CV_GRAY2BGR);
    for(vector<KeyPoint>::const_iterator iter= mvKeys.begin(); iter!=mvKeys.end(); iter++ )
    {
        KeyPoint point = *iter;
        cv::circle(imageWithFeatures, Point(point.pt.x, point.pt.y), 3, Scalar(0,255,0), 1);
    }

    return imageWithFeatures.clone();

}


int main(int argc, char *argv[])
{
    Config::setParameterFile("../config/config.yaml");

    MSCKF msckf;

    DataReader data;
    vector<CAMERA> vCameraData = data.mvCameraData;

    for(vector<CAMERA>::iterator iter = vCameraData.begin(); iter!=vCameraData.end();iter++)
    {
        string imagePath = iter->img_name;
        Mat image = cv::imread(imagePath,CV_LOAD_IMAGE_GRAYSCALE);
        //cout << "image_path = " << imagePath << endl;
        msckf.mImage = image.clone();
        msckf.extractFeatures();
        cv::Mat imFeature = showFeatures(msckf.mImage, msckf.mvKeys);
        cv::imshow("features", imFeature);
        cout << "feature size = " << msckf.mvKeys.size() << endl;

        cv::waitKey(0);

    }

    return 0;
}
