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
    cv::Mat lastFrame;
    vector<KeyPoint> lastKeyPoints;
    Mat lastDesc;
    unsigned int count = 0;

    for(vector<CAMERA>::iterator iter = vCameraData.begin(); iter!=vCameraData.end();iter++)
    {
        string imagePath = iter->img_name;
        Mat image = cv::imread(imagePath,CV_LOAD_IMAGE_GRAYSCALE);
        //cout << "image_path = " << imagePath << endl;
        msckf.mImage = image.clone();
        msckf.extractFeatures();
        cv::Mat imFeature = showFeatures(msckf.mImage, msckf.mvKeys);
        cv::imshow("features", imFeature);

        if(count)
        {
            /*match*/
            /*for now we have get the two image and its keypoints with descriptors*/
            FlannBasedMatcher matcher;
            std::vector<cv::DMatch> matches;
            matcher.match(msckf.mDescriptors, lastDesc, matches);

            //-- Quick calculation of max and min distances between keypoints
            double max_dist = 0; double min_dist = 100;
            for( int i = 0; i < msckf.mDescriptors.rows; i++ )
            { double dist = matches[i].distance;
              if( dist < min_dist ) min_dist = dist;
              if( dist > max_dist ) max_dist = dist;
            }

            printf("-- Max dist : %f \n", max_dist );
            printf("-- Min dist : %f \n", min_dist );

            //-- Draw only "good" matches (i.e. whose distance is less than 2*min_dist,
            //-- or a small arbitary value ( 0.02 ) in the event that min_dist is very
            //-- small)
            //-- PS.- radiusMatch can also be used here.
            std::vector< DMatch > good_matches;

            for( int i = 0; i < msckf.mDescriptors.rows; i++ )
            { if( matches[i].distance <= max(2*min_dist, 0.02) )
              { good_matches.push_back( matches[i]); }
            }

            //-- Draw only "good" matches
            Mat img_matches;
            drawMatches( msckf.mImage, msckf.mvKeys, lastFrame, lastKeyPoints,
                         good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
                         vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

            //-- Show detected matches
            imshow( "Good Matches", img_matches);


        }




        lastFrame = msckf.mImage.clone();
        lastKeyPoints = msckf.mvKeys;
        lastDesc = msckf.mDescriptors.clone();
        count++;

        cv::waitKey(0);

    }

    return 0;
}
