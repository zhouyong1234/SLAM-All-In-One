#include <iostream>
#include <opencv2/opencv.hpp>
#include "data_reader.h"
#include "types.h"
#include "common_include.h"
#include "config.h"
#include "feature.h"
#include <fstream>

using namespace std;
using namespace cv;
using namespace MSCKF_MINE;

void GenerateMask( Mat &mask ,vector<Point2f> &corners);

int main(int argc, char *argv[])
{

    Config::setParameterFile("../config/config.yaml");
    DataReader data;
    vector<CAMERA> vCamera = data.mvCameraData;
    unsigned int maxCorners = 300;
    double qualityLevel = 0.01;
    double minDistance = 20;
    int blockSize = 10;
    bool useHarrisDetector = false;
    double k = 0.04;

    int id_start = 0;
    int id_frame = 0;

    Mat currentImage, lastImage;
    vector<Point2f> corners;
    vector<Point2f> corners_after;

    Mat first_image = imread(vCamera[0].img_name, CV_LOAD_IMAGE_GRAYSCALE);
    goodFeaturesToTrack( first_image,
                         corners,
                         maxCorners,
                         qualityLevel,
                         minDistance,
                         Mat(),
                         blockSize,
                         useHarrisDetector,
                         k );
    lastImage = first_image.clone();

    VectorOfFeatures vFeatures;
    for(int i = 0; i < corners.size(); i++)
    {
        Feature feature;
        feature.mnId = i;
        feature.muv = corners[i];
        feature.mnFrameId = 0;
        feature.mvObservation.push_back(Vector2d(corners[i].x, corners[i].y));
        vFeatures.push_back(feature);

    }
    id_start = 300;


    /*so we need to manage the vFeatures*/


    for(vector<CAMERA>::iterator iter_cam = vCamera.begin() + 1; iter_cam!=vCamera.end(); iter_cam++)
    {
        id_frame++;
        Mat image = imread(iter_cam->img_name,CV_LOAD_IMAGE_GRAYSCALE);
        currentImage = image;

        /*L-K Tracking*/
        vector<uchar> status;
        vector<float> err;
        corners_after.clear();
        cout << "--Tracking information:" << endl;
        cout << "corners_before: \n" << corners.size() << endl;


        calcOpticalFlowPyrLK(lastImage,currentImage,corners,corners_after, status, err);
        lastImage = currentImage.clone();

        cout << "corners_after: \n" << corners_after.size() << endl;

        int j = 0;
        cvtColor(currentImage,currentImage,CV_GRAY2BGR);

        vector<Point2f> corners_before = corners;
        corners.clear();
        cout << "L-K -- > Corners_after: " << corners_after.size() << endl;
        cout << "L-K -- > Status_after: "  << status.size()        << endl;
        vector<int> vDelete;
        vector<int> vLongTrackedFeaturesIndex;
        for(int i=0;i<corners_after.size();i++)
        {

            if(status[i]
                    &&((abs(corners_before[i].x-corners_after[i].x)+abs(corners_before[i].y-corners_after[i].y))>=0)
                    &&((abs(corners_before[i].x-corners_after[i].x)+abs(corners_before[i].y-corners_after[i].y))<30)
                    &&corners_after[i].x > 0 && corners_after[i].x < 752 && corners_after[i].y > 0 && corners_after[i].y < 480)
            {
                corners.push_back(corners_after[i]);
                j++;
//                cv::circle( currentImage, corners[i], 2, Scalar(255,0,0), -1, 8, 0 );
                cv::circle( currentImage, corners_after[i], 3, Scalar(0,0,255), -1, 8, 0 );
//                cv::line(currentImage,corners[i],corners_after[i],Scalar(0,255,255),1,8,0);

                /*we need add the observation*/
                vFeatures[i].mvObservation.push_back(Vector2d(corners_after[i].x,corners_after[i].y));
                if(vFeatures[i].mvObservation.size() > 15)
                {
                    /*delete that feature or use it to update the filter*/
                    //vFeatures.erase(vFeatures.begin()+i);
                    vDelete.push_back(i);
                    vLongTrackedFeaturesIndex.push_back(i);
                    //corners.erase(corners.begin()+i );
                }


            }
            else
            {
                /*features that are not tracked, we need to delate it, but we should delete later*/
//                vFeatures.erase(vFeatures.begin()+i);
                vDelete.push_back(i);

            }

        }

        /*delete the untracked or more than 20 times observed points*/
        sort(vDelete.begin(),vDelete.end());
        sort(vLongTrackedFeaturesIndex.begin(),vLongTrackedFeaturesIndex.end());
        for(int i = vDelete.size() -1; i >=0 ; i--)
        {
            vFeatures.erase(vFeatures.begin()+vDelete[i]);
        }
        /*corners also should be deleted*/
        for(int i = vLongTrackedFeaturesIndex.size() -1; i >=0 ; i--)
        {
            corners.erase(corners.begin() + vLongTrackedFeaturesIndex[i]);
        }

        /*For this time we lost how many features*/

        cout << "We lost " << vDelete.size() << " features." << endl;


        /*for test we track 15th feature*/
        cout << "Information of 15th feature" << endl;
        cout << "Feature ID: " << vFeatures[14].mnId << endl;
        cout <<BOLDGREEN << "Frame ID:"    << vFeatures[14].mnFrameId <<WHITE <<  endl;
        Feature &feature = vFeatures[14];
        for(int i = 0; i < feature.mvObservation.size(); i++)
        {
            cout << "The " << i << "th observation\n" <<feature.mvObservation[i] << endl;
        }


        cout << "** Run Here --> corners_after.size = " << corners.size() << endl;

        /*add new corners since some features are lost*/
        /*step1:generate MASK and calc num to add*/

        /*we need to delete the feature which tracked for more than 10 times*/


        Mat mask = image.clone();
        GenerateMask(mask,corners);
        int cornersNum = maxCorners - corners.size();
        cout << "corners.size = " << corners.size() << endl;
        cout << "Need add " << cornersNum << endl;

        /*step2: calc the new corners*/

        vector<Point2f> corners_new;

        if(cornersNum > 20)
        {
            goodFeaturesToTrack( image,
                                 corners_new,
                                 cornersNum,
                                 qualityLevel,
                                 minDistance,
                                 mask,
                                 blockSize,
                                 useHarrisDetector,
                                 k );

            /*contact the corners*/
            corners.insert(corners.end(), corners_new.begin(), corners_new.end());
            /*since we add new features, we need to add to the vFeatures*/
            for(int i = 0; i < corners_new.size(); i++)
            {
                Feature feature;
                feature.mnId = id_start++;
                feature.muv = corners_new[i];
                feature.mnFrameId = id_frame;
                feature.mvObservation.push_back(Vector2d(corners_new[i].x, corners_new[i].y));
                vFeatures.push_back(feature);
            }


        }

        /// Draw corners detected
        cout<<"** Number of corners new added : "<<corners_new.size()<<endl;
        cout<<"** Number of corners detected: "<<corners.size()<<endl;
        int r = 3;
        for( int i = 0; i < corners_new.size(); i++ )
        { circle( currentImage, corners_new[i], r, Scalar(0,255,0), -1, 8, 0 ); }

        cv::TermCriteria criteria = cv::TermCriteria(
                    cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS,
                    40,
                    0.01);

        cv::cornerSubPix(image, corners, cv::Size(5, 5), cv::Size(-1, -1), criteria);
        //cout << "corners = \n" << corners << endl;
        cv::imshow("Tracking-New", currentImage);
        cv::waitKey(0);

    }

    return 0;
}

void GenerateMask( Mat &mask ,vector<Point2f> &corners)
{
    size_t r = 5;
    /*the mask must be CV_8UC1*/
    for(vector<Point2f>::iterator iter_corner = corners.begin(); iter_corner!=corners.end(); iter_corner++ )
    {
        Point2f &corner = *iter_corner;

        float top_left_x = corner.x - r;

        if(top_left_x > mask.cols)
            top_left_x = mask.cols;
        else if (top_left_x < 0)
            top_left_x = 0;

        float top_left_y = corner.y - r;

        if(top_left_y > mask.rows)
            top_left_y = mask.rows;
        else if (top_left_y < 0)
            top_left_y = 0;

        Point2f top_left = Point2f(top_left_x,top_left_y);

        float bottom_right_x = corner.x + r;

        if(bottom_right_x > mask.cols)
            bottom_right_x = mask.cols;
        else if (bottom_right_x < 0)
            bottom_right_x = 0;

        float bottom_right_y = corner.y + r;

        if(bottom_right_y > mask.rows)
            bottom_right_y = mask.rows;
        else if (bottom_right_y < 0)
            bottom_right_y = 0;

        Point2f bottom_right = Point2f(bottom_right_x,bottom_right_y);

        cv::rectangle(mask, top_left, bottom_right, Scalar(0), -1, 8);

    }

}
