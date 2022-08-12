#ifndef FRAME_H
#define FRAME_H

#include "common_include.h"
#include "config.h"
#include "camera.h"
#include "converter.h"
#include "ORBextractor.h"

namespace MSCKF_MINE
{

#define FRAME_GRID_ROWS 48
#define FRAME_GRID_COLS 64

class Frame
{
public:
    Frame();

    Frame(const Frame &frame);

    Frame(const Mat &im, const double &timeStamp, ORBextractor* extractor);

    ORBextractor* mpORBextractor;

    double mTimeStamp;

    Camera mCamera;

    int N;
    unsigned int mnId;
    static long unsigned int nNextId;

    std::vector<cv::KeyPoint> mvKeys;
    std::vector<cv::KeyPoint> mvKeysUn;

    cv::Mat mDescriptors;

    cv::Mat mImageGray;

    map<int,int> matchesId; //matches  currentframe features[id], lastframe features[id]

    static bool mbInitialComputations;

    // Keypoints are assigned to cells in a grid to reduce matching complexity when projecting MapPoints.
    static float mfGridElementWidthInv;
    static float mfGridElementHeightInv;
    std::vector<std::size_t> mGrid[FRAME_GRID_COLS][FRAME_GRID_ROWS];

    // Undistorted Image Bounds (computed once).
    static float mnMinX;
    static float mnMaxX;
    static float mnMinY;
    static float mnMaxY;

public:
    void ExtractORB(const cv::Mat &im, int flag = 0);
    void unDistortImage();

    vector<size_t> GetFeaturesInArea(const float &x, const float  &y, const float  &r, const int minLevel=-1, const int maxLevel=-1) const;

    // Computes image bounds for the undistorted image (called in the constructor).
    void ComputeImageBounds(const cv::Mat &imLeft);

    // Assign keypoints to the grid for speed up feature matching (called in the constructor).
    void AssignFeaturesToGrid();

    // Compute the cell of a keypoint (return false if outside the grid)
    bool PosInGrid(const cv::KeyPoint &kp, int &posX, int &posY);


};

}



#endif // FRAME_H
