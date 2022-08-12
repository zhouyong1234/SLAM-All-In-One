#ifndef FEATURE_H
#define FEATURE_H

#include "common_include.h"

using namespace std;
using namespace Eigen;
using namespace cv;

namespace MSCKF_MINE
{

class Feature
{
public:
    unsigned int     mnId;          /*feature id*/
    cv::Point2f      muv;           /*the position in the most current image*/
    vector<Vector2d> mvObservation; /*since the feature are tracked through time, so the uv are not only one*/
    unsigned int     mnFrameId;     /*The feature first were extract in which frame*/

    Feature();
};
typedef std::vector<Feature> VectorOfFeatures;



}


#endif // FEATURE_H
