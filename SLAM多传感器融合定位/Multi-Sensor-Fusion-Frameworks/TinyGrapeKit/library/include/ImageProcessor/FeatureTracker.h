#pragma once

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>

namespace TGK {
namespace ImageProcessor {

class FeatureTracker {
public:
    FeatureTracker();
    virtual ~FeatureTracker() { }

    virtual void TrackImage(const cv::Mat& image, 
                            std::vector<Eigen::Vector2d>* tracked_pts, 
                            std::vector<long int>* tracked_pt_ids,
                            std::vector<long int>* lost_pt_ids = nullptr,
                            std::set<long int>* new_pt_ids = nullptr) = 0;

    virtual void DeleteFeature(const long int pt_id) = 0;

protected:
    static long int corner_id_;
};

}  // namespace ImageProcessor
}  // namespace TGK