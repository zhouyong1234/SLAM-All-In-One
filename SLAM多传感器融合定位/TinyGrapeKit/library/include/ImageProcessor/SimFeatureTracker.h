#pragma once

#include <set>
#include <unordered_map>
#include <vector>

#include <Eigen/Core>

namespace TGK {
namespace ImageProcessor {

class SimFeatureTrakcer {
public: 
    SimFeatureTrakcer();

    void TrackSimFrame(const std::vector<Eigen::Vector2d>& sim_features, const std::vector<long int> sim_feature_ids,
                       std::vector<Eigen::Vector2d>* tracked_pts, 
                       std::vector<long int>* tracked_pt_ids,
                       std::vector<long int>* lost_pt_ids = nullptr,
                       std::set<long int>* new_pt_ids = nullptr);

    virtual void DeleteFeature(const long int ft_id);

private:
    std::unordered_map<long int, Eigen::Vector2d> last_features_;
};

}  // namespace ImageProcessor
}  // namespace TGK