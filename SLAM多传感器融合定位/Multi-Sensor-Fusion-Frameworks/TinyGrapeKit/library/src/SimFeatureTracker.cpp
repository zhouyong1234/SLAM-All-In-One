#include <ImageProcessor/SimFeatureTracker.h>

#include <unordered_set>

namespace TGK {
namespace ImageProcessor {

SimFeatureTrakcer::SimFeatureTrakcer() { }

void SimFeatureTrakcer::TrackSimFrame(const std::vector<Eigen::Vector2d>& sim_features, 
                                      const std::vector<long int> sim_feature_ids,
                                      std::vector<Eigen::Vector2d>* tracked_pts, 
                                      std::vector<long int>* tracked_pt_ids,
                                      std::vector<long int>* lost_pt_ids,
                                      std::set<long int>* new_pt_ids) {
    *tracked_pts = sim_features;
    *tracked_pt_ids = sim_feature_ids;

    if (lost_pt_ids != nullptr) {
        lost_pt_ids->clear();
        const std::unordered_set<long int> cur_ids_set(sim_feature_ids.begin(), sim_feature_ids.end());
        for (const auto& last_id_ft : last_features_) {
            const long int last_ft_id = last_id_ft.first;
            if (cur_ids_set.count(last_ft_id) == 0) {
                lost_pt_ids->push_back(last_ft_id);
            }
        }
    }

    if (new_pt_ids != nullptr) {
        new_pt_ids->clear();
        for (const long int cur_ft_id : sim_feature_ids) {
            if (last_features_.find(cur_ft_id) == last_features_.end()) {
                new_pt_ids->insert(cur_ft_id);
            }
        }
    }

    // Reset last features.
    last_features_.clear();
    for (size_t i = 0; i < sim_feature_ids.size(); ++i) {
        const long int ft_id = sim_feature_ids[i];
        const Eigen::Vector2d& ft = sim_features[i];
        last_features_[ft_id] = ft;
    }
}

void SimFeatureTrakcer::DeleteFeature(const long int ft_id) {
    auto iter = last_features_.find(ft_id);
    if (iter != last_features_.end()) {
        last_features_.erase(iter);
    }
}

}  // namespace ImageProcessor
}  // namespace TGK