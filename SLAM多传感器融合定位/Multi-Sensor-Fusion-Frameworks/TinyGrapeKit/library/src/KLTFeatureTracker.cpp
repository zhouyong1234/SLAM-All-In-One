#include <ImageProcessor/KLTFeatureTracker.h>

#include <glog/logging.h>


namespace TGK {
namespace ImageProcessor {

KLTFeatureTracker::KLTFeatureTracker(const Config& config) : config_(config) { }

void KLTFeatureTracker::TrackImage(const cv::Mat& image, 
                                   std::vector<Eigen::Vector2d>* tracked_pts, 
                                   std::vector<long int>* tracked_pt_ids,
                                   std::vector<long int>* lost_pt_ids,
                                   std::set<long int>* new_pt_ids) {
    tracked_pts->clear();
    tracked_pt_ids->clear();
    if (lost_pt_ids != nullptr) { lost_pt_ids->clear(); }
    if (new_pt_ids != nullptr) { new_pt_ids->clear(); }

    // 1. Track features from the last frame.
    if (!last_image_.empty() && !last_pt_ids_.empty()) {
        std::vector<cv::Point2f> track_cv_pts;
        std::vector<uchar> track_status;
        std::vector<float> error;
        // Forward track.
        // 光流跟踪
        cv::calcOpticalFlowPyrLK(last_image_, image, last_cv_pts_, track_cv_pts, track_status, error,
                                 cv::Size(51, 51), 5, 
                                 cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 30, 0.01));
        
        // Backward track.
        std::vector<cv::Point2f> bacK_track_cv_pts = last_cv_pts_;
        std::vector<uchar> back_track_status;
        std::vector<float> back_error;
        cv::calcOpticalFlowPyrLK(image, last_image_, track_cv_pts, bacK_track_cv_pts, back_track_status, back_error,
                                 cv::Size(51, 51), 5, 
                                 cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 30, 0.01),
                                 cv::OPTFLOW_USE_INITIAL_FLOW);
        
        std::vector<cv::Point2f> prev_points;
        std::vector<cv::Point2f> cur_points;
        std::vector<int> cur_ids;
        for (size_t i = 0; i < track_cv_pts.size(); ++i) {
            const auto& pt_id = last_pt_ids_[i];

            if (track_status[i] == 0 || back_track_status[i] == 0) { 
                if (lost_pt_ids != nullptr) { lost_pt_ids->push_back(pt_id); }
                continue; 
            }

            prev_points.push_back(last_cv_pts_[i]);
            cur_points.push_back(track_cv_pts[i]);
            cur_ids.push_back(pt_id);
        }

        if (!cur_points.empty()) {
            // Remove with F.
            // 去除外点
            std::vector<uchar> f_status;
            const double f_thresh = 0.5;
            cv::findFundamentalMat(cur_points, prev_points, cv::FM_RANSAC, f_thresh, 0.999, f_status);

            for (size_t i = 0; i < cur_points.size(); ++i) {
                if (!f_status[i]) {
                    continue;
                }

                tracked_pts->emplace_back(cur_points[i].x, cur_points[i].y);
                tracked_pt_ids->push_back(cur_ids[i]);
            }
        }
    }

    // 2. Create new features.
    const int num_new_pts = config_.max_num_corners - tracked_pt_ids->size();
    if (num_new_pts > 0) {
        cv::Mat mask;
        CreateMask(image.cols, image.rows, *tracked_pts, &mask);

        std::vector<cv::Point2f> new_pts;
        // 角点检测
        cv::goodFeaturesToTrack(image, new_pts, num_new_pts, config_.quality_level, config_.min_dist, mask);

        for (const auto& pt : new_pts) {
            const auto id = corner_id_;
            ++corner_id_;

            if (new_pt_ids != nullptr) { new_pt_ids->insert(id); }

            tracked_pts->emplace_back(pt.x, pt.y);
            tracked_pt_ids->push_back(id);
        }   
    }

    // 3. Reset last frame data.
    last_image_ = image;
    last_pt_ids_ = *tracked_pt_ids;
    last_cv_pts_.clear();
    last_cv_pts_.reserve(tracked_pts->size());
    for (size_t i = 0; i < tracked_pts->size(); ++i) {
        last_cv_pts_.emplace_back(tracked_pts->at(i)[0], tracked_pts->at(i)[1]);
    }
}

void KLTFeatureTracker::CreateMask(const int width, const int heigth, 
                                const std::vector<Eigen::Vector2d>& points, cv::Mat* mask) {
    *mask = cv::Mat(heigth, width, CV_8UC1, cv::Scalar(1));
    for (const auto& pt : points) {
        cv::circle(*mask, cv::Point2i(pt[0], pt[1]), config_.min_dist, 0, -1);
    }
} 

void KLTFeatureTracker::DeleteFeature(const long int pt_id) {
    const auto iter = std::find(last_pt_ids_.begin(), last_pt_ids_.end(), pt_id);
    if (iter == last_pt_ids_.end()) { return; }
    int idx = std::distance(last_pt_ids_.begin(), iter);
    last_pt_ids_.erase(iter);
    last_cv_pts_.erase(last_cv_pts_.begin() + idx);
}

}  // namespace ImageProcessor
}  // namespace TGK