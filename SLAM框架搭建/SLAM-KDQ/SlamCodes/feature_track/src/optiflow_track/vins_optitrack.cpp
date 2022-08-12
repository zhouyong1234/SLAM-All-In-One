#include <sys/stat.h>
#include <chrono>
#include "optiflow_track/vins_optitrack.hpp"

OptiflowTrackByVINS::OptiflowTrackByVINS(int featSize,float thres,float minDist,std::string savePath,bool trackBack):
featureSize_(featSize),fastResponseThreshold_(thres),min_dist_(minDist),trackBack_(trackBack) {
  id_ = 0;
  std::string par_path(savePath+"/vins_track/");
  if (!par_path.empty()) {
    struct stat info{};
    for (int i = 0; i < 5; i++) {
      save_image_path_ = par_path + std::to_string(i);
      if (stat(save_image_path_.c_str(), &info) == 0 ) {
        if (i == 4) {
          std::string cmd("rm -rf " + par_path);
          int sta = system(cmd.c_str());
          if(!WIFEXITED(sta)) {
            save_image_path_ = "";
          } else {
            std::cout << "Warning: too many replay folders. remove old ones.\n";
            i = -1;
            continue;
          }
        } else {
          continue;
        }  
      }
      std::string cmd("mkdir -p " + save_image_path_);
      int status = system(cmd.c_str());
      if (!WIFEXITED(status)) {
        save_image_path_ = "";
      } else {
        break;
      }
    }
  }
  //cv::namedWindow("track",CV_WINDOW_AUTOSIZE);
}

OptiflowTrackByVINS::~OptiflowTrackByVINS() {

}

bool OptiflowTrackByVINS::inBorder(const cv::Point2f& pt) {
  int img_x = cvRound(pt.x);
  int img_y = cvRound(pt.y);
  return img_x > 2 && img_x < (cols_ - 2) && img_y > 2 && img_y < (rows_ - 2); 
}

double OptiflowTrackByVINS::distance(const cv::Point2f& pt1,const cv::Point2f& pt2) {
  double dx = pt1.x - pt2.x;
  double dy = pt1.y - pt2.y;
  return sqrt(dx * dx + dy * dy);
}

template <typename RType>
void OptiflowTrackByVINS::reduceVector(const std::vector<uchar>& status,std::vector<RType>& vec) {
  int j = 0;
  for (size_t i = 0; i < status.size(); i++) {
    if(status[i]) {
      vec[j++] = vec[i];
    }
  }
  vec.resize(j);
}

void OptiflowTrackByVINS::setMask() {
  if(cur_pts_.empty()) {
    return;
  }
  mask_ = cv::Mat(rows_, cols_, CV_8UC1, cv::Scalar(255));
  std::vector<std::pair<int, std::pair<cv::Point2f, int>>> cnt_pts_id;
  for (unsigned int i = 0; i < cur_pts_.size(); i++) {
    cnt_pts_id.push_back(std::make_pair(track_cnt_[i], std::make_pair(cur_pts_[i], ids_[i])));
  }
  sort(cnt_pts_id.begin(), cnt_pts_id.end(), [](const std::pair<int, std::pair<cv::Point2f, int>> &a, 
    const std::pair<int, std::pair<cv::Point2f, int>> &b) {
      return a.first > b.first;
    });
  cur_pts_.clear();
  ids_.clear();
  track_cnt_.clear();
  for (auto &it : cnt_pts_id) {
    if (mask_.at<uchar>(it.second.first) == 255) {
        cur_pts_.push_back(it.second.first);
        ids_.push_back(it.second.second);
        track_cnt_.push_back(it.first);
        cv::circle(mask_, it.second.first, min_dist_, 0, -1);
    }
  }
}



std::vector<cv::Point2f> OptiflowTrackByVINS::optiTrackFeature(double timestamp,const cv::Mat& img) {
  if(img.empty()) {
    std::cout << "image is empty" << std::endl;
    return cur_pts_;
  }
  auto current_tic = std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
  t_ = timestamp;
  cols_ = img.cols;
  rows_ = img.rows;
  prev_img_ = cur_img_.clone();
  cur_img_ = img.clone();
  prev_pts_.clear();
  prev_pts_ = cur_pts_;
  if(!prev_pts_.empty()) {
    std::vector<uchar> status;
    std::vector<float> err;
    cv::calcOpticalFlowPyrLK(prev_img_,cur_img_,prev_pts_,cur_pts_,status,err,
        cv::Size(32,32),3,cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS,30,0.01));
    if(trackBack_) {
      std::vector<cv::Point2f> reverse_pts = prev_pts_;
      std::vector<uchar> reverse_status;
      std::vector<float> reverse_err;
      cv::calcOpticalFlowPyrLK(cur_img_,prev_img_,cur_pts_,reverse_pts,reverse_status,reverse_err,
      cv::Size(32,32),3,cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS,30,0.01));
      for(size_t i = 0; i < cur_pts_.size();i++) {
        if(status[i] && (!reverse_status[i] || distance(reverse_pts[i],prev_pts_[i]) > 0.5 )) 
          status[i] = 0;
      }
    }
    for(size_t i = 0; i < cur_pts_.size();i++) {
      if(status[i] && !inBorder(cur_pts_[i])) {
        status[i] = 0;
      }
    }
    reduceVector<cv::Point2f>(status,cur_pts_);
    reduceVector<int>(status,ids_);
    reduceVector<int>(status,track_cnt_);
    for (size_t i = 0; i < track_cnt_.size(); i++) {
      track_cnt_[i]++;
    }
    track_size_ = cur_pts_.size();

  } 
  auto current_toc = std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
  track_cost_ = static_cast<double>(current_toc - current_tic) * 1e3;
  if(cur_pts_.size() < featureSize_) {
    setMask();
    std::vector<cv::Point2f> new_pts;
    cv::goodFeaturesToTrack(cur_img_, new_pts, featureSize_ - cur_pts_.size(),fastResponseThreshold_, min_dist_, mask_);
    for (size_t i = 0; i < new_pts.size(); i++){
      id_++;
      ids_.push_back(id_);
      cur_pts_.push_back(new_pts[i]);
      track_cnt_.push_back(0);
    }
  }
  current_tic = std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
  detect_cost_ = static_cast<double>(current_tic - current_toc) * 1e3;
  if (true)
    drawTrack();
  if(!track_img_.empty()) {
 
    cv::imshow("track",track_img_);
    cv::waitKey(1);
  }
  return cur_pts_;
}

void OptiflowTrackByVINS::drawTrack() {
  bool expandDisplayFlg = false;
  cv::Mat cur_img_clr;
  cv::cvtColor(cur_img_, cur_img_clr, CV_GRAY2RGB);
  if(std::max(cols_,rows_) < 400 ) {
    expandDisplayFlg = true;
    cv::resize(cur_img_clr,cur_img_clr,cv::Size(cols_*2,rows_*2));
  }
  for (size_t j = 0; j < cur_pts_.size(); j++) {
    double len = std::min(1.0, 1.0 * track_cnt_[j] / 5);
    cv::Point2f cur_pt = cur_pts_[j];
    if(expandDisplayFlg) {
      cur_pt *=2.0; 
    } 
    int b_c = 255, g_c = 0;
    if(track_cnt_[j] > 0) {
      b_c = 0;
      g_c = 255;
    }
    cv::circle(cur_img_clr, cur_pt, 2, cv::Scalar(b_c, 0, g_c), 2);
    char id_str[5],track_cnt[3];
    snprintf(id_str, 5, "%d", ids_[j]);
    snprintf(track_cnt, 3, "%d",track_cnt_[j]);
    cv::putText(cur_img_clr, std::string(id_str),cur_pt + cv::Point2f(6, 5),cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0,255,0));
    cv::putText(cur_img_clr, std::string(track_cnt),cur_pt + cv::Point2f(6, -5), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255,0,0));
  }
  char cost_time[2][12],track_sum[2][12];
  snprintf(cost_time[0], 12, "%.3f", track_cost_);
  snprintf(cost_time[1], 12, "%.3f", detect_cost_);
  snprintf(track_sum[0], 12, "%3d", static_cast<int>(cur_pts_.size()));
  snprintf(track_sum[1], 12, "%3d", track_size_);

  cv::putText(cur_img_clr, "time-s:" + std::to_string(t_),cv::Point2f(10, 10),cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0,0,255));
  cv::putText(cur_img_clr, "getFts:" + std::string(track_sum[0]),cv::Point2f(10, 25),cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0,0,255));
  cv::putText(cur_img_clr, "trkFts:" + std::string(track_sum[1]),cv::Point2f(10, 40),cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0,0,255));
  cv::putText(cur_img_clr, "trkCst:" + std::string(cost_time[0]),cv::Point2f(10, 55),cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0,0,255));
  cv::putText(cur_img_clr, "detCst:" + std::string(cost_time[1]),cv::Point2f(10, 70),cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0,0,255));

  track_img_ = cur_img_clr.clone();
  if(!save_image_path_.empty()) {
    char timestamp[12];
    int str_n = snprintf(timestamp, 12, "%8.3f", t_);
    cv::imwrite(save_image_path_+ '/' + std::string(timestamp) + ".png",track_img_);
  }
}