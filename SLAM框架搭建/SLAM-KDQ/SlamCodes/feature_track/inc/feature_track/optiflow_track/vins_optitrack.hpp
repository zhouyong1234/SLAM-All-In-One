#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>

class OptiflowTrackByVINS {
  private:
    cv::Mat prev_img_,cur_img_,track_img_,mask_;
    std::vector<cv::Point2f> prev_pts_, cur_pts_;
    std::vector<int> ids_,track_cnt_;
    int id_,track_size_;
    double t_,track_cost_,detect_cost_;
    float fastResponseThreshold_;
    int featureSize_,cols_,rows_;
    bool trackBack_;
    float min_dist_;
    std::string save_image_path_;
    /** @brief add mask at points those were tracked  
      * @param none
      */ 
    void setMask();
    /** @brief show track result 
      * @param none 
      */ 
    void drawTrack();
    /** @brief  calculate distance of pixel range between two pixels
      * @param pt1 - pixel coordinate in image
      * @param pt2 - pixel coordinate in another image
      */ 
    double distance(const cv::Point2f& pt1,const cv::Point2f& pt2);
    /** @brief check pixel coordinate whether is out of image size or not 
      * @param pt - pixel coordinate
      */  
    bool inBorder(const cv::Point2f& pt);
    /** @brief reduce object vector throngh the status vector 
      * @param status - status vector
      * @param vec - object vector 
      */  
    template <typename RType>
    static void reduceVector(const std::vector<uchar>& status,std::vector<RType>& vec);

  public:
    /** @brief Constructor
      */ 
    OptiflowTrackByVINS(int featSize,float thres,float minDist,std::string savePath,bool trackBack);
    /** @brief Disconstructor
      */ 
    ~OptiflowTrackByVINS();
    /** @brief track features in current image using KLT Optflow Algrithm
      * @param timestamp - timestamp of image
      * @param img - image data 
      */  
    std::vector<cv::Point2f> optiTrackFeature(double timestamp,const cv::Mat& img);
    
};