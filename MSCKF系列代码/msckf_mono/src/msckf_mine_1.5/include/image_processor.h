#ifndef IMAGE_PROCESSOR_H
#define IMAGE_PROCESSOR_H
#include "common_include.h"
#include "config.h"

namespace MSCKF_MINE
{
class ImageProcessor
{
public:
    ImageProcessor();
    ~ImageProcessor();

    // Initialize the object.
    bool initialize();

    typedef boost::shared_ptr<ImageProcessor> Ptr;
    typedef boost::shared_ptr<const ImageProcessor> ConstPtr;

    /*
     * @brief ProcessorConfig Configuration parameters for
     *    feature detection and tracking.
     */
    struct ProcessorConfig
    {
        int grid_row;
        int grid_col;
        int grid_min_feature_num;
        int grid_max_feature_num;

        int pyramid_levels;
        int patch_size;
        int fast_threshold;
        int max_iteration;
        double track_precision;
        double ransac_threshold;
        double stereo_threshold;
    };

    /*
     * @brief FeatureIDType An alias for unsigned long long int.
     */
    typedef unsigned long long int FeatureIDType;

    /*
     * @brief FeatureMetaData Contains necessary information
     *    of a feature for easy access.
     */
    struct FeatureMetaData {
      FeatureIDType id;
      float response;
      int lifetime;
      cv::Point2f cam0_point;
    };

    /*
     * @brief GridFeatures Organize features based on the grid
     *    they belong to. Note that the key is encoded by the
     *    grid index.
     */
    typedef std::map<int, std::vector<FeatureMetaData> > GridFeatures;

    /*
     * @brief keyPointCompareByResponse
     *    Compare two keypoints based on the response.
     */
    static bool keyPointCompareByResponse(
        const cv::KeyPoint& pt1,
        const cv::KeyPoint& pt2) {
      // Keypoint with higher response will be at the
      // beginning of the vector.
      return pt1.response > pt2.response;
    }
    /*
     * @brief featureCompareByResponse
     *    Compare two features based on the response.
     */
    static bool featureCompareByResponse(
        const FeatureMetaData& f1,
        const FeatureMetaData& f2) {
      // Features with higher response will be at the
      // beginning of the vector.
      return f1.response > f2.response;
    }
    /*
     * @brief featureCompareByLifetime
     *    Compare two features based on the lifetime.
     */
    static bool featureCompareByLifetime(
        const FeatureMetaData& f1,
        const FeatureMetaData& f2) {
      // Features with longer lifetime will be at the
      // beginning of the vector.
      return f1.lifetime > f2.lifetime;
    }

    /*
     * @brief loadParameters
     *    Load parameters from the parameter server.
     */
    bool loadParameters();


    /*
     * @initializeFirstFrame
     *    Initialize the image processing sequence, which is
     *    bascially detect new features on the first set of
     *    stereo images.
     */
    void initializeFirstFrame();

    /*
     * @brief trackFeatures
     *    Tracker features on the newly received stereo images.
     */
    void trackFeatures();

    /*
     * @addNewFeatures
     *    Detect new features on the image to ensure that the
     *    features are uniformly distributed on the image.
     */
    void addNewFeatures();

    /*
     * @brief pruneGridFeatures
     *    Remove some of the features of a grid in case there are
     *    too many features inside of that grid, which ensures the
     *    number of features within each grid is bounded.
     */
    void pruneGridFeatures();

    /*
     * @brief drawFeaturesStereo
     *    Draw tracked and newly detected features on the
     *    stereo images.
     */
    void drawFeaturesStereo();

    /*
     * @brief createImagePyramids
     *    Create image pyramids used for klt tracking.
     */
    void createImagePyramids();


    // Indicate if this is the first image message.
    bool is_first_img;

    // ID for the next new feature.
    FeatureIDType next_feature_id;

    // Feature detector
    ProcessorConfig processor_config;
    cv::Ptr<cv::Feature2D> detector_ptr;

    // IMU message buffer.
    std::vector<sensor_msgs::Imu> imu_msg_buffer;

    // Camera calibration parameters
    std::string cam0_distortion_model;
    cv::Vec2i cam0_resolution;
    cv::Vec4d cam0_intrinsics;
    cv::Vec4d cam0_distortion_coeffs;

    // Take a vector from cam0 frame to the IMU frame.
    cv::Matx33d R_cam0_imu;
    cv::Vec3d t_cam0_imu;

    // Previous and current images
    cv_bridge::CvImageConstPtr cam0_prev_img_ptr;
    cv_bridge::CvImageConstPtr cam0_curr_img_ptr;
    cv_bridge::CvImageConstPtr cam1_curr_img_ptr;

    // Pyramids for previous and current image
    std::vector<cv::Mat> prev_cam0_pyramid_;
    std::vector<cv::Mat> curr_cam0_pyramid_;
    std::vector<cv::Mat> curr_cam1_pyramid_;

    // Features in the previous and current image.
    boost::shared_ptr<GridFeatures> prev_features_ptr;
    boost::shared_ptr<GridFeatures> curr_features_ptr;

    // Number of features after each outlier removal step.
    int before_tracking;
    int after_tracking;
    int after_matching;
    int after_ransac;

};

typedef ImageProcessor::Ptr ImageProcessorPtr;
typedef ImageProcessor::ConstPtr ImageProcessorConstPtr;

}
#endif // IMAGE_PROCESSOR_H
