#include <opencv/cv.hpp>
#include "../include/feature_tracker.hpp"


/**
 * FeatureTrack main function, track 2 images get the disparities
 * @param imageRef
 * @param imageCur
 * @param pxRef
 * @param pxCur
 * @param disparities
 */
void FeatureTracker::track(cv::Mat imageRef, cv::Mat imageCur, std::vector<cv::Point2f> &pxRef,
                           std::vector<cv::Point2f> &pxCur, std::vector<double> &disparities) {

    const int klt_win_size = 21;
    const int klt_max_iter = 30;
    const double klt_eps = 0.001;
    std::vector<uchar> status;
    std::vector<float> error;
    std::vector<float> min_eig_vec;
    cv::TermCriteria termCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, klt_max_iter, klt_eps);
    cv::calcOpticalFlowPyrLK(imageRef, imageCur,
                             pxRef, pxCur,
                             status, error,
                             cv::Size2i(klt_win_size, klt_win_size),
                             4, termCriteria, 0);

    auto px_ref_it = pxRef.begin();
    auto px_cur_it = pxCur.begin();
    disparities.clear(); disparities.reserve(pxCur.size());
    for (size_t i = 0; px_ref_it != pxRef.end(); ++i)
    {
        if (!status[i])
        {
            px_ref_it = pxRef.erase(px_ref_it);
            px_cur_it = pxCur.erase(px_cur_it);
            continue;
        }
        disparities.push_back(norm(cv::Point2d(px_ref_it->x - px_cur_it->x, px_ref_it->y - px_cur_it->y)));
        ++px_ref_it;
        ++px_cur_it;
    }
}


/**
 * This function detect feature of an image, currently using
 * FAST algorithm to detect.
 * Then convert KeyPoints into Vector of Points
 * @param image
 * @param pxVec
 */
void FeatureTracker::detect(cv::Mat image, std::vector<cv::Point2f> &pxVec) {
    std::vector<cv::KeyPoint> keyPoints;

    int fastThreshold = 30;
    bool nms = true;
    cv::FAST(image, keyPoints, fastThreshold, nms);

    //-- 绘制特征点
	cv::Mat img_keypoints; 
	cv::drawKeypoints(image, keyPoints, img_keypoints, cv::Scalar(0, 255, 0), cv::DrawMatchesFlags::DEFAULT); 
	//-- 显示特征点
	cv::imshow("Keypoints", img_keypoints);

    cv::KeyPoint::convert(keyPoints, pxVec);
}