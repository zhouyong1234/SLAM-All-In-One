//
// Created by tao on 7/28/21.
//

#include "stereo_converter.h"

#include <rosbag/bag.h>
#include <sensor_msgs/Image.h>
#include <boost/filesystem.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
// #include <opencv2/imgcodecs/legacy/constants_c.h>
#include <cv_bridge/cv_bridge.h>

namespace kaist2bag {

StereoConverter::StereoConverter(const std::string &dataset_dir, const std::string &save_dir,
                                     const std::string &left_topic, const std::string& right_topic)
        : Converter(dataset_dir, save_dir), left_topic_(left_topic), right_topic_(right_topic) {
    left_bag_name_ = FilterSlash(left_topic_) + ".bag";
    right_bag_name_ = FilterSlash(right_topic_) + ".bag";
}

int StereoConverter::Convert() {
    CheckAndCreateSaveDir();

    boost::filesystem::path left_bag_file = boost::filesystem::path(save_dir_) / left_bag_name_;
    boost::filesystem::path right_bag_file = boost::filesystem::path(save_dir_) / right_bag_name_;

    const std::string stamp_file = dataset_dir_ + "/" + default_stamp_file;
    const std::string left_data_dir = dataset_dir_ + "/" + default_left_data_dir;
    const std::string right_data_dir = dataset_dir_ + "/" + default_right_data_dir;

    ROS_INFO("saving %s", left_bag_file.c_str());
    Convert(stamp_file, left_data_dir, left_bag_file.string(), left_topic_, "/stereo/left");
    ROS_INFO("done saving %s", left_bag_file.c_str());
    ROS_INFO("saving %s", right_bag_file.c_str());
    Convert(stamp_file, right_data_dir, right_bag_file.string(), right_topic_, "/stereo/right");
    ROS_INFO("done saving %s", right_bag_file.c_str());

    return 0;
}

void StereoConverter::Convert(const std::string& stamp_file, const std::string& data_dir, const std::string& bag_file,
                                const std::string& topic, const std::string& frame_id) {
    rosbag::Bag bag(bag_file, rosbag::bagmode::Write);
    bag.setChunkThreshold(768*1024);
    bag.setCompression(rosbag::compression::BZ2);

    FILE* fp = fopen(stamp_file.c_str(), "r");
    int64_t stamp;
    std::vector<int64_t> all_stamps;
    while (fscanf(fp, "%ld\n", &stamp) == 1) {
        all_stamps.push_back(stamp);
    }
    fclose(fp);

    size_t total = all_stamps.size();
    for (size_t i = 0; i < all_stamps.size(); ++i) {
        std::string st = std::to_string(all_stamps[i]);
        std::string frame_file = data_dir + "/" + st + ".png";
        ROS_INFO("converting %s\n", frame_file.c_str());
        if (!boost::filesystem::exists(frame_file)) {
            ROS_WARN("%s not exist\n", frame_file.c_str());
            continue;
        }
        cv::Mat img = cv::imread(frame_file, CV_LOAD_IMAGE_ANYDEPTH);
        cv_bridge::CvImage img_msg;
        img_msg.header.stamp.fromNSec(all_stamps[i]);
        img_msg.header.frame_id = frame_id;
        img_msg.encoding = sensor_msgs::image_encodings::BAYER_BGGR8;
        img_msg.image = img;
        sensor_msgs::Image msg;
        img_msg.toImageMsg(msg);
        bag.write(topic, msg.header.stamp, msg);

        ROS_INFO("done converting %s\n", frame_file.c_str());
        ROS_INFO("total %lu, already convert %lu, remain %lu\n", total, i + 1, total - i - 1);
    }
    bag.close();
}

} // namespace kaist2bag