//
// Created by tao on 7/21/21.
//

#include "fog_converter.h"

#include <rosbag/bag.h>
#include <irp_sen_msgs/fog_3axis.h>
#include <boost/filesystem.hpp>

namespace kaist2bag {

FogConverter::FogConverter(const std::string &dataset_dir, const std::string &save_dir,
                                       const std::string &topic)
        : Converter(dataset_dir, save_dir), topic_(topic) {
    bag_name_ = FilterSlash(topic_) + ".bag";
}

int FogConverter::Convert() {
    CheckAndCreateSaveDir();

    boost::filesystem::path bag_file = boost::filesystem::path(save_dir_) / bag_name_;
    rosbag::Bag bag(bag_file.string(), rosbag::bagmode::Write);
    ROS_INFO("saving %s", bag_file.c_str());

    const std::string data_file = dataset_dir_ + "/" + default_data_file;
    FILE* fp = fopen(data_file.c_str(), "r");
    irp_sen_msgs::fog_3axis fog_data;
    int64_t stamp;
    float roll, pitch, yaw;
    while (fscanf(fp, "%ld,%f,%f,%f\n", &stamp, &roll, &pitch, &yaw) == 4) {
        fog_data.header.stamp.fromNSec(stamp);
        fog_data.header.frame_id = "dsp1760";
        fog_data.d_roll = roll;
        fog_data.d_pitch = pitch;
        fog_data.d_yaw = yaw;

        bag.write(topic_, fog_data.header.stamp, fog_data);
    }
    bag.close();
    fclose(fp);
    ROS_INFO("done saving %s", bag_file.c_str());

    return 0;
}

} // namespace kaist2bag