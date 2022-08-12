//
// Created by tao on 7/11/21.
//

#include "altimeter_converter.h"

#include <rosbag/bag.h>
#include <irp_sen_msgs/altimeter.h>
#include <boost/filesystem.hpp>

namespace kaist2bag {

AltimeterConverter::AltimeterConverter(const std::string &dataset_dir, const std::string &save_dir,
                                       const std::string &topic)
                                       : Converter(dataset_dir, save_dir), topic_(topic) {
    bag_name_ = FilterSlash(topic_) + ".bag";
}

int AltimeterConverter::Convert() {
    CheckAndCreateSaveDir();

    boost::filesystem::path bag_file = boost::filesystem::path(save_dir_) / bag_name_;
    rosbag::Bag bag(bag_file.string(), rosbag::bagmode::Write);
    ROS_INFO("saving %s", bag_file.c_str());

    const std::string data_file = dataset_dir_ + "/" + default_data_file;
    FILE* fp = fopen(data_file.c_str(), "r");
    irp_sen_msgs::altimeter altimeter_data;
    int64_t stamp;
    double altimeter_value;
    while (fscanf(fp, "%ld,%lf\n", &stamp, &altimeter_value) == 2) {
        altimeter_data.header.stamp.fromNSec(stamp);
        altimeter_data.header.frame_id = "altimeter";
        altimeter_data.data = altimeter_value;

        bag.write(topic_, altimeter_data.header.stamp, altimeter_data);
    }
    bag.close();
    fclose(fp);
    ROS_INFO("done saving %s", bag_file.c_str());

    return 0;
}

} // namespace kaist2bag