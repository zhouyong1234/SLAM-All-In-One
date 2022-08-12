//
// Created by tao on 7/22/21.
//

#include "gps_converter.h"

#include <rosbag/bag.h>
#include <sensor_msgs/NavSatFix.h>
#include <boost/filesystem.hpp>

namespace kaist2bag {

GpsConverter::GpsConverter(const std::string &dataset_dir, const std::string &save_dir,
                                       const std::string &topic)
        : Converter(dataset_dir, save_dir), topic_(topic) {
    bag_name_ = FilterSlash(topic_) + ".bag";
}

int GpsConverter::Convert() {
    CheckAndCreateSaveDir();

    boost::filesystem::path bag_file = boost::filesystem::path(save_dir_) / bag_name_;
    rosbag::Bag bag(bag_file.string(), rosbag::bagmode::Write);
    ROS_INFO("saving %s", bag_file.c_str());

    const std::string data_file = dataset_dir_ + "/" + default_data_file;
    FILE* fp = fopen(data_file.c_str(), "r");
    sensor_msgs::NavSatFix gps_data;
    int64_t stamp;
    double latitude, longitude, altitude;
    double cov[9];
    while (fscanf(fp,"%ld,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf\n",
                  &stamp,&latitude,&longitude,&altitude,
                  &cov[0],&cov[1],&cov[2],&cov[3],&cov[4],
                  &cov[5],&cov[6],&cov[7],&cov[8]) == 13) {
        gps_data.header.stamp.fromNSec(stamp);
        gps_data.header.frame_id = "gps";
        gps_data.latitude = latitude;
        gps_data.longitude = longitude;
        gps_data.altitude = altitude;
        for (int i = 0; i < 9; ++i) {
            gps_data.position_covariance[i] = cov[i];
        }

        bag.write(topic_, gps_data.header.stamp, gps_data);
    }
    bag.close();
    fclose(fp);
    ROS_INFO("done saving %s", bag_file.c_str());

    return 0;
}

} // namespace kaist2bag