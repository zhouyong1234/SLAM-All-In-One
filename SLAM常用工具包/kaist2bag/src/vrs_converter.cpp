//
// Created by tao on 7/22/21.
//

#include "vrs_converter.h"

#include <rosbag/bag.h>
#include <sensor_msgs/NavSatFix.h>
#include <irp_sen_msgs/vrs.h>
#include <boost/filesystem.hpp>

namespace kaist2bag {

VrsConverter::VrsConverter(const std::string &dataset_dir, const std::string &save_dir,
                                       const std::string &topic)
        : Converter(dataset_dir, save_dir), topic_(topic) {
    bag_name_ = FilterSlash(topic_) + ".bag";
}

int VrsConverter::Convert() {
    CheckAndCreateSaveDir();

    boost::filesystem::path bag_file = boost::filesystem::path(save_dir_) / bag_name_;
    rosbag::Bag bag(bag_file.string(), rosbag::bagmode::Write);
    ROS_INFO("saving %s", bag_file.c_str());

    const std::string data_file = dataset_dir_ + "/" + default_data_file;
    FILE* fp = fopen(data_file.c_str(), "r");
    sensor_msgs::NavSatFix gps_data;
    double cov[9];

    // irp_sen_msgs::vrs vrs_data;
    int64_t stamp;
    double latitude, longitude, altitude, altitude_orthometric;
    double x_coordinate, y_coordinate, horizental_precision, lat_std, lon_std, altitude_std,
           heading_magnet, speed_knot, speed_km;
    int fix_state, number_of_sat, heading_valid;
    char GNVTG_mode;
    while (fscanf(fp,"%ld,%lf,%lf,%lf,%lf,%lf,%d,%d,%lf,%lf,%lf,%lf,%d,%lf,%lf,%lf,%c,%lf\n",
                  &stamp,&latitude,&longitude,&x_coordinate,&y_coordinate,&altitude,&fix_state,
                  &number_of_sat,&horizental_precision,&lat_std,&lon_std,&altitude_std,
                  &heading_valid,&heading_magnet,&speed_knot,&speed_km,&GNVTG_mode) == 17) {

        
        gps_data.header.stamp.fromNSec(stamp);
        gps_data.header.frame_id = "vrs_gps";
        gps_data.latitude = latitude;
        gps_data.longitude = longitude;
        gps_data.altitude = altitude;
        gps_data.position_covariance[0] = lat_std;
        gps_data.position_covariance[4] = lon_std;
        gps_data.position_covariance[8] = altitude_std;

        bag.write(topic_, gps_data.header.stamp, gps_data);

        // vrs_data.header.stamp.fromNSec(stamp);
        // vrs_data.header.frame_id = "altimeter";
        // vrs_data.latitude = latitude;
        // vrs_data.altitude_orthometric = altitude_orthometric;
        // vrs_data.longitude = longitude;
        // vrs_data.x_coordinate = x_coordinate;
        // vrs_data.y_coordinate = y_coordinate;
        // vrs_data.altitude = altitude;
        // vrs_data.fix_state = fix_state;
        // if (fix_state == 1) vrs_data.fix_state_str = "normal";
        // if (fix_state == 4) vrs_data.fix_state_str = "fix";
        // if (fix_state == 5) vrs_data.fix_state_str = "float";
        // vrs_data.number_of_sat = number_of_sat;
        // vrs_data.horizental_precision = horizental_precision;
        // vrs_data.lat_std = lat_std;
        // vrs_data.lon_std = lon_std;
        // vrs_data.altitude_std = altitude_std;
        // vrs_data.heading_valid = heading_valid;
        // vrs_data.heading_magnet = heading_magnet;
        // vrs_data.speed_knot = speed_knot;
        // vrs_data.speed_km = speed_km;
        // vrs_data.GNVTG_mode = GNVTG_mode;

        // bag.write(topic_, vrs_data.header.stamp, vrs_data);
    }
    bag.close();
    fclose(fp);
    ROS_INFO("done saving %s", bag_file.c_str());

    return 0;
}

} // namespace kaist2bag
