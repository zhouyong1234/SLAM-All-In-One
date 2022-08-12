#include <rosbag/bag.h>
#include <std_msgs/Int32.h>
#include <ros/ros.h>
#include "altimeter_converter.h"
#include "encoder_converter.h"
#include "fog_converter.h"
#include "gps_converter.h"
#include "vrs_converter.h"
#include "imu_converter.h"
#include "velodyne_converter.h"
#include "sick_converter.h"
#include "stereo_converter.h"

using namespace kaist2bag;

enum SensorType {
    kAltimeter = 0,
    kEncoder,
    kFog,
    kGps,
    kVrs,
    kImu,
    kVelodyne,
    kSick,
    kStereo,
    kSensorTypeCount
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "kaist2bag");

    ros::NodeHandle nh;
    std::vector<bool> sensors(kSensorTypeCount, false);
    std::vector<std::string> sensor_names = {
        "altimeter",
        "encoder",
        "fog",
        "gps",
        "vrs",
        "imu",
        "velodyne",
        "sick",
        "stereo"
    };

    for (size_t i = 0; i < sensors.size(); ++i) {
        bool sensor_on = false;
        nh.getParam(sensor_names[i], sensor_on);
        sensors[i] = sensor_on;
    }

    std::string dataset;
    nh.getParam("dataset", dataset);
    std::string save_to;
    nh.getParam("save_to", save_to);
    double t0 = ros::Time::now().toSec();


    if(sensors[SensorType::kAltimeter]) {
        std::string altimeter_topic;
        nh.getParam("altimeter_topic", altimeter_topic);
        AltimeterConverter altimeter(dataset, save_to, altimeter_topic);
        altimeter.Convert();
    }
    double t1 = ros::Time::now().toSec();
    double altimeter_cost = t1 - t0;


    if(sensors[SensorType::kEncoder]) {
        std::string encoder_irp_topic, encoder_raw_topic;
        nh.getParam("encoder_irp_topic", encoder_irp_topic);
        nh.getParam("encoder_raw_topic", encoder_raw_topic);
        EncoderConverter encoder(dataset, save_to, encoder_irp_topic, encoder_raw_topic);
        encoder.Convert();
    }
    double t2 = ros::Time::now().toSec();
    double encoder_cost = t2 - t1;


    if(sensors[SensorType::kFog]) {
        std::string fog_topic;
        nh.getParam("fog_topic", fog_topic);
        FogConverter fog(dataset, save_to, fog_topic);
        fog.Convert();
    }
    double t3 = ros::Time::now().toSec();
    double fog_cost = t3 - t2;


    if(sensors[SensorType::kGps]) {
        std::string gps_topic;
        nh.getParam("gps_topic", gps_topic);
        GpsConverter gps(dataset, save_to, gps_topic);
        gps.Convert();
    }
    double t4 = ros::Time::now().toSec();
    double gps_cost = t4 - t3;


    if(sensors[SensorType::kVrs]) {
        std::string vrs_topic;
        nh.getParam("vrs_topic", vrs_topic);
        VrsConverter vrs(dataset, save_to, vrs_topic);
        vrs.Convert();
    }
    double t5 = ros::Time::now().toSec();
    double vrs_cost = t5 - t4;


    if(sensors[SensorType::kImu]) {
        std::string irp_topic, raw_topic, mag_topic;
        // nh.getParam("imu_irp_topic", irp_topic);
        nh.getParam("imu_raw_topic", raw_topic);
        // nh.getParam("imu_mag_topic", mag_topic);
        ImuConverter imu(dataset, save_to, raw_topic);
        // ImuConverter imu(dataset, save_to, irp_topic, raw_topic, mag_topic);
        imu.Convert();
    }
    double t6 = ros::Time::now().toSec();
    double imu_cost = t6 - t5;


    if(sensors[SensorType::kVelodyne]) {
        std::string vlp_left_topic, vlp_right_topic;
        nh.getParam("velodyne_left_topic", vlp_left_topic);
        nh.getParam("velodyne_right_topic", vlp_right_topic);
        VelodyneConverter vlp(dataset, save_to, vlp_left_topic, vlp_right_topic);
        vlp.Convert();
    }
    double t7 = ros::Time::now().toSec();
    double velodyne_cost = t7 - t6;


    if(sensors[SensorType::kSick]) {
        std::string sick_back_topic, sick_middle_topic;
        nh.getParam("sick_back_topic", sick_back_topic);
        nh.getParam("sick_middle_topic", sick_middle_topic);
        SickConverter sick(dataset, save_to, sick_back_topic, sick_middle_topic);
        sick.Convert();
    }
    double t8 = ros::Time::now().toSec();
    double sick_cost = t8 - t7;


    if(sensors[SensorType::kStereo]) {
        std::string stereo_left_topic, stereo_right_topic;
        nh.getParam("stereo_left_topic", stereo_left_topic);
        nh.getParam("stereo_right_topic", stereo_right_topic);
        StereoConverter stereo(dataset, save_to, stereo_left_topic, stereo_right_topic);
        stereo.Convert();
    }
    double t9 = ros::Time::now().toSec();
    double stereo_cost = t9 - t8;


    double all_cost = t9 - t0;
    ROS_INFO("altimeter_cost %f\n", altimeter_cost);
    ROS_INFO("encoder_cost %f\n", encoder_cost);
    ROS_INFO("fog_cost %f\n", fog_cost);
    ROS_INFO("gps_cost %f\n", gps_cost);
    ROS_INFO("vrs_cost %f\n", vrs_cost);
    ROS_INFO("imu_cost %f\n", imu_cost);
    ROS_INFO("velodyne_cost %f\n", velodyne_cost);
    ROS_INFO("sick_cost %f\n", sick_cost);
    ROS_INFO("stereo_cost %f\n", stereo_cost);
    ROS_INFO("all_cost %f\n", all_cost);

    return 0;
}