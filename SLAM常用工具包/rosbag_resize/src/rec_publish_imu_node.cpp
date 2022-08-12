#include "ros/ros.h"
#include <rosbag/bag.h>
#include "sensor_msgs/Imu.h"
#include <fstream>
#include <iostream>
#include <vector>

std::vector<std::string> StringSplit(std::string str,std::string pattern)
{
  std::string::size_type pos;
  std::vector<std::string> result;
  str+=pattern;
  int size=str.size();
 
  for(int i=0; i<size; i++)
  {
    pos=str.find(pattern,i);
    if(pos<size)
    {
      std::string s=str.substr(i,pos-i);
      result.push_back(s);
      i=pos+pattern.size()-1;
    }
  }
  return result;
}

ros::Time TimestampToRosTime(std::string timestamp)
{
    size_t len = timestamp.length();
    size_t secLen = len - 9;
    std::string sec_string = timestamp.substr(0, 10);
    std::string nsec_string = timestamp.substr(10,len);

    // std::cout << sec_string << ", " << nsec_string << std::endl;

    while(nsec_string.length() < 9){
        nsec_string += "0";
    }
    return ros::Time(std::stoi(sec_string),std::stoi(nsec_string));
}

double StringToDouble(std::string strData)
{
    return std::stod(strData);
}

int main(int argc, char** argv)
{
    // if (argc<2){
    //     std::cout<<"input imu path!"<<std::endl;
    //     return -1;
    // }

    ros::init(argc,argv, "imu_publisher");
    ros::NodeHandle nh("~");

    std::string imuFilePath;
    nh.getParam("imu_file",imuFilePath);
    std::cout<<"input imu path success: "<< imuFilePath <<std::endl;

    std::string raw_bag_file = "/home/touchair/kaist_urban_dataset/urban08/bag/imu_data_raw.bag";

    rosbag::Bag raw_bag(raw_bag_file, rosbag::bagmode::Write);


    ros::Publisher imuPub = nh.advertise<sensor_msgs::Imu>("/imu/data_raw", 1000);
    ros::Rate loopRate(500);

    std::fstream imuFile(imuFilePath);//argv[1]
    std::string readLine;
    sensor_msgs::Imu imuData;
    // std::getline(imuFile,readLine);
    while(ros::ok() && std::getline(imuFile,readLine)){
        // std::cout << readLine << std::endl;
        std::vector<std::string> lineSplit = StringSplit(readLine,",");

        // std::cout << lineSplit[0] << std::endl;

    
        // if(lineSplit.size()<6){
        //     continue;
        // }

        // sensor_msgs::Imu imuData;
        std::string timestamp = lineSplit[0];


        // std::cout << timestamp.length() << std::endl;


        // while(timestamp.length() < 14){
        //   timestamp += "0";
        // }

        imuData.header.stamp = TimestampToRosTime(timestamp);


        // std::cout << "------------------------------" << std::endl;
        // std::cout << timestamp << std::endl;
        // std::cout << "imu: " << TimestampToRosTime(timestamp) << std::endl;
        // std::cout << "------------------------------" << std::endl;

        imuData.header.frame_id = "imu";

        imuData.orientation.x = StringToDouble(lineSplit[1]);
        imuData.orientation.y = StringToDouble(lineSplit[2]);
        imuData.orientation.z = StringToDouble(lineSplit[3]);
        imuData.orientation.w = StringToDouble(lineSplit[4]);

        imuData.angular_velocity.x = StringToDouble(lineSplit[8]);
        imuData.angular_velocity.y = StringToDouble(lineSplit[9]);
        imuData.angular_velocity.z = StringToDouble(lineSplit[10]);

        imuData.linear_acceleration.x = StringToDouble(lineSplit[11]);
        imuData.linear_acceleration.y = StringToDouble(lineSplit[12]);
        imuData.linear_acceleration.z = StringToDouble(lineSplit[13]);

        imuData.orientation_covariance[0] = 3;
        imuData.orientation_covariance[4] = 3;
        imuData.orientation_covariance[8] = 3;
        imuData.angular_velocity_covariance[0] = 3;
        imuData.angular_velocity_covariance[4] = 3;
        imuData.angular_velocity_covariance[8] = 3;
        imuData.linear_acceleration_covariance[0] = 3;
        imuData.linear_acceleration_covariance[4] = 3;
        imuData.linear_acceleration_covariance[8] = 3;

        // raw_bag.write("/imu/data_raw", imuData.header.stamp, imuData);

        imuPub.publish(imuData);
        // ros::spinOnce();
        loopRate.sleep();
    }

    raw_bag.close();  
    imuFile.close();

    std::cout << "done saving: " << raw_bag_file << std::endl;

    return 0;
}