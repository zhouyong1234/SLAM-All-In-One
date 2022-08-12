#include "ros/ros.h"
#include <sensor_msgs/NavSatFix.h>
#include <fstream>
#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>

cv::RNG rng;


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
    std::string sec_string = timestamp.substr(0,10);
    std::string nsec_string = timestamp.substr(11, len);

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

    ros::init(argc,argv, "gps_publisher");
    ros::NodeHandle nh("~");

    std::string gpsFilePath;
    nh.getParam("gps_file",gpsFilePath);
    std::cout<<"input gps path success: "<< gpsFilePath <<std::endl;



    ros::Publisher gpsPub = nh.advertise<sensor_msgs::NavSatFix>("/android/fix", 20);
    ros::Rate loopRate(10);

    std::fstream gpsFile(gpsFilePath);//argv[1]
    std::string readLine;
    std::getline(gpsFile,readLine);

    // std::cout << "getline" << std::endl;

    while(ros::ok() && std::getline(gpsFile,readLine)){

        // std::cout<< readLine <<std::endl;

        std::vector<std::string> lineSplit = StringSplit(readLine,",");


        std::cout << lineSplit[0] << ", " << lineSplit[1] << ", " << lineSplit[2] << std::endl;

    
        if(lineSplit.size()<3){
            continue;
        }

        sensor_msgs::NavSatFix gpsData;
        std::string timestamp = lineSplit[0];
        gpsData.header.stamp = TimestampToRosTime(timestamp);


        // std::cout << "------------------------------" << std::endl;
        // std::cout << timestamp << std::endl;
        // std::cout << "gps: " << TimestampToRosTime(timestamp) << std::endl;
        // std::cout << "------------------------------" << std::endl;

        gpsData.header.frame_id = "gps";


        std::vector<std::string> xSplit = StringSplit(lineSplit[1], ".");
        std::vector<std::string> ySplit = StringSplit(lineSplit[2], ".");

        // std::cout << StringToDouble(xSplit[0]) << ", " << StringToDouble(ySplit[0]) << std::endl;

        if(StringToDouble(xSplit[0]) != 120.0 || StringToDouble(ySplit[0]) != 31.0 || StringToDouble(lineSplit[1]) == 0 || StringToDouble(lineSplit[2]) == 0)
        {
          std::cout << "longitude != 120, latitude != 31" << std::endl;
          gpsData.latitude = 31;
          gpsData.longitude = 120;
          // gpsData.altitude = StringToDouble(lineSplit[3]);
          gpsData.altitude = 0;

          gpsData.position_covariance[0] = 10000;
          gpsData.position_covariance[4] = 10000;
          gpsData.position_covariance[8] = 10000;
        }
        else
        {
          gpsData.latitude = StringToDouble(lineSplit[2]);
          gpsData.longitude = StringToDouble(lineSplit[1]);
          // gpsData.altitude = StringToDouble(lineSplit[3]);
          gpsData.altitude = 0;



          gpsData.position_covariance[0] = 10 + rng.gaussian(1.0);
          gpsData.position_covariance[4] = 10 + rng.gaussian(1.0);
          gpsData.position_covariance[8] = 10 + rng.gaussian(1.0);
        }


        
        gpsData.position_covariance_type = 1;

        gpsPub.publish(gpsData);
        ros::spinOnce();
        loopRate.sleep();
    }

    gpsFile.close();
    return 0;
}