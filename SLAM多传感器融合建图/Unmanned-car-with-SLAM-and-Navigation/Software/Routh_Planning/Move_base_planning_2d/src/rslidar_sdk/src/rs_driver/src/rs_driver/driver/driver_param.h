/*********************************************************************************************************************
Copyright (c) 2020 RoboSense
All rights reserved

By downloading, copying, installing or using the software you agree to this license. If you do not agree to this
license, do not download, install, copy or use the software.

License Agreement
For RoboSense LiDAR SDK Library
(3-clause BSD License)

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following
disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following
disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the names of the RoboSense, nor Suteng Innovation Technology, nor the names of other contributors may be used
to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*********************************************************************************************************************/

#pragma once
#include <string>
#include <rs_driver/common/common_header.h>
namespace robosense
{
namespace lidar
{
enum LidarType  ///< LiDAR type
{
  RS16 = 1,
  RS32,
  RSBP,
  RS128,
  RS80,
  RSHELIOS,
  RSM1 = 10
};

enum SplitFrameMode
{
  SPLIT_BY_ANGLE = 1,
  SPLIT_BY_FIXED_PKTS,
  SPLIT_BY_CUSTOM_PKTS
};

typedef struct RSCameraTriggerParam  ///< Camera trigger parameters
{
  std::map<double, std::string> trigger_map;  ///< Map stored the trigger angle and camera frame id
  void print() const                          
  {
    RS_INFO << "------------------------------------------------------" << RS_REND;
    RS_INFO << "             RoboSense Camera Trigger Parameters " << RS_REND;
    for (auto iter : trigger_map)
    {
      RS_INFOL << "camera_frame_id: " << iter.second << " trigger_angle : " << iter.first << RS_REND;
    }
    RS_INFO << "------------------------------------------------------" << RS_REND;
  }
} RSCameraTriggerParam;

typedef struct RSTransformParam  ///< The Point transform parameter
{
  float x = 0.0f;      ///< unit, m
  float y = 0.0f;      ///< unit, m
  float z = 0.0f;      ///< unit, m
  float roll = 0.0f;   ///< unit, radian
  float pitch = 0.0f;  ///< unit, radian
  float yaw = 0.0f;    ///< unit, radian
  void print() const   
  {
    RS_INFO << "------------------------------------------------------" << RS_REND;
    RS_INFO << "             RoboSense Transform Parameters " << RS_REND;
    RS_INFOL << "x: " << x << RS_REND;
    RS_INFOL << "y: " << y << RS_REND;
    RS_INFOL << "z: " << z << RS_REND;
    RS_INFOL << "roll: " << roll << RS_REND;
    RS_INFOL << "pitch: " << pitch << RS_REND;
    RS_INFOL << "yaw: " << yaw << RS_REND;
    RS_INFO << "------------------------------------------------------" << RS_REND;
  }
} RSTransformParam;

typedef struct RSDecoderParam  ///< LiDAR decoder parameter
{
  float max_distance = 200.0f;                                       ///< Max distance of point cloud range
  float min_distance = 0.2f;                                         ///< Minimum distance of point cloud range
  float start_angle = 0.0f;                                          ///< Start angle of point cloud
  float end_angle = 360.0f;                                          ///< End angle of point cloud
  SplitFrameMode split_frame_mode = SplitFrameMode::SPLIT_BY_ANGLE;  ///< 1: Split frames by cut_angle;
                                                                     ///< 2: Split frames by fixed number of packets;
                                                                     ///< 3: Split frames by custom number of packets (num_pkts_split)
  uint32_t num_pkts_split = 1;         ///< Number of packets in one frame, only be used when split_frame_mode=3
  float cut_angle = 0.0f;              ///< Cut angle(degree) used to split frame, only be used when split_frame_mode=1
  bool use_lidar_clock = false;        ///< true: use LiDAR clock as timestamp; false: use system clock as timestamp
  RSTransformParam transform_param;    ///< Used to transform points
  RSCameraTriggerParam trigger_param;  ///< Used to trigger camera
  void print() const                  
  {
    transform_param.print();
    trigger_param.print();
    RS_INFO << "------------------------------------------------------" << RS_REND;
    RS_INFO << "             RoboSense Decoder Parameters " << RS_REND;
    RS_INFOL << "max_distance: " << max_distance << RS_REND;
    RS_INFOL << "min_distance: " << min_distance << RS_REND;
    RS_INFOL << "start_angle: " << start_angle << RS_REND;
    RS_INFOL << "end_angle: " << end_angle << RS_REND;
    RS_INFOL << "use_lidar_clock: " << use_lidar_clock << RS_REND;
    RS_INFOL << "split_frame_mode: " << split_frame_mode << RS_REND;
    RS_INFOL << "num_pkts_split: " << num_pkts_split << RS_REND;
    RS_INFOL << "cut_angle: " << cut_angle << RS_REND;
    RS_INFO << "------------------------------------------------------" << RS_REND;
  }
} RSDecoderParam;

typedef struct RSInputParam  ///< The LiDAR input parameter
{
  std::string device_ip = "192.168.1.200";     ///< Ip of LiDAR
  std::string multi_cast_address = "0.0.0.0";  ///< Address of multicast
  uint16_t msop_port = 6699;                   ///< Msop packet port number
  uint16_t difop_port = 7788;                  ///< Difop packet port number
  bool read_pcap = false;          ///< true: The driver will process the pcap through pcap_path. false: The driver will
                                   ///< Get data from online LiDAR
  double pcap_rate = 1;            ///< Rate to read the pcap file
  bool pcap_repeat = true;         ///< true: The pcap bag will repeat play
  std::string pcap_path = "null";  ///< Absolute path of pcap file
  void print() const             
  {
    RS_INFO << "------------------------------------------------------" << RS_REND;
    RS_INFO << "             RoboSense Input Parameters " << RS_REND;
    RS_INFOL << "multi_cast_address: " << multi_cast_address << RS_REND;
    RS_INFOL << "msop_port: " << msop_port << RS_REND;
    RS_INFOL << "difop_port: " << difop_port << RS_REND;
    RS_INFOL << "read_pcap: " << read_pcap << RS_REND;
    RS_INFOL << "pcap_repeat: " << pcap_repeat << RS_REND;
    RS_INFOL << "pcap_path: " << pcap_path << RS_REND;
    RS_INFO << "------------------------------------------------------" << RS_REND;
  }
} RSInputParam;

typedef struct RSDriverParam  ///< The LiDAR driver parameter
{
  RSInputParam input_param;                ///< Input parameter
  RSDecoderParam decoder_param;            ///< Decoder parameter
  std::string angle_path = "null";         ///< Path of angle calibration files(angle.csv).Only used for internal debugging.
  std::string frame_id = "rslidar";        ///< The frame id of LiDAR message
  LidarType lidar_type = LidarType::RS16;  ///< Lidar type
  bool wait_for_difop = true;              ///< true: start sending point cloud until receive difop packet
  bool saved_by_rows = false;  ///< true: the output point cloud will be saved by rows (default is saved by columns)
  void print() const           
  {
    input_param.print();
    decoder_param.print();
    RS_INFO << "------------------------------------------------------" << RS_REND;
    RS_INFOL << "             RoboSense Driver Parameters " << RS_REND;
    RS_INFOL << "angle_path: " << angle_path << RS_REND;
    RS_INFOL << "frame_id: " << frame_id << RS_REND;
    RS_INFOL << "lidar_type: ";
    RS_INFO << lidarTypeToStr(lidar_type) << RS_REND;
    RS_INFOL << "------------------------------------------------------" << RS_REND;
  }
  static std::string lidarTypeToStr(const LidarType& type)
  {
    std::string str = "";
    switch (type)
    {
      case LidarType::RS16:
        str = "RS16";
        break;
      case LidarType::RS32:
        str = "RS32";
        break;
      case LidarType::RSBP:
        str = "RSBP";
        break;
      case LidarType::RS128:
        str = "RS128";
        break;
      case LidarType::RS80:
        str = "RS80";
        break;
      case LidarType::RSM1:
        str = "RSM1";
        break;
      case LidarType::RSHELIOS:
        str = "RSHELIOS";
        break;
      default:
        str = "ERROR";
        RS_ERROR << "RS_ERROR" << RS_REND;
    }
    return str;
  }
  static LidarType strToLidarType(const std::string& type)
  {
    if (type == "RS16")
    {
      return lidar::LidarType::RS16;
    }
    else if (type == "RS32")
    {
      return lidar::LidarType::RS32;
    }
    else if (type == "RSBP")
    {
      return lidar::LidarType::RSBP;
    }
    else if (type == "RS128")
    {
      return lidar::LidarType::RS128;
    }
    else if (type == "RS80")
    {
      return lidar::LidarType::RS80;
    }
    else if (type == "RSM1")
    {
      return lidar::LidarType::RSM1;
    }
    else if (type == "RSHELIOS")
    {
      return lidar::LidarType::RSHELIOS;
    }
    else
    {
      RS_ERROR << "Wrong lidar type: " << type << RS_REND;
      RS_ERROR << "Please setup the correct type: RS16, RS32, RSBP, RS128, RS80, RSM1, RSHELIOS" << RS_REND;
      exit(-1);
    }
  }
} RSDriverParam;
}  // namespace lidar
}  // namespace robosense
