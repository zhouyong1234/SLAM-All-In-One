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
#include <rs_driver/driver/lidar_driver_impl.hpp>

namespace robosense
{
namespace lidar
{
/**
 * @brief This is the RoboSense LiDAR driver interface class
 */
template <typename PointT>
class LidarDriver
{
public:
  /**
   * @brief Constructor, instanciate the driver pointer
   */
  LidarDriver():driver_ptr_(std::make_shared<LidarDriverImpl<PointT>>())
  {
  }

  /**
   * @brief Deconstructor, stop all threads
   */
  ~LidarDriver()
  {
    stop();
  }

  /**
   * @brief The initialization function, used to set up parameters and instance objects,
   *        used when get packets from online lidar or pcap
   * @param param The custom struct RSDriverParam
   * @return If successful, return true; else return false
   */
  inline bool init(const RSDriverParam& param)
  {
    return driver_ptr_->init(param);
  }

  /**
   * @brief The initialization function which only initialize decoder(not include input module). If lidar packets are
   * from ROS or other ways excluding online lidar and pcap, call this function to initialize instead of calling init()
   * @param param The custom struct RSDriverParam
   */
  inline void initDecoderOnly(const RSDriverParam& param)
  {
    driver_ptr_->initDecoderOnly(param);
  }

  /**
   * @brief Start the thread to receive and decode packets
   * @return If successful, return true; else return false
   */
  inline bool start()
  {
    return driver_ptr_->start();
  }

  /**
   * @brief Stop all threads
   */
  inline void stop()
  {
    driver_ptr_->stop();
  }

  /**
   * @brief Register the lidar point cloud callback function to driver. When point cloud is ready, this function will be
   * called
   * @param callback The callback function
   */
  inline void regRecvCallback(const std::function<void(const PointCloudMsg<PointT>&)>& callback)
  {
    driver_ptr_->regRecvCallback(callback);
  }

  /**
   * @brief Register the lidar scan message callback function to driver.When lidar scan message is ready, this function
   * will be called
   * @param callback The callback function
   */
  inline void regRecvCallback(const std::function<void(const ScanMsg&)>& callback)
  {
    driver_ptr_->regRecvCallback(callback);
  }

  /**
   * @brief Register the lidar difop packet message callback function to driver. When lidar difop packet message is
   * ready, this function will be called
   * @param callback The callback function
   */
  inline void regRecvCallback(const std::function<void(const PacketMsg&)>& callback)
  {
    driver_ptr_->regRecvCallback(callback);
  }

  /**
   * @brief Register the camera trigger message callback function to driver. When trigger message is ready, this
   * function will be called
   * @param callback The callback function
   */
  inline void regRecvCallback(const std::function<void(const CameraTrigger&)>& callback)
  {
    driver_ptr_->regRecvCallback(callback);
  }

  /**
   * @brief Register the exception message callback function to driver. When error occurs, this function will be called
   * @param callback The callback function
   */
  inline void regExceptionCallback(const std::function<void(const Error&)>& callback)
  {
    driver_ptr_->regExceptionCallback(callback);
  }

  /**
   * @brief Get the current lidar temperature
   * @param input_temperature The variable to store lidar temperature
   * @return if get temperature successfully, return true; else return false
   */
  inline bool getLidarTemperature(double& input_temperature)
  {
    return driver_ptr_->getLidarTemperature(input_temperature);
  }

  /**
   * @brief Decode lidar scan messages to point cloud
   * @note This function will only work after decodeDifopPkt is called unless wait_for_difop is set to false
   * @param pkt_scan_msg The lidar scan message
   * @param point_cloud_msg The output point cloud message
   * @return if decode successfully, return true; else return false
   */
  inline bool decodeMsopScan(const ScanMsg& pkt_scan_msg, PointCloudMsg<PointT>& point_msg)
  {
    return driver_ptr_->decodeMsopScan(pkt_scan_msg, point_msg);
  }

  /**
   * @brief Decode lidar difop messages
   * @param pkt_msg The lidar difop packet
   */
  inline void decodeDifopPkt(const PacketMsg& pkt_msg)
  {
    driver_ptr_->decodeDifopPkt(pkt_msg);
  }

private:
  std::shared_ptr<LidarDriverImpl<PointT>> driver_ptr_;  ///< The driver pointer
};

}  // namespace lidar
}  // namespace robosense
