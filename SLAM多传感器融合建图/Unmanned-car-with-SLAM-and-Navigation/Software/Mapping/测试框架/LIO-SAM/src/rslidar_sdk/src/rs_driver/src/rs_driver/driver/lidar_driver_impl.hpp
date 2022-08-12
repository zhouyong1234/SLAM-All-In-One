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
#include <rs_driver/msg/point_cloud_msg.h>
#include <rs_driver/msg/packet_msg.h>
#include <rs_driver/msg/scan_msg.h>
#include <rs_driver/utility/lock_queue.h>
#include <rs_driver/utility/thread_pool.hpp>
#include <rs_driver/utility/time.h>
#include <rs_driver/common/error_code.h>
#include <rs_driver/driver/input.hpp>
#include <rs_driver/driver/decoder/decoder_factory.hpp>
constexpr size_t MAX_PACKETS_BUFFER_SIZE = 100000;
namespace robosense
{
namespace lidar
{
template <typename T_Point>
class LidarDriverImpl
{
public:
  LidarDriverImpl();
  ~LidarDriverImpl();
  bool init(const RSDriverParam& param);
  void initDecoderOnly(const RSDriverParam& param);
  bool start();
  void stop();
  void regRecvCallback(const std::function<void(const PointCloudMsg<T_Point>&)>& callback);
  void regRecvCallback(const std::function<void(const ScanMsg&)>& callback);
  void regRecvCallback(const std::function<void(const PacketMsg&)>& callback);
  void regRecvCallback(const std::function<void(const CameraTrigger&)>& callback);
  void regExceptionCallback(const std::function<void(const Error&)>& callback);
  bool getLidarTemperature(double& input_temperature);
  bool decodeMsopScan(const ScanMsg& scan_msg, PointCloudMsg<T_Point>& point_cloud_msg);
  void decodeDifopPkt(const PacketMsg& msg);

private:
  void runCallBack(const ScanMsg& msg);
  void runCallBack(const PacketMsg& msg);
  void runCallBack(const PointCloudMsg<T_Point>& msg);
  void reportError(const Error& error);
  void msopCallback(const PacketMsg& msg);
  void difopCallback(const PacketMsg& msg);
  void processMsop();
  void processDifop();
  void localCameraTriggerCallback(const CameraTrigger& msg);
  void initPointCloudTransFunc();
  void setScanMsgHeader(ScanMsg& msg);
  void setPointCloudMsgHeader(PointCloudMsg<T_Point>& msg);

private:
  Queue<PacketMsg> msop_pkt_queue_;
  Queue<PacketMsg> difop_pkt_queue_;
  std::vector<std::function<void(const ScanMsg&)>> msop_pkt_cb_vec_;
  std::vector<std::function<void(const PacketMsg&)>> difop_pkt_cb_vec_;
  std::vector<std::function<void(const PointCloudMsg<T_Point>&)>> point_cloud_cb_vec_;
  std::vector<std::function<void(const CameraTrigger&)>> camera_trigger_cb_vec_;
  std::vector<std::function<void(const Error&)>> excb_;
  std::shared_ptr<std::thread> lidar_thread_ptr_;
  std::shared_ptr<DecoderBase<T_Point>> lidar_decoder_ptr_;
  std::shared_ptr<Input> input_ptr_;
  std::shared_ptr<ScanMsg> scan_ptr_;
  std::shared_ptr<ThreadPool> thread_pool_ptr_;
  bool init_flag_;
  bool start_flag_;
  bool difop_flag_;
  uint32_t point_cloud_seq_;
  uint32_t scan_seq_;
  uint32_t ndifop_count_;
  RSDriverParam driver_param_;
  std::function<typename PointCloudMsg<T_Point>::PointCloudPtr(const typename PointCloudMsg<T_Point>::PointCloudPtr,
                                                               const size_t& height)>
      point_cloud_transform_func_;
  typename PointCloudMsg<T_Point>::PointCloudPtr point_cloud_ptr_;
};

template <typename T_Point>
inline LidarDriverImpl<T_Point>::LidarDriverImpl()
  : init_flag_(false), start_flag_(false), difop_flag_(false), point_cloud_seq_(0), scan_seq_(0), ndifop_count_(0)
{
  thread_pool_ptr_ = std::make_shared<ThreadPool>();
  point_cloud_ptr_ = std::make_shared<typename PointCloudMsg<T_Point>::PointCloud>();
  scan_ptr_ = std::make_shared<ScanMsg>();
}

template <typename T_Point>
inline LidarDriverImpl<T_Point>::~LidarDriverImpl()
{
  stop();
  thread_pool_ptr_.reset();
  input_ptr_.reset();
}

template <typename T_Point>
inline bool LidarDriverImpl<T_Point>::init(const RSDriverParam& param)
{
  if (init_flag_)
  {
    return false;
  }
  driver_param_ = param;
  input_ptr_ = std::make_shared<Input>(driver_param_.lidar_type, driver_param_.input_param,
                                       std::bind(&LidarDriverImpl<T_Point>::reportError, this, std::placeholders::_1));
  input_ptr_->regRecvMsopCallback(std::bind(&LidarDriverImpl<T_Point>::msopCallback, this, std::placeholders::_1));
  input_ptr_->regRecvDifopCallback(std::bind(&LidarDriverImpl<T_Point>::difopCallback, this, std::placeholders::_1));
  if (!input_ptr_->init())
  {
    return false;
  }
  lidar_decoder_ptr_ = DecoderFactory<T_Point>::createDecoder(driver_param_);
  lidar_decoder_ptr_->regRecvCallback(
      std::bind(&LidarDriverImpl<T_Point>::localCameraTriggerCallback, this, std::placeholders::_1));
  init_flag_ = true;
  initPointCloudTransFunc();
  return true;
}

template <typename T_Point>
inline void LidarDriverImpl<T_Point>::initDecoderOnly(const RSDriverParam& param)
{
  if (init_flag_)
  {
    return;
  }
  driver_param_ = param;
  lidar_decoder_ptr_ = DecoderFactory<T_Point>::createDecoder(driver_param_);
  lidar_decoder_ptr_->regRecvCallback(
      std::bind(&LidarDriverImpl<T_Point>::localCameraTriggerCallback, this, std::placeholders::_1));
  init_flag_ = true;
  initPointCloudTransFunc();
}

template <typename T_Point>
inline void LidarDriverImpl<T_Point>::initPointCloudTransFunc()
{
  if (driver_param_.saved_by_rows)
  {
    point_cloud_transform_func_ = [](const typename PointCloudMsg<T_Point>::PointCloudPtr input_ptr,
                                     const size_t& height) -> typename PointCloudMsg<T_Point>::PointCloudPtr
    {
      typename PointCloudMsg<T_Point>::PointCloudPtr row_major_ptr =
          std::make_shared<typename PointCloudMsg<T_Point>::PointCloud>();
      row_major_ptr->resize(input_ptr->size());
      size_t width = input_ptr->size() / height;
      for (int i = 0; i < static_cast<int>(height); i++)
      {
        for (int j = 0; j < static_cast<int>(width); j++)
        {
          row_major_ptr->at(i * width + j) = input_ptr->at(j * height + i);
        }
      }
      return row_major_ptr;
    };
  }
  else
  {
    point_cloud_transform_func_ = [](const typename PointCloudMsg<T_Point>::PointCloudPtr input_ptr,
                                     const size_t& height) -> typename PointCloudMsg<T_Point>::PointCloudPtr
    {
      return input_ptr;
    };
  }
}

template <typename T_Point>
inline bool LidarDriverImpl<T_Point>::start()
{
  if (start_flag_ || input_ptr_ == nullptr)
  {
    return false;
  }
  start_flag_ = true;
  return input_ptr_->start();
}

template <typename T_Point>
inline void LidarDriverImpl<T_Point>::stop()
{
  if (input_ptr_ != nullptr)
  {
    input_ptr_->stop();
  }
  start_flag_ = false;
  if (!msop_pkt_cb_vec_.empty() || !difop_pkt_cb_vec_.empty())
  {
    std::this_thread::sleep_for(std::chrono::microseconds(10));
  }
}

template <typename T_Point>
inline void
LidarDriverImpl<T_Point>::regRecvCallback(const std::function<void(const PointCloudMsg<T_Point>&)>& callback)
{
  point_cloud_cb_vec_.emplace_back(callback);
}

template <typename T_Point>
inline void LidarDriverImpl<T_Point>::regRecvCallback(const std::function<void(const ScanMsg&)>& callback)
{
  msop_pkt_cb_vec_.emplace_back(callback);
}

template <typename T_Point>
inline void LidarDriverImpl<T_Point>::regRecvCallback(const std::function<void(const PacketMsg&)>& callback)
{
  difop_pkt_cb_vec_.emplace_back(callback);
}

template <typename T_Point>
inline void LidarDriverImpl<T_Point>::regRecvCallback(const std::function<void(const CameraTrigger&)>& callback)
{
  camera_trigger_cb_vec_.emplace_back(callback);
}

template <typename T_Point>
inline void LidarDriverImpl<T_Point>::regExceptionCallback(const std::function<void(const Error&)>& callback)
{
  excb_.emplace_back(callback);
}

template <typename T_Point>
inline bool LidarDriverImpl<T_Point>::getLidarTemperature(double& input_temperature)
{
  if (lidar_decoder_ptr_ != nullptr)
  {
    input_temperature = lidar_decoder_ptr_->getLidarTemperature();
    return true;
  }
  return false;
}

template <typename T_Point>
inline bool LidarDriverImpl<T_Point>::decodeMsopScan(const ScanMsg& scan_msg, PointCloudMsg<T_Point>& point_cloud_msg)
{
  typename PointCloudMsg<T_Point>::PointCloudPtr output_point_cloud_ptr =
      std::make_shared<typename PointCloudMsg<T_Point>::PointCloud>();
  if (!difop_flag_ && driver_param_.wait_for_difop)
  {
    ndifop_count_++;
    if (ndifop_count_ > 20)
    {
      reportError(Error(ERRCODE_NODIFOPRECV));
      ndifop_count_ = 0;
    }
    point_cloud_msg.point_cloud_ptr = output_point_cloud_ptr;
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    return false;
  }

  std::vector<std::vector<T_Point>> pointcloud_one_frame;
  int height = 1;
  pointcloud_one_frame.resize(scan_msg.packets.size());
  for (int i = 0; i < static_cast<int>(scan_msg.packets.size()); i++)
  {
    std::vector<T_Point> pointcloud_one_packet;
    RSDecoderResult ret =
        lidar_decoder_ptr_->processMsopPkt(scan_msg.packets[i].packet.data(), pointcloud_one_packet, height);
    switch (ret)
    {
      case RSDecoderResult::DECODE_OK:
      case RSDecoderResult::FRAME_SPLIT:
        pointcloud_one_frame[i] = std::move(pointcloud_one_packet);
        break;
      case RSDecoderResult::WRONG_PKT_HEADER:
        reportError(Error(ERRCODE_WRONGPKTHEADER));
        break;
      case RSDecoderResult::PKT_NULL:
        reportError(Error(ERRCODE_PKTNULL));
        break;
      default:
        break;
    }
  }
  for (auto iter : pointcloud_one_frame)
  {
    output_point_cloud_ptr->insert(output_point_cloud_ptr->end(), iter.begin(), iter.end());
  }
  point_cloud_msg.point_cloud_ptr = point_cloud_transform_func_(output_point_cloud_ptr, height);
  point_cloud_msg.height = height;
  point_cloud_msg.width = point_cloud_msg.point_cloud_ptr->size() / point_cloud_msg.height;
  setPointCloudMsgHeader(point_cloud_msg);
  point_cloud_msg.timestamp = scan_msg.timestamp;
  if (point_cloud_msg.point_cloud_ptr->size() == 0)
  {
    reportError(Error(ERRCODE_ZEROPOINTS));
    return false;
  }
  return true;
}

template <typename T_Point>
inline void LidarDriverImpl<T_Point>::decodeDifopPkt(const PacketMsg& msg)
{
  lidar_decoder_ptr_->processDifopPkt(msg.packet.data());
  difop_flag_ = true;
}

template <typename T_Point>
inline void LidarDriverImpl<T_Point>::runCallBack(const ScanMsg& msg)
{
  if (msg.seq != 0)
  {
    for (auto& it : msop_pkt_cb_vec_)
    {
      it(msg);
    }
  }
}

template <typename T_Point>
inline void LidarDriverImpl<T_Point>::runCallBack(const PacketMsg& msg)
{
  for (auto& it : difop_pkt_cb_vec_)
  {
    it(msg);
  }
}

template <typename T_Point>
inline void LidarDriverImpl<T_Point>::runCallBack(const PointCloudMsg<T_Point>& msg)
{
  if (msg.seq != 0)
  {
    for (auto& it : point_cloud_cb_vec_)
    {
      it(msg);
    }
  }
}

template <typename T_Point>
inline void LidarDriverImpl<T_Point>::reportError(const Error& error)
{
  for (auto& it : excb_)
  {
    it(error);
  }
}

template <typename T_Point>
inline void LidarDriverImpl<T_Point>::msopCallback(const PacketMsg& msg)
{
  if (msop_pkt_queue_.size() > MAX_PACKETS_BUFFER_SIZE)
  {
    reportError(Error(ERRCODE_PKTBUFOVERFLOW));
    msop_pkt_queue_.clear();
  }
  msop_pkt_queue_.push(msg);
  if (msop_pkt_queue_.is_task_finished_.load())
  {
    msop_pkt_queue_.is_task_finished_.store(false);
    thread_pool_ptr_->commit([this]() { processMsop(); });
  }
}

template <typename T_Point>
inline void LidarDriverImpl<T_Point>::difopCallback(const PacketMsg& msg)
{
  difop_pkt_queue_.push(msg);
  if (difop_pkt_queue_.is_task_finished_.load())
  {
    difop_pkt_queue_.is_task_finished_.store(false);
    thread_pool_ptr_->commit([this]() { processDifop(); });
  }
}

template <typename T_Point>
inline void LidarDriverImpl<T_Point>::processMsop()
{
  if (!difop_flag_ && driver_param_.wait_for_difop)
  {
    ndifop_count_++;
    if (ndifop_count_ > 120)
    {
      reportError(Error(ERRCODE_NODIFOPRECV));
      ndifop_count_ = 0;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    msop_pkt_queue_.clear();
    msop_pkt_queue_.is_task_finished_.store(true);
    return;
  }
  while (msop_pkt_queue_.size() > 0)
  {
    PacketMsg pkt = msop_pkt_queue_.popFront();
    int height = 1;
    int ret = lidar_decoder_ptr_->processMsopPkt(pkt.packet.data(), *point_cloud_ptr_, height);
    scan_ptr_->packets.emplace_back(std::move(pkt));
    if ((ret == DECODE_OK || ret == FRAME_SPLIT))
    {
      if (ret == FRAME_SPLIT)
      {
        PointCloudMsg<T_Point> msg(point_cloud_transform_func_(point_cloud_ptr_, height));
        msg.height = height;
        msg.width = point_cloud_ptr_->size() / msg.height;
        setPointCloudMsgHeader(msg);
        if (driver_param_.decoder_param.use_lidar_clock == true)
        {
          msg.timestamp = lidar_decoder_ptr_->getLidarTime(pkt.packet.data());
        }
        else
        {
          msg.timestamp = getTime();
        }
        if (msg.point_cloud_ptr->size() == 0)
        {
          reportError(Error(ERRCODE_ZEROPOINTS));
        }
        else
        {
          runCallBack(msg);
        }
        setScanMsgHeader(*scan_ptr_);
        runCallBack(*scan_ptr_);
        point_cloud_ptr_.reset(new typename PointCloudMsg<T_Point>::PointCloud);
        scan_ptr_.reset(new ScanMsg);
      }
    }
    else
    {
      reportError(Error(ERRCODE_WRONGPKTHEADER));
      msop_pkt_queue_.clear();
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  }
  msop_pkt_queue_.is_task_finished_.store(true);
}

template <typename T_Point>
inline void LidarDriverImpl<T_Point>::localCameraTriggerCallback(const CameraTrigger& msg)
{
  for (auto& it : camera_trigger_cb_vec_)
  {
    it(msg);
  }
}

template <typename T_Point>
inline void LidarDriverImpl<T_Point>::processDifop()
{
  while (difop_pkt_queue_.size() > 0)
  {
    PacketMsg pkt = difop_pkt_queue_.popFront();
    decodeDifopPkt(pkt);
    runCallBack(pkt);
  }
  difop_pkt_queue_.is_task_finished_.store(true);
}

template <typename T_Point>
inline void LidarDriverImpl<T_Point>::setScanMsgHeader(ScanMsg& msg)
{
  msg.timestamp = getTime();
  if (driver_param_.decoder_param.use_lidar_clock == true)
  {
    msg.timestamp = lidar_decoder_ptr_->getLidarTime(msg.packets.back().packet.data());
  }
  msg.seq = scan_seq_++;
  msg.frame_id = driver_param_.frame_id;
}

template <typename T_Point>
inline void LidarDriverImpl<T_Point>::setPointCloudMsgHeader(PointCloudMsg<T_Point>& msg)
{
  msg.seq = point_cloud_seq_++;
  msg.frame_id = driver_param_.frame_id;
  msg.is_dense = false;
}

}  // namespace lidar
}  // namespace robosense
