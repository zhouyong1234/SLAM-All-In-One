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

#include <rs_driver/driver/decoder/decoder_base.hpp>

namespace robosense
{
namespace lidar
{
const uint32_t SINGLE_PKT_NUM = 630;
const uint32_t DUAL_PKT_NUM = 1260;
const int ANGLE_OFFSET = 32768;
#pragma pack(push, 1)

typedef struct
{
  uint16_t distance;
  uint16_t pitch;
  uint16_t yaw;
  uint8_t intensity;
  uint8_t point_attribute;
  uint8_t elongation;
} RSM1Channel;

typedef struct
{
  uint8_t time_offset;
  uint8_t return_seq;
  RSM1Channel channel[5];
} RSM1Block;

typedef struct
{
  uint32_t id;
  uint16_t pkt_cnt;
  uint16_t protocol_version;
  uint8_t return_mode;
  uint8_t time_mode;
  RSTimestampUTC timestamp;
  uint8_t reserved[10];
  uint8_t lidar_type;
  uint8_t temperature;
} RSM1MsopHeader;

typedef struct
{
  RSM1MsopHeader header;
  RSM1Block blocks[25];
  uint8_t reserved[3];
} RSM1MsopPkt;

typedef struct
{
  uint8_t ip_local[4];
  uint8_t ip_remote[4];
  uint8_t mac[6];
  uint8_t msop_port[2];
  uint8_t difop_port[2];
} RSM1DifopEther;

typedef struct
{
  uint8_t horizontal_fov_start[2];
  uint8_t horizontal_fov_end[2];
  uint8_t vertical_fov_start[2];
  uint8_t vertical_fov_end[2];
} RSM1DifopFov;

typedef struct
{
  uint8_t pl_ver[5];
  uint8_t ps_ver[5];
} RSM1DifopVerInfo;

typedef struct
{
  uint8_t current_1[3];
  uint8_t current_2[3];
  uint16_t voltage_1;
  uint16_t voltage_2;
  uint8_t reserved[10];
} RSM1DifopRunSts;

typedef struct
{
  uint8_t param_sign;
  uint16_t data;
} RSM1DifopCalibration;

typedef struct
{
  uint64_t id;
  uint8_t reserved_1;
  uint8_t frame_rate;
  RSM1DifopEther ether;
  RSM1DifopFov fov_setting;
  RSM1DifopVerInfo ver_info;
  RSSn sn;
  uint8_t return_mode;
  RSTimeInfo time_info;
  RSM1DifopRunSts status;
  uint8_t diag_reserved[40];
  RSM1DifopCalibration cali_param[20];
  uint8_t reserved_2[71];
} RSM1DifopPkt;
#pragma pack(pop)

template <typename T_Point>
class DecoderRSM1 : public DecoderBase<T_Point>
{
public:
  DecoderRSM1(const RSDecoderParam& param, const LidarConstantParameter& lidar_const_param);
  RSDecoderResult decodeDifopPkt(const uint8_t* pkt);
  RSDecoderResult decodeMsopPkt(const uint8_t* pkt, std::vector<T_Point>& vec, int& height, int& azimuth);
  double getLidarTime(const uint8_t* pkt);
  RSDecoderResult processMsopPkt(const uint8_t* pkt, std::vector<T_Point>& pointcloud_vec, int& height);

private:
  uint32_t last_pkt_cnt_;
  uint32_t max_pkt_num_;
  double last_pkt_time_;
};

template <typename T_Point>
inline DecoderRSM1<T_Point>::DecoderRSM1(const RSDecoderParam& param, const LidarConstantParameter& lidar_const_param)
  : DecoderBase<T_Point>(param, lidar_const_param), last_pkt_cnt_(1), max_pkt_num_(SINGLE_PKT_NUM), last_pkt_time_(0)
{
  if (this->param_.max_distance > 200.0f)
  {
    this->param_.max_distance = 200.0f;
  }
  if (this->param_.min_distance < 0.2f || this->param_.min_distance > this->param_.max_distance)
  {
    this->param_.min_distance = 0.2f;
  }
  this->time_duration_between_blocks_ = 5 * 1e-6;
}

template <typename T_Point>
inline double DecoderRSM1<T_Point>::getLidarTime(const uint8_t* pkt)
{
  return this->template calculateTimeUTC<RSM1MsopPkt>(pkt, LidarType::RSM1);
}

template <typename T_Point>
inline RSDecoderResult DecoderRSM1<T_Point>::processMsopPkt(const uint8_t* pkt, std::vector<T_Point>& pointcloud_vec,
                                                            int& height)
{
  int azimuth = 0;
  RSDecoderResult ret = decodeMsopPkt(pkt, pointcloud_vec, height, azimuth);
  this->pkt_count_++;
  switch (this->param_.split_frame_mode)
  {
    case SplitFrameMode::SPLIT_BY_ANGLE:
    case SplitFrameMode::SPLIT_BY_FIXED_PKTS:
      return ret;
    case SplitFrameMode::SPLIT_BY_CUSTOM_PKTS:
      if (this->pkt_count_ >= this->param_.num_pkts_split)
      {
        this->pkt_count_ = 0;
        this->trigger_index_ = 0;
        this->prev_angle_diff_ = RS_ONE_ROUND;
        return FRAME_SPLIT;
      }
      break;
    default:
      break;
  }
  return DECODE_OK;
}

template <typename T_Point>
inline RSDecoderResult DecoderRSM1<T_Point>::decodeMsopPkt(const uint8_t* pkt, std::vector<T_Point>& vec, int& height,
                                                           int& azimuth)
{
  height = this->lidar_const_param_.LASER_NUM;
  RSM1MsopPkt* mpkt_ptr = (RSM1MsopPkt*)pkt;
  if (mpkt_ptr->header.id != this->lidar_const_param_.MSOP_ID)
  {
    return RSDecoderResult::WRONG_PKT_HEADER;
  }
  this->protocol_ver_ = RS_SWAP_SHORT(mpkt_ptr->header.protocol_version);
  double pkt_timestamp = 0;
  switch (mpkt_ptr->blocks[0].return_seq)
  {
    case 0:
      pkt_timestamp = this->get_point_time_func_(pkt);
      break;
    case 1:
      pkt_timestamp = this->get_point_time_func_(pkt);
      last_pkt_time_ = pkt_timestamp;
      break;
    case 2:
      pkt_timestamp = last_pkt_time_;
      break;
  }

  for (size_t blk_idx = 0; blk_idx < this->lidar_const_param_.BLOCKS_PER_PKT; blk_idx++)
  {
    RSM1Block blk = mpkt_ptr->blocks[blk_idx];
    double point_time = pkt_timestamp + blk.time_offset * 1e-6;
    for (size_t channel_idx = 0; channel_idx < this->lidar_const_param_.CHANNELS_PER_BLOCK; channel_idx++)
    {
      T_Point point;
      float distance = RS_SWAP_SHORT(blk.channel[channel_idx].distance) * RS_DIS_RESOLUTION;
      if (distance <= this->param_.max_distance && distance >= this->param_.min_distance)
      {
        int pitch = RS_SWAP_SHORT(blk.channel[channel_idx].pitch) - ANGLE_OFFSET;
        int yaw = RS_SWAP_SHORT(blk.channel[channel_idx].yaw) - ANGLE_OFFSET;
        uint8_t intensity = blk.channel[channel_idx].intensity;
        float x = distance * this->checkCosTable(pitch) * this->checkCosTable(yaw);
        float y = distance * this->checkCosTable(pitch) * this->checkSinTable(yaw);
        float z = distance * this->checkSinTable(pitch);
        this->transformPoint(x, y, z);
        setX(point, x);
        setY(point, y);
        setZ(point, z);
        setIntensity(point, intensity);
      }
      else
      {
        setX(point, NAN);
        setY(point, NAN);
        setZ(point, NAN);
        setIntensity(point, 0);
      }
      setTimestamp(point, point_time);
      setRing(point, channel_idx + 1);
      vec.emplace_back(std::move(point));
    }
  }
  unsigned int pkt_cnt = RS_SWAP_SHORT(mpkt_ptr->header.pkt_cnt);
  if (pkt_cnt == max_pkt_num_ || pkt_cnt < last_pkt_cnt_)
  {
    last_pkt_cnt_ = 1;
    return RSDecoderResult::FRAME_SPLIT;
  }
  last_pkt_cnt_ = pkt_cnt;
  return RSDecoderResult::DECODE_OK;
}

template <typename T_Point>
inline RSDecoderResult DecoderRSM1<T_Point>::decodeDifopPkt(const uint8_t* pkt)
{
  RSM1DifopPkt* dpkt_ptr = (RSM1DifopPkt*)pkt;
  if (dpkt_ptr->id != this->lidar_const_param_.DIFOP_ID)
  {
    return RSDecoderResult::WRONG_PKT_HEADER;
  }
  if (!this->difop_flag_)
  {
    this->echo_mode_ = this->getEchoMode(LidarType::RSM1, dpkt_ptr->return_mode);
    if (this->echo_mode_ == RSEchoMode::ECHO_DUAL)
    {
      max_pkt_num_ = DUAL_PKT_NUM;
    }
    this->difop_flag_ = true;
  }
  return RSDecoderResult::DECODE_OK;
}

}  // namespace lidar
}  // namespace robosense
