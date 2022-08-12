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
#include <rs_driver/common/common_header.h>
#include <rs_driver/utility/time.h>
#include <rs_driver/driver/driver_param.h>
namespace robosense
{
namespace lidar
{
#define DEFINE_MEMBER_CHECKER(member)                                                                                  \
  template <typename T, typename V = bool>                                                                             \
  struct has_##member : std::false_type                                                                                \
  {                                                                                                                    \
  };                                                                                                                   \
  template <typename T>                                                                                                \
  struct has_##member<                                                                                                 \
      T, typename std::enable_if<!std::is_same<decltype(std::declval<T>().member), void>::value, bool>::type>          \
    : std::true_type                                                                                                   \
  {                                                                                                                    \
  };
#define RS_HAS_MEMBER(C, member) has_##member<C>::value
DEFINE_MEMBER_CHECKER(x)
DEFINE_MEMBER_CHECKER(y)
DEFINE_MEMBER_CHECKER(z)
DEFINE_MEMBER_CHECKER(intensity)
DEFINE_MEMBER_CHECKER(ring)
DEFINE_MEMBER_CHECKER(timestamp)
#define RS_SWAP_SHORT(x) ((((x)&0xFF) << 8) | (((x)&0xFF00) >> 8))
#define RS_SWAP_LONG(x) ((((x)&0xFF) << 24) | (((x)&0xFF00) << 8) | (((x)&0xFF0000) >> 8) | (((x)&0xFF000000) >> 24))
#define RS_TO_RADS(x) ((x) * (M_PI) / 180)
constexpr float RS_DIS_RESOLUTION = 0.005;
constexpr float RS_HELIOS_DIS_RESOLUTION = 0.0025;
constexpr float RS_ANGLE_RESOLUTION = 0.01;
constexpr float MICRO = 1000000.0;
constexpr float NANO = 1000000000.0;
constexpr int RS_ONE_ROUND = 36000;
constexpr uint16_t PROTOCOL_VER_0 = 0x00;
/* Echo mode definition */
enum RSEchoMode
{
  ECHO_SINGLE,
  ECHO_DUAL
};

/* Decode result definition */
enum RSDecoderResult
{
  DECODE_OK = 0,
  FRAME_SPLIT = 1,
  WRONG_PKT_HEADER = -1,
  PKT_NULL = -2
};

#pragma pack(push, 1)
typedef struct
{
  uint64_t MSOP_ID;
  uint64_t DIFOP_ID;
  uint64_t BLOCK_ID;
  uint32_t PKT_RATE;
  uint16_t BLOCKS_PER_PKT;
  uint16_t CHANNELS_PER_BLOCK;
  uint16_t LASER_NUM;
  float DSR_TOFFSET;
  float FIRING_FREQUENCY;
  float RX;
  float RY;
  float RZ;
} LidarConstantParameter;

typedef struct
{
  uint8_t year;
  uint8_t month;
  uint8_t day;
  uint8_t hour;
  uint8_t minute;
  uint8_t second;
  uint16_t ms;
  uint16_t us;
} RSTimestampYMD;

typedef struct
{
  uint8_t sec[6];
  uint32_t us;
} RSTimestampUTC;

typedef struct
{
  uint8_t sync_mode;
  uint8_t sync_sts;
  RSTimestampUTC timestamp;
} RSTimeInfo;

typedef struct
{
  uint64_t id;
  uint8_t reserved_1[12];
  RSTimestampYMD timestamp;
  uint8_t lidar_type;
  uint8_t reserved_2[7];
  uint16_t temp_raw;
  uint8_t reserved_3[2];
} RSMsopHeader;

typedef struct
{
  uint32_t id;
  uint16_t protocol_version;
  uint8_t reserved_1;
  uint8_t wave_mode;
  uint8_t temp_low;
  uint8_t temp_high;
  RSTimestampUTC timestamp;
  uint8_t reserved_2[10];
  uint8_t lidar_type;
  uint8_t reserved_3[49];
} RSMsopHeaderNew;

typedef struct
{
  uint8_t lidar_ip[4];
  uint8_t host_ip[4];
  uint8_t mac_addr[6];
  uint16_t local_port;
  uint16_t dest_port;
  uint16_t port3;
  uint16_t port4;
} RSEthNet;

typedef struct
{
  uint8_t lidar_ip[4];
  uint8_t dest_ip[4];
  uint8_t mac_addr[6];
  uint16_t msop_port;
  uint16_t reserve_1;
  uint16_t difop_port;
  uint16_t reserve_2;
} RSEthNetNew;

typedef struct
{
  uint16_t start_angle;
  uint16_t end_angle;
} RSFOV;

typedef struct
{
  uint8_t sign;
  uint16_t value;
} RSCalibrationAngle;

typedef struct
{
  uint16_t distance;
  uint8_t intensity;
} RSChannel;

typedef struct
{
  uint8_t top_ver[5];
  uint8_t bottom_ver[5];
} RSVersion;

typedef struct
{
  uint8_t top_firmware_ver[5];
  uint8_t bot_firmware_ver[5];
  uint8_t bot_soft_ver[5];
  uint8_t motor_firmware_ver[5];
  uint8_t hw_ver[3];
} RSVersionNew;

typedef struct
{
  uint8_t num[6];
} RSSn;

typedef struct
{
  uint8_t device_current[3];
  uint8_t main_current[3];
  uint16_t vol_12v;
  uint16_t vol_sim_1v8;
  uint16_t vol_dig_3v3;
  uint16_t vol_sim_3v3;
  uint16_t vol_dig_5v4;
  uint16_t vol_sim_5v;
  uint16_t vol_ejc_5v;
  uint16_t vol_recv_5v;
  uint16_t vol_apd;
} RSStatus;

typedef struct
{
  uint16_t device_current;
  uint16_t vol_fpga;
  uint16_t vol_12v;
  uint16_t vol_dig_5v4;
  uint16_t vol_sim_5v;
  uint16_t vol_apd;
  uint8_t reserved[12];
} RSStatusNew;

typedef struct
{
  uint8_t reserved_1[9];
  uint16_t checksum;
  uint16_t manc_err1;
  uint16_t manc_err2;
  uint8_t gps_status;
  uint16_t temperature1;
  uint16_t temperature2;
  uint16_t temperature3;
  uint16_t temperature4;
  uint16_t temperature5;
  uint8_t reserved_2[5];
  uint16_t cur_rpm;
  uint8_t reserved_3[7];
} RSDiagno;

typedef struct
{
  uint16_t bot_fpga_temperature;
  uint16_t recv_A_temperature;
  uint16_t recv_B_temperature;
  uint16_t main_fpga_temperature;
  uint16_t main_fpga_core_temperature;
  uint16_t real_rpm;
  uint8_t lane_up;
  uint16_t lane_up_cnt;
  uint16_t main_status;
  uint8_t gps_status;
  uint8_t reserved[22];
} RSDiagnoNew;

#pragma pack(pop)

//----------------- Decoder ---------------------
template <typename T_Point>
class DecoderBase
{
public:
  explicit DecoderBase(const RSDecoderParam& param, const LidarConstantParameter& lidar_const_param);
  DecoderBase(const DecoderBase&) = delete;
  DecoderBase& operator=(const DecoderBase&) = delete;
  virtual ~DecoderBase() = default;
  virtual RSDecoderResult processMsopPkt(const uint8_t* pkt, std::vector<T_Point>& point_cloud_vec, int& height);
  virtual RSDecoderResult processDifopPkt(const uint8_t* pkt);
  virtual void loadCalibrationFile(const std::string& angle_path);
  virtual void regRecvCallback(const std::function<void(const CameraTrigger&)>& callback);  ///< Camera trigger
  virtual double getLidarTemperature();
  virtual double getLidarTime(const uint8_t* pkt) = 0;

protected:
  virtual float computeTemperature(const uint16_t& temp_raw);
  virtual float computeTemperature(const uint8_t& temp_low, const uint8_t& temp_high);
  virtual int azimuthCalibration(const float& azimuth, const int& channel);
  virtual void checkTriggerAngle(const int& angle, const double& timestamp);
  virtual RSDecoderResult decodeMsopPkt(const uint8_t* pkt, std::vector<T_Point>& vec, int& height, int& azimuth) = 0;
  virtual RSDecoderResult decodeDifopPkt(const uint8_t* pkt) = 0;
  RSEchoMode getEchoMode(const LidarType& type, const uint8_t& return_mode);
  template <typename T_Msop>
  double calculateTimeUTC(const uint8_t* pkt, const LidarType& type);
  template <typename T_Msop>
  double calculateTimeYMD(const uint8_t* pkt);
  template <typename T_Difop>
  void decodeDifopCommon(const uint8_t* pkt, const LidarType& type);
  template <typename T_Difop>
  void decodeDifopCalibration(const uint8_t* pkt, const LidarType& type);
  void transformPoint(float& x, float& y, float& z);
  float checkCosTable(const int& angle);
  float checkSinTable(const int& angle);
  void sortBeamTable();

private:
  std::vector<double> initTrigonometricLookupTable(const std::function<double(const double)>& func);

protected:
  const LidarConstantParameter lidar_const_param_;
  RSDecoderParam param_;
  RSEchoMode echo_mode_;
  unsigned int pkts_per_frame_;
  unsigned int pkt_count_;
  unsigned int trigger_index_;
  unsigned int prev_angle_diff_;
  unsigned int rpm_;
  unsigned int protocol_ver_;
  int start_angle_;
  int end_angle_;
  int cut_angle_;
  int last_azimuth_;
  bool angle_flag_;
  bool difop_flag_;
  float fov_time_jump_diff_;
  float time_duration_between_blocks_;
  float current_temperature_;
  float azi_diff_between_block_theoretical_;
  std::vector<int> vert_angle_list_;
  std::vector<int> hori_angle_list_;
  std::vector<uint16_t> beam_ring_table_;
  std::vector<std::function<void(const CameraTrigger&)>> camera_trigger_cb_vec_;
  std::function<double(const uint8_t*)> get_point_time_func_;
  std::function<void(const int&, const uint8_t*)> check_camera_trigger_func_;

private:
  std::vector<double> cos_lookup_table_;
  std::vector<double> sin_lookup_table_;
};

template <typename T_Point>
inline DecoderBase<T_Point>::DecoderBase(const RSDecoderParam& param, const LidarConstantParameter& lidar_const_param)
  : lidar_const_param_(lidar_const_param)
  , param_(param)
  , echo_mode_(ECHO_SINGLE)
  , pkts_per_frame_(lidar_const_param.PKT_RATE / 10)
  , pkt_count_(0)
  , trigger_index_(0)
  , prev_angle_diff_(RS_ONE_ROUND)
  , rpm_(600)
  , protocol_ver_(0)
  , start_angle_(param.start_angle * 100)
  , end_angle_(param.end_angle * 100)
  , cut_angle_(param.cut_angle * 100)
  , last_azimuth_(-36001)
  , angle_flag_(true)
  , difop_flag_(false)
  , fov_time_jump_diff_(0)
  , time_duration_between_blocks_(0)
  , current_temperature_(0)
  , azi_diff_between_block_theoretical_(20)
{
  if (cut_angle_ > RS_ONE_ROUND)
  {
    cut_angle_ = 0;
  }
  if (this->start_angle_ > RS_ONE_ROUND || this->start_angle_ < 0 || this->end_angle_ > RS_ONE_ROUND ||
      this->end_angle_ < 0)
  {
    RS_WARNING << "start_angle & end_angle should be in range 0~360° " << RS_REND;
    this->start_angle_ = 0;
    this->end_angle_ = RS_ONE_ROUND;
  }
  if (this->start_angle_ > this->end_angle_)
  {
    this->angle_flag_ = false;
  }

  /* Point time function*/
  if (RS_HAS_MEMBER(T_Point, timestamp))  ///< return the timestamp of the first block in one packet
  {
    if (this->param_.use_lidar_clock)
    {
      get_point_time_func_ = [this](const uint8_t* pkt) { return getLidarTime(pkt); };
    }
    else
    {
      get_point_time_func_ = [this](const uint8_t* pkt) {
        double ret_time =
            getTime() - (this->lidar_const_param_.BLOCKS_PER_PKT - 1) * this->time_duration_between_blocks_;
        return ret_time;
      };
    }
  }
  else
  {
    get_point_time_func_ = [this](const uint8_t* pkt) { return 0; };
  }
  /*Camera trigger function*/
  if (param.trigger_param.trigger_map.size() != 0)
  {
    if (this->param_.use_lidar_clock)
    {
      check_camera_trigger_func_ = [this](const int& azimuth, const uint8_t* pkt) {
        checkTriggerAngle(azimuth, getLidarTime(pkt));
      };
    }
    else
    {
      check_camera_trigger_func_ = [this](const int& azimuth, const uint8_t* pkt) {
        checkTriggerAngle(azimuth, getTime());
      };
    }
  }
  else
  {
    check_camera_trigger_func_ = [this](const int& azimuth, const uint8_t* pkt) { return; };
  }

  /* Cos & Sin look-up table*/
  cos_lookup_table_ = initTrigonometricLookupTable([](const double rad) -> double { return std::cos(rad); });
  sin_lookup_table_ = initTrigonometricLookupTable([](const double rad) -> double { return std::sin(rad); });
}

template <typename T_Point>
inline RSDecoderResult DecoderBase<T_Point>::processDifopPkt(const uint8_t* pkt)
{
  if (pkt == NULL)
  {
    return PKT_NULL;
  }
  return decodeDifopPkt(pkt);
}

template <typename T_Point>
inline RSDecoderResult DecoderBase<T_Point>::processMsopPkt(const uint8_t* pkt, std::vector<T_Point>& point_cloud_vec,
                                                            int& height)
{
  if (pkt == NULL)
  {
    return PKT_NULL;
  }
  int azimuth = 0;
  RSDecoderResult ret = decodeMsopPkt(pkt, point_cloud_vec, height, azimuth);
  if (ret != RSDecoderResult::DECODE_OK)
  {
    return ret;
  }
  this->pkt_count_++;
  switch (this->param_.split_frame_mode)
  {
    case SplitFrameMode::SPLIT_BY_ANGLE:
      if (azimuth < this->last_azimuth_)
      {
        this->last_azimuth_ -= RS_ONE_ROUND;
      }
      if (this->last_azimuth_ != -36001 && this->last_azimuth_ < this->cut_angle_ && azimuth >= this->cut_angle_)
      {
        this->last_azimuth_ = azimuth;
        this->pkt_count_ = 0;
        this->trigger_index_ = 0;
        this->prev_angle_diff_ = RS_ONE_ROUND;
        return FRAME_SPLIT;
      }
      this->last_azimuth_ = azimuth;
      break;
    case SplitFrameMode::SPLIT_BY_FIXED_PKTS:
      if (this->pkt_count_ >= this->pkts_per_frame_)
      {
        this->pkt_count_ = 0;
        this->trigger_index_ = 0;
        this->prev_angle_diff_ = RS_ONE_ROUND;
        return FRAME_SPLIT;
      }
      break;
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
inline void DecoderBase<T_Point>::regRecvCallback(const std::function<void(const CameraTrigger&)>& callback)
{
  camera_trigger_cb_vec_.emplace_back(callback);
}

template <typename T_Point>
inline double DecoderBase<T_Point>::getLidarTemperature()
{
  return current_temperature_;
}

template <typename T_Point>
inline void DecoderBase<T_Point>::loadCalibrationFile(const std::string& angle_path)
{
  std::string line_str;
  std::ifstream fd_angle(angle_path.c_str(), std::ios::in);
  if (fd_angle.is_open())
  {
    unsigned int row_index = 0;
    while (std::getline(fd_angle, line_str))
    {
      std::stringstream ss(line_str);
      std::string str;
      std::vector<std::string> vect_str;
      while (std::getline(ss, str, ','))
      {
        vect_str.emplace_back(str);
      }
      try
      {
        this->vert_angle_list_[row_index] = std::stof(vect_str.at(0)) * 100;  // degree
        this->hori_angle_list_[row_index] = std::stof(vect_str.at(1)) * 100;  // degree
      }
      catch (...)
      {
        RS_WARNING << "Wrong calibration file format! Please check your angle.csv file!" << RS_REND;
        break;
      }
      row_index++;
      if (row_index >= this->lidar_const_param_.LASER_NUM)
      {
        this->sortBeamTable();
        break;
      }
    }
    fd_angle.close();
  }
}

template <typename T_Point>
inline void DecoderBase<T_Point>::checkTriggerAngle(const int& angle, const double& timestamp)
{
  std::map<double, std::string>::iterator iter = param_.trigger_param.trigger_map.begin();
  for (size_t i = 0; i < trigger_index_; i++)
  {
    iter++;
  }
  if (iter != param_.trigger_param.trigger_map.end())
  {
    unsigned int angle_diff = std::abs(iter->first * 100 - angle);
    if (angle_diff < prev_angle_diff_)
    {
      prev_angle_diff_ = angle_diff;
      return;
    }
    else
    {
      trigger_index_++;
      prev_angle_diff_ = RS_ONE_ROUND;
      for (auto cb : camera_trigger_cb_vec_)
      {
        cb(std::make_pair(iter->second, timestamp));
      }
    }
  }
}

/* 16, 32, BP & RSHELIOS */
template <typename T_Point>
inline float DecoderBase<T_Point>::computeTemperature(const uint16_t& temp_raw)
{
  uint8_t neg_flag = (temp_raw >> 8) & 0x80;
  float msb = (temp_raw >> 8) & 0x7F;
  float lsb = (temp_raw & 0x00FF) >> 3;
  float temp;
  if (neg_flag == 0x80)
  {
    temp = -1 * (msb * 32 + lsb) * 0.0625f;
  }
  else
  {
    temp = (msb * 32 + lsb) * 0.0625f;
  }

  return temp;
}

/* 128 & 80 */
template <typename T_Point>
inline float DecoderBase<T_Point>::computeTemperature(const uint8_t& temp_low, const uint8_t& temp_high)
{
  uint8_t neg_flag = temp_low & 0x80;
  float msb = temp_low & 0x7F;
  float lsb = temp_high >> 4;
  float temp;
  if (neg_flag == 0x80)
  {
    temp = -1 * (msb * 16 + lsb) * 0.0625f;
  }
  else
  {
    temp = (msb * 16 + lsb) * 0.0625f;
  }

  return temp;
}

template <typename T_Point>
inline int DecoderBase<T_Point>::azimuthCalibration(const float& azimuth, const int& channel)
{
  return (static_cast<int>(azimuth) + this->hori_angle_list_[channel] + RS_ONE_ROUND) % RS_ONE_ROUND;
}

template <typename T_Point>
template <typename T_Difop>
inline void DecoderBase<T_Point>::decodeDifopCommon(const uint8_t* pkt, const LidarType& type)
{
  const T_Difop* dpkt_ptr = reinterpret_cast<const T_Difop*>(pkt);
  this->echo_mode_ = this->getEchoMode(type, dpkt_ptr->return_mode);
  this->rpm_ = RS_SWAP_SHORT(dpkt_ptr->rpm);
  if (this->rpm_ == 0)
  {
    RS_WARNING << "LiDAR RPM is 0" << RS_REND;
    this->rpm_ = 600;
  }
  this->time_duration_between_blocks_ =
      (60 / static_cast<float>(this->rpm_)) /
      ((this->lidar_const_param_.PKT_RATE * 60 / this->rpm_) * this->lidar_const_param_.BLOCKS_PER_PKT);
  int fov_start_angle = RS_SWAP_SHORT(dpkt_ptr->fov.start_angle);
  int fov_end_angle = RS_SWAP_SHORT(dpkt_ptr->fov.end_angle);
  int fov_range = (fov_start_angle < fov_end_angle) ? (fov_end_angle - fov_start_angle) :
                                                      (RS_ONE_ROUND - fov_start_angle + fov_end_angle);
  int blocks_per_round =
      (this->lidar_const_param_.PKT_RATE / (this->rpm_ / 60)) * this->lidar_const_param_.BLOCKS_PER_PKT;
  this->fov_time_jump_diff_ =
      this->time_duration_between_blocks_ * (fov_range / (RS_ONE_ROUND / static_cast<float>(blocks_per_round)));
  if (this->echo_mode_ == RSEchoMode::ECHO_DUAL)
  {
    this->pkts_per_frame_ = ceil(2 * this->lidar_const_param_.PKT_RATE * 60 / this->rpm_);
  }
  else
  {
    this->pkts_per_frame_ = ceil(this->lidar_const_param_.PKT_RATE * 60 / this->rpm_);
  }
  this->azi_diff_between_block_theoretical_ =
      (RS_ONE_ROUND / this->lidar_const_param_.BLOCKS_PER_PKT) /
      static_cast<float>(this->pkts_per_frame_);  ///< ((rpm/60)*360)/pkts_rate/blocks_per_pkt
}

template <typename T_Point>
template <typename T_Difop>
inline void DecoderBase<T_Point>::decodeDifopCalibration(const uint8_t* pkt, const LidarType& type)
{
  const T_Difop* dpkt_ptr = reinterpret_cast<const T_Difop*>(pkt);
  const uint8_t* p_ver_cali = reinterpret_cast<const uint8_t*>(dpkt_ptr->ver_angle_cali);
  if ((p_ver_cali[0] == 0x00 || p_ver_cali[0] == 0xFF) && (p_ver_cali[1] == 0x00 || p_ver_cali[1] == 0xFF) &&
      (p_ver_cali[2] == 0x00 || p_ver_cali[2] == 0xFF) && (p_ver_cali[3] == 0x00 || p_ver_cali[3] == 0xFF))
  {
    return;
  }
  int neg = 1;
  for (size_t i = 0; i < this->lidar_const_param_.LASER_NUM; i++)
  {
    /* vert angle calibration data */
    neg = static_cast<int>(dpkt_ptr->ver_angle_cali[i].sign) == 0 ? 1 : -1;
    this->vert_angle_list_[i] = static_cast<int>(RS_SWAP_SHORT(dpkt_ptr->ver_angle_cali[i].value)) * neg;
    /* horizon angle calibration data */
    neg = static_cast<int>(dpkt_ptr->hori_angle_cali[i].sign) == 0 ? 1 : -1;
    this->hori_angle_list_[i] = static_cast<int>(RS_SWAP_SHORT(dpkt_ptr->hori_angle_cali[i].value)) * neg;
    if (type == LidarType::RS32)
    {
      this->vert_angle_list_[i] *= 0.1f;
      this->hori_angle_list_[i] *= 0.1f;
    }
  }
  this->sortBeamTable();
  this->difop_flag_ = true;
}

template <typename T_Point>
template <typename T_Msop>
inline double DecoderBase<T_Point>::calculateTimeUTC(const uint8_t* pkt, const LidarType& type)
{
  const T_Msop* mpkt_ptr = reinterpret_cast<const T_Msop*>(pkt);
  union u_ts
  {
    uint8_t data[8];
    uint64_t ts;
  } timestamp;
  timestamp.data[7] = 0;
  timestamp.data[6] = 0;
  timestamp.data[5] = mpkt_ptr->header.timestamp.sec[0];
  timestamp.data[4] = mpkt_ptr->header.timestamp.sec[1];
  timestamp.data[3] = mpkt_ptr->header.timestamp.sec[2];
  timestamp.data[2] = mpkt_ptr->header.timestamp.sec[3];
  timestamp.data[1] = mpkt_ptr->header.timestamp.sec[4];
  timestamp.data[0] = mpkt_ptr->header.timestamp.sec[5];

  if ((type == LidarType::RS80 || type == LidarType::RS128) && this->protocol_ver_ == PROTOCOL_VER_0)
  {
    return static_cast<double>(timestamp.ts) +
           (static_cast<double>(RS_SWAP_LONG(mpkt_ptr->header.timestamp.us))) / NANO;
  }

  return static_cast<double>(timestamp.ts) + (static_cast<double>(RS_SWAP_LONG(mpkt_ptr->header.timestamp.us))) / MICRO;
}

template <typename T_Point>
template <typename T_Msop>
inline double DecoderBase<T_Point>::calculateTimeYMD(const uint8_t* pkt)
{
#ifdef _MSC_VER
  long timezone = 0;
  _get_timezone(&timezone);
#endif
  const T_Msop* mpkt_ptr = reinterpret_cast<const T_Msop*>(pkt);
  std::tm stm;
  memset(&stm, 0, sizeof(stm));
  stm.tm_year = mpkt_ptr->header.timestamp.year + 100;
  stm.tm_mon = mpkt_ptr->header.timestamp.month - 1;
  stm.tm_mday = mpkt_ptr->header.timestamp.day;
  stm.tm_hour = mpkt_ptr->header.timestamp.hour;
  stm.tm_min = mpkt_ptr->header.timestamp.minute;
  stm.tm_sec = mpkt_ptr->header.timestamp.second;
  return std::mktime(&stm) + static_cast<double>(RS_SWAP_SHORT(mpkt_ptr->header.timestamp.ms)) / 1000.0 +
         static_cast<double>(RS_SWAP_SHORT(mpkt_ptr->header.timestamp.us)) / 1000000.0 - static_cast<double>(timezone);
}

template <typename T_Point>
inline void DecoderBase<T_Point>::transformPoint(float& x, float& y, float& z)
{
#ifdef ENABLE_TRANSFORM
  Eigen::AngleAxisd current_rotation_x(param_.transform_param.roll, Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd current_rotation_y(param_.transform_param.pitch, Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd current_rotation_z(param_.transform_param.yaw, Eigen::Vector3d::UnitZ());
  Eigen::Translation3d current_translation(param_.transform_param.x, param_.transform_param.y,
                                           param_.transform_param.z);
  Eigen::Matrix4d trans = (current_translation * current_rotation_z * current_rotation_y * current_rotation_x).matrix();
  Eigen::Vector4d target_ori(x, y, z, 1);
  Eigen::Vector4d target_rotate = trans * target_ori;
  x = target_rotate(0);
  y = target_rotate(1);
  z = target_rotate(2);
#endif
}

template <typename T_Point>
inline void DecoderBase<T_Point>::sortBeamTable()
{
  std::vector<size_t> sorted_idx(this->lidar_const_param_.LASER_NUM);
  std::iota(sorted_idx.begin(), sorted_idx.end(), 0);
  std::sort(sorted_idx.begin(), sorted_idx.end(), [this](std::size_t i1, std::size_t i2) -> bool {
    return this->vert_angle_list_[i1] < this->vert_angle_list_[i2];
  });
  for (size_t i = 0; i < this->lidar_const_param_.LASER_NUM; i++)
  {
    this->beam_ring_table_[sorted_idx[i]] = i;
  }
}

template <typename T_Point>
inline typename std::enable_if<!RS_HAS_MEMBER(T_Point, x)>::type setX(T_Point& point, const float& value)
{
}

template <typename T_Point>
inline typename std::enable_if<RS_HAS_MEMBER(T_Point, x)>::type setX(T_Point& point, const float& value)
{
  point.x = value;
}

template <typename T_Point>
inline typename std::enable_if<!RS_HAS_MEMBER(T_Point, y)>::type setY(T_Point& point, const float& value)
{
}

template <typename T_Point>
inline typename std::enable_if<RS_HAS_MEMBER(T_Point, y)>::type setY(T_Point& point, const float& value)
{
  point.y = value;
}

template <typename T_Point>
inline typename std::enable_if<!RS_HAS_MEMBER(T_Point, z)>::type setZ(T_Point& point, const float& value)
{
}

template <typename T_Point>
inline typename std::enable_if<RS_HAS_MEMBER(T_Point, z)>::type setZ(T_Point& point, const float& value)
{
  point.z = value;
}

template <typename T_Point>
inline typename std::enable_if<!RS_HAS_MEMBER(T_Point, intensity)>::type setIntensity(T_Point& point,
                                                                                      const uint8_t& value)
{
}

template <typename T_Point>
inline typename std::enable_if<RS_HAS_MEMBER(T_Point, intensity)>::type setIntensity(T_Point& point,
                                                                                     const uint8_t& value)
{
  point.intensity = value;
}

template <typename T_Point>
inline typename std::enable_if<!RS_HAS_MEMBER(T_Point, ring)>::type setRing(T_Point& point, const uint16_t& value)
{
}

template <typename T_Point>
inline typename std::enable_if<RS_HAS_MEMBER(T_Point, ring)>::type setRing(T_Point& point, const uint16_t& value)
{
  point.ring = value;
}

template <typename T_Point>
inline typename std::enable_if<!RS_HAS_MEMBER(T_Point, timestamp)>::type setTimestamp(T_Point& point,
                                                                                      const double& value)
{
}

template <typename T_Point>
inline typename std::enable_if<RS_HAS_MEMBER(T_Point, timestamp)>::type setTimestamp(T_Point& point,
                                                                                     const double& value)
{
  point.timestamp = value;
}

template <typename T_Point>
inline RSEchoMode DecoderBase<T_Point>::getEchoMode(const LidarType& type, const uint8_t& return_mode)
{
  switch (type)
  {
    case LidarType::RS128:
    case LidarType::RS80:
    case LidarType::RSHELIOS:
      switch (return_mode)
      {
        case 0x00:
        case 0x03:
          return RSEchoMode::ECHO_DUAL;
        default:
          return RSEchoMode::ECHO_SINGLE;
      }
    case LidarType::RS16:
    case LidarType::RS32:
    case LidarType::RSBP:
      switch (return_mode)
      {
        case 0x00:
          return RSEchoMode::ECHO_DUAL;
        default:
          return RSEchoMode::ECHO_SINGLE;
      }
    case LidarType::RSM1:
      switch (return_mode)
      {
        case 0x00:
        case 0x01:
        case 0x02:
        case 0x03:
          return RSEchoMode::ECHO_DUAL;
        default:
          return RSEchoMode::ECHO_SINGLE;
      }
  }
  return RSEchoMode::ECHO_SINGLE;
}

template <typename T_Point>
inline float DecoderBase<T_Point>::checkCosTable(const int& angle)
{
  return cos_lookup_table_[angle + RS_ONE_ROUND];
}
template <typename T_Point>
inline float DecoderBase<T_Point>::checkSinTable(const int& angle)
{
  return sin_lookup_table_[angle + RS_ONE_ROUND];
}

template <typename T_Point>
inline std::vector<double>
DecoderBase<T_Point>::initTrigonometricLookupTable(const std::function<double(const double)>& func)
{
  std::vector<double> temp_table = std::vector<double>(2 * RS_ONE_ROUND, 0.0);
  for (int i = 0; i < 2 * RS_ONE_ROUND; i++)
  {
    const double rad = RS_TO_RADS(static_cast<double>(i - RS_ONE_ROUND) * RS_ANGLE_RESOLUTION);
    temp_table[i] = func(rad);
  }
  return temp_table;
}

}  // namespace lidar
}  // namespace robosense