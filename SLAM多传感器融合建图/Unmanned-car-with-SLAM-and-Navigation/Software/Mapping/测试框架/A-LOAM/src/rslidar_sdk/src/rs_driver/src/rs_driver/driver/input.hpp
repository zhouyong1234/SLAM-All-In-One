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
#include <rs_driver/common/error_code.h>
#include <rs_driver/utility/thread_pool.hpp>
#include <rs_driver/driver/driver_param.h>
#include <rs_driver/msg/packet_msg.h>
///< 1.0 second / 10 Hz / (360 degree / horiz angle resolution / column per msop packet) * (s to us)
constexpr double RS16_PCAP_SLEEP_DURATION = 1200;  ///< us
constexpr double RS32_PCAP_SLEEP_DURATION = 530;          ///< us
constexpr double RSBP_PCAP_SLEEP_DURATION = 530;          ///< us
constexpr double RS128_PCAP_SLEEP_DURATION = 100;          ///< us
constexpr double RS80_PCAP_SLEEP_DURATION = 135;           ///< us
constexpr double RSM1_PCAP_SLEEP_DURATION = 90;                ///< us
constexpr double RSHELIOS_PCAP_SLEEP_DURATION = 530;      ///< us
using boost::asio::deadline_timer;
using boost::asio::ip::address;
using boost::asio::ip::udp;

namespace robosense
{
namespace lidar
{
class Input
{
public:
  Input(const LidarType& type, const RSInputParam& input_param, const std::function<void(const Error&)>& excb);
  ~Input();
  bool init();
  bool start();
  void stop();
  void regRecvMsopCallback(const std::function<void(const PacketMsg&)>& callback);
  void regRecvDifopCallback(const std::function<void(const PacketMsg&)>& callback);

private:
  inline bool setSocket(const std::string& pkt_type);
  inline void getMsopPacket();
  inline void getDifopPacket();
  inline void getPcapPacket();
  inline void checkDifopDeadline();
  inline void checkMsopDeadline();
  static void handleReceive(const boost::system::error_code& ec, std::size_t length, boost::system::error_code* out_ec,
                            std::size_t* out_length);

private:
  LidarType lidar_type_;
  RSInputParam input_param_;
  std::function<void(const Error&)> excb_;
  bool init_flag_;
  uint32_t msop_pkt_length_;
  uint32_t difop_pkt_length_;
  /* pcap file parse */
  pcap_t* pcap_;
  bpf_program pcap_msop_filter_;
  bpf_program pcap_difop_filter_;
  std::chrono::time_point<std::chrono::system_clock, std::chrono::system_clock::duration> last_packet_time_;
  /* live socket */
  std::unique_ptr<udp::socket> msop_sock_ptr_;
  std::unique_ptr<udp::socket> difop_sock_ptr_;
  std::unique_ptr<deadline_timer> msop_deadline_;
  std::unique_ptr<deadline_timer> difop_deadline_;
  Thread msop_thread_;
  Thread difop_thread_;
  Thread pcap_thread_;
  boost::asio::io_service msop_io_service_;
  boost::asio::io_service difop_io_service_;
  std::vector<std::function<void(const PacketMsg&)>> difop_cb_;
  std::vector<std::function<void(const PacketMsg&)>> msop_cb_;
};

inline Input::Input(const LidarType& type, const RSInputParam& input_param,
                    const std::function<void(const Error&)>& excb)
  : lidar_type_(type), input_param_(input_param), excb_(excb), init_flag_(false), pcap_(nullptr)
{
  last_packet_time_ = std::chrono::system_clock::now();
  input_param_.pcap_rate = input_param_.pcap_rate < 0.1 ? 0.1 : input_param_.pcap_rate;
  switch (type)
  {
    case LidarType::RSM1:
      msop_pkt_length_ = MEMS_MSOP_LEN;
      difop_pkt_length_ = MEMS_DIFOP_LEN;
      break;
    default:
      msop_pkt_length_ = MECH_PKT_LEN;
      difop_pkt_length_ = MECH_PKT_LEN;
      break;
  }
}

inline Input::~Input()
{
  stop();
  if (pcap_ != NULL)
  {
    pcap_close(pcap_);
  }
  msop_sock_ptr_.reset();
  difop_sock_ptr_.reset();
  msop_deadline_.reset();
  difop_deadline_.reset();
}

inline bool Input::init()
{
  if (input_param_.read_pcap)
  {
    char errbuf[PCAP_ERRBUF_SIZE];
    if ((pcap_ = pcap_open_offline(input_param_.pcap_path.c_str(), errbuf)) == NULL)
    {
      excb_(Error(ERRCODE_PCAPWRONGPATH));
      return false;
    }
    else
    {
      std::stringstream msop_filter;
      std::stringstream difop_filter;
      /*ip address filter*/
      // msop_filter << "src host " << input_param_.device_ip << " && ";
      // difop_filter << "src host " << input_param_.device_ip << " && ";
      msop_filter << "udp dst port " << input_param_.msop_port;
      difop_filter << "udp dst port " << input_param_.difop_port;
      pcap_compile(pcap_, &pcap_msop_filter_, msop_filter.str().c_str(), 1, 0xFFFFFFFF);
      pcap_compile(pcap_, &pcap_difop_filter_, difop_filter.str().c_str(), 1, 0xFFFFFFFF);
    }
  }
  else
  {
    if (!setSocket("msop") || !setSocket("difop"))
    {
      return false;
    }
  }
  init_flag_ = true;
  return true;
}

inline bool Input::start()
{
  if (!init_flag_)
  {
    excb_(Error(ERRCODE_STARTBEFOREINIT));
    return false;
  }
  if (!input_param_.read_pcap)
  {
    msop_thread_.start_.store(true);
    difop_thread_.start_.store(true);
    msop_thread_.thread_.reset(new std::thread([this]() { getMsopPacket(); }));
    difop_thread_.thread_.reset(new std::thread([this]() { getDifopPacket(); }));
  }
  else
  {
    pcap_thread_.start_.store(true);
    pcap_thread_.thread_.reset(new std::thread([this]() { getPcapPacket(); }));
  }
  return true;
}

inline void Input::stop()
{
  if (!input_param_.read_pcap)
  {
    msop_thread_.start_.store(false);
    difop_thread_.start_.store(false);
    if (msop_thread_.thread_ != nullptr && msop_thread_.thread_->joinable())
    {
      msop_thread_.thread_->join();
    }
    if (difop_thread_.thread_ != nullptr && difop_thread_.thread_->joinable())
    {
      difop_thread_.thread_->join();
    }
  }
  else
  {
    pcap_thread_.start_.store(false);
    if (pcap_thread_.thread_ != nullptr && pcap_thread_.thread_->joinable())
    {
      pcap_thread_.thread_->join();
    }
  }
}

inline void Input::regRecvMsopCallback(const std::function<void(const PacketMsg&)>& callback)
{
  msop_cb_.emplace_back(callback);
}

inline void Input::regRecvDifopCallback(const std::function<void(const PacketMsg&)>& callback)
{
  difop_cb_.emplace_back(callback);
}

inline bool Input::setSocket(const std::string& pkt_type)
{
  if (pkt_type == "msop")
  {
    try
    {
      msop_sock_ptr_.reset(new udp::socket(msop_io_service_, udp::endpoint(udp::v4(), input_param_.msop_port)));
      msop_deadline_.reset(new deadline_timer(msop_io_service_));
    }
    catch (...)
    {
      excb_(Error(ERRCODE_MSOPPORTBUZY));
      return false;
    }
    if (input_param_.multi_cast_address != "0.0.0.0")
    {
      msop_sock_ptr_->set_option(
          boost::asio::ip::multicast::join_group(address::from_string(input_param_.multi_cast_address).to_v4(),
                                                 udp::endpoint(udp::v4(), input_param_.msop_port).address().to_v4()));
    }
    msop_deadline_->expires_at(boost::posix_time::pos_infin);
    checkMsopDeadline();
  }
  else if (pkt_type == "difop")
  {
    try
    {
      difop_sock_ptr_.reset(new udp::socket(difop_io_service_, udp::endpoint(udp::v4(), input_param_.difop_port)));
      difop_deadline_.reset(new deadline_timer(difop_io_service_));
    }
    catch (...)
    {
      excb_(Error(ERRCODE_DIFOPPORTBUZY));
      return false;
    }
    if (input_param_.multi_cast_address != "0.0.0.0")
    {
      difop_sock_ptr_->set_option(
          boost::asio::ip::multicast::join_group(address::from_string(input_param_.multi_cast_address).to_v4(),
                                                 udp::endpoint(udp::v4(), input_param_.difop_port).address().to_v4()));
    }
    difop_deadline_->expires_at(boost::posix_time::pos_infin);
    checkDifopDeadline();
  }
  return true;
}

inline void Input::getMsopPacket()
{
  char* precv_buffer = (char*)malloc(msop_pkt_length_);
  while (msop_thread_.start_.load())
  {
    msop_deadline_->expires_from_now(boost::posix_time::seconds(1));
    boost::system::error_code ec = boost::asio::error::would_block;
    std::size_t ret = 0;

    msop_sock_ptr_->async_receive(boost::asio::buffer(precv_buffer, msop_pkt_length_),
                                  boost::bind(&Input::handleReceive, _1, _2, &ec, &ret));
    do
    {
      msop_io_service_.run_one();
    } while (ec == boost::asio::error::would_block);
    if (ec)
    {
      excb_(Error(ERRCODE_MSOPTIMEOUT));
      continue;
    }
    if (ret < msop_pkt_length_)
    {
      excb_(Error(ERRCODE_MSOPINCOMPLETE));
      continue;
    }
    PacketMsg msg(msop_pkt_length_);
    memcpy(msg.packet.data(), precv_buffer, msop_pkt_length_);
    for (auto& iter : msop_cb_)
    {
      iter(msg);
    }
  }
  free(precv_buffer);
}

inline void Input::getDifopPacket()
{
  char* precv_buffer = (char*)malloc(difop_pkt_length_);
  while (difop_thread_.start_.load())
  {
    difop_deadline_->expires_from_now(boost::posix_time::seconds(2));
    boost::system::error_code ec = boost::asio::error::would_block;
    std::size_t ret = 0;

    difop_sock_ptr_->async_receive(boost::asio::buffer(precv_buffer, difop_pkt_length_),
                                   boost::bind(&Input::handleReceive, _1, _2, &ec, &ret));
    do
    {
      difop_io_service_.run_one();
    } while (ec == boost::asio::error::would_block);
    if (ec)
    {
      excb_(Error(ERRCODE_DIFOPTIMEOUT));
      continue;
    }
    if (ret < difop_pkt_length_)
    {
      excb_(Error(ERRCODE_DIFOPINCOMPLETE));
      continue;
    }
    PacketMsg msg(difop_pkt_length_);
    memcpy(msg.packet.data(), precv_buffer, difop_pkt_length_);
    for (auto& iter : difop_cb_)
    {
      iter(msg);
    }
  }
  free(precv_buffer);
}

inline void Input::getPcapPacket()
{
  while (pcap_thread_.start_.load())
  {
    struct pcap_pkthdr* header;
    const u_char* pkt_data;
    auto time2go = last_packet_time_;
    switch (lidar_type_)
    {
      case LidarType::RS16:
        time2go += std::chrono::microseconds(static_cast<long long>(RS16_PCAP_SLEEP_DURATION / input_param_.pcap_rate));
        break;
      case LidarType::RS32:
        time2go += std::chrono::microseconds(static_cast<long long>(RS32_PCAP_SLEEP_DURATION / input_param_.pcap_rate));
        break;
      case LidarType::RSBP:
        time2go += std::chrono::microseconds(static_cast<long long>(RSBP_PCAP_SLEEP_DURATION / input_param_.pcap_rate));
        break;
      case LidarType::RS128:
        time2go +=
            std::chrono::microseconds(static_cast<long long>(RS128_PCAP_SLEEP_DURATION / input_param_.pcap_rate));
        break;
      case LidarType::RS80:
        time2go += std::chrono::microseconds(static_cast<long long>(RS80_PCAP_SLEEP_DURATION / input_param_.pcap_rate));
        break;
      case LidarType::RSM1:
        time2go += std::chrono::microseconds(static_cast<long long>(RSM1_PCAP_SLEEP_DURATION / input_param_.pcap_rate));
        break;
      case LidarType::RSHELIOS:
        time2go +=
            std::chrono::microseconds(static_cast<long long>(RSHELIOS_PCAP_SLEEP_DURATION / input_param_.pcap_rate));
        break;

      default:
        break;
    }
    std::this_thread::sleep_until(time2go);
    last_packet_time_ = std::chrono::system_clock::now();
    if (!pcap_thread_.start_.load())
    {
      break;
    }
    if (pcap_next_ex(pcap_, &header, &pkt_data) >= 0)
    {
      if (pcap_offline_filter(&pcap_msop_filter_, header, pkt_data) != 0)
      {
        PacketMsg msg(msop_pkt_length_);
        memcpy(msg.packet.data(), pkt_data + 42, msop_pkt_length_);
        for (auto& iter : msop_cb_)
        {
          iter(msg);
        }
      }
      else if (pcap_offline_filter(&pcap_difop_filter_, header, pkt_data) != 0)
      {
        PacketMsg msg(difop_pkt_length_);
        memcpy(msg.packet.data(), pkt_data + 42, difop_pkt_length_);
        for (auto& iter : difop_cb_)
        {
          iter(msg);
        }
      }
      else
      {
        continue;
      }
    }
    else
    {
      if (input_param_.pcap_repeat)
      {
        excb_(Error(ERRCODE_PCAPREPEAT));
        char errbuf[PCAP_ERRBUF_SIZE];
        pcap_ = pcap_open_offline(input_param_.pcap_path.c_str(), errbuf);
      }
      else
      {
        excb_(Error(ERRCODE_PCAPEXIT));
        break;
      }
    }
  }
}  // namespace lidar

inline void Input::checkDifopDeadline()
{
  if (difop_deadline_->expires_at() <= deadline_timer::traits_type::now())
  {
    difop_sock_ptr_->cancel();
    difop_deadline_->expires_at(boost::posix_time::pos_infin);
  }
  difop_deadline_->async_wait(boost::bind(&Input::checkDifopDeadline, this));
}

inline void Input::checkMsopDeadline()
{
  if (msop_deadline_->expires_at() <= deadline_timer::traits_type::now())
  {
    msop_sock_ptr_->cancel();
    msop_deadline_->expires_at(boost::posix_time::pos_infin);
  }
  msop_deadline_->async_wait(boost::bind(&Input::checkMsopDeadline, this));
}

inline void Input::handleReceive(const boost::system::error_code& ec, std::size_t length,
                                 boost::system::error_code* out_ec, std::size_t* out_length)
{
  *out_ec = ec;
  *out_length = length;
}

}  // namespace lidar
}  // namespace robosense
