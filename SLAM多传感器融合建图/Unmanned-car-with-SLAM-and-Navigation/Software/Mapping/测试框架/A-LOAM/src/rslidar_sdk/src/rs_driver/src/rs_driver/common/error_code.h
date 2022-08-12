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
namespace robosense
{
namespace lidar
{
/**
 * @brief Error Code for RoboSense LiDAR Driver.
 * 0x00 For success info
 * 0x01 ~ 0x40 for Infos, some infomation during the program running
 * 0x41 ~ 0x80 for Warning, the program may not work normally
 * 0x81 ~ 0xC0 for Critical Error, the program will exit
 */
enum class ErrCodeType
{
  INFO_CODE,     ///< Common information
  WARNING_CODE,  ///< Program may not work normally
  ERROR_CODE     ///< Program will exit immediately
};
enum ErrCode
{
  ERRCODE_SUCCESS = 0x00,          ///< Normal
  ERRCODE_PCAPREPEAT = 0x01,       ///< Pcap file will play repeatedly
  ERRCODE_PCAPEXIT = 0x02,         ///< Pcap thread will exit
  ERRCODE_MSOPTIMEOUT = 0x41,      ///< Msop packets receive overtime (1 sec)
  ERRCODE_DIFOPTIMEOUT = 0x42,     ///< Difop packets receive overtime (2 sec)
  ERRCODE_MSOPINCOMPLETE = 0x43,   ///< Incomplete msop packets received
  ERRCODE_DIFOPINCOMPLETE = 0x44,  ///< Incomplete difop packets received
  ERRCODE_NODIFOPRECV = 0x45,      ///< Point cloud decoding process will not start until the difop packet receive
  ERRCODE_ZEROPOINTS = 0x46,       ///< Size of the point cloud is zero
  ERRCODE_STARTBEFOREINIT = 0x47,  ///< start() function is called before initializing successfully
  ERRCODE_PCAPWRONGPATH = 0x48,    ///< Input directory of pcap file is wrong
  ERRCODE_MSOPPORTBUZY = 0x49,     ///< Input msop port is already used
  ERRCODE_DIFOPPORTBUZY = 0x50,    ///< Input difop port is already used
  ERRCODE_WRONGPKTHEADER = 0x51,   ///< Packet header is wrong
  ERRCODE_PKTNULL = 0x52,          ///< Input packet is null
  ERRCODE_PKTBUFOVERFLOW = 0x53    ///< Packet buffer is over flow
};

struct Error
{
  ErrCode error_code;
  ErrCodeType error_code_type;
  explicit Error(const ErrCode& code) : error_code(code)
  {
    if (error_code <= 0x40)
    {
      error_code_type = ErrCodeType::INFO_CODE;
    }
    else if (error_code <= 0x80)
    {
      error_code_type = ErrCodeType::WARNING_CODE;
    }
    else
    {
      error_code_type = ErrCodeType::ERROR_CODE;
    }
  }
  std::string toString() const
  {
    switch (error_code)
    {
      case ERRCODE_PCAPWRONGPATH:
        return "ERRCODE_PCAPWRONGPATH";
      case ERRCODE_MSOPPORTBUZY:
        return "ERRCODE_MSOPPORTBUZY";
      case ERRCODE_DIFOPPORTBUZY:
        return "ERRCODE_DIFOPPORTBUZY";
      case ERRCODE_PCAPREPEAT:
        return "Info_PcapRepeat";
      case ERRCODE_PCAPEXIT:
        return "Info_PcapExit";
      case ERRCODE_MSOPTIMEOUT:
        return "ERRCODE_MSOPTIMEOUT";
      case ERRCODE_DIFOPTIMEOUT:
        return "ERRCODE_DIFOPTIMEOUT";
      case ERRCODE_MSOPINCOMPLETE:
        return "ERRCODE_MSOPINCOMPLETE";
      case ERRCODE_DIFOPINCOMPLETE:
        return "ERRCODE_DIFOPINCOMPLETE";
      case ERRCODE_NODIFOPRECV:
        return "ERRCODE_NODIFOPRECV";
      case ERRCODE_ZEROPOINTS:
        return "ERRCODE_ZEROPOINTS";
      case ERRCODE_STARTBEFOREINIT:
        return "ERRCODE_STARTBEFOREINIT";
      case ERRCODE_WRONGPKTHEADER:
        return "ERRCODE_WRONGPKTHEADER";
      case ERRCODE_PKTNULL:
        return "ERRCODE_PKTNULL";
      case ERRCODE_PKTBUFOVERFLOW:
        return "ERRCODE_PKTBUFOVERFLOW";
      default:
        return "ERRCODE_SUCCESS";
    }
  }
};

}  // namespace lidar
}  // namespace robosense
