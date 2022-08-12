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
#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif
/*Common*/
#include <cstdint>
#include <string>
#include <vector>
#include <iostream>
#include <fstream>
#include <sstream>
#include <ctime>
#include <cmath>
#include <memory>
#include <array>
#include <algorithm>
#include <functional>
#include <iterator>
#include <chrono>
#include <queue>
#include <thread>
#include <atomic>
#include <condition_variable>
#include <future>
#include <stdexcept>
#include <mutex>
#include <type_traits>
#include <numeric>
#include <boost/bind.hpp>
#include <boost/asio.hpp>
#include <boost/date_time/posix_time/posix_time_types.hpp>
#include <rs_driver/macro/version.h>
/*Linux*/
#ifdef __linux__
#include <arpa/inet.h>
#include <netdb.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>
#elif _WIN32
#include <winsock2.h>
#include <windows.h>
#endif

/*Eigen*/
#ifdef ENABLE_TRANSFORM
#include <Eigen/Dense>
#endif

#if defined(_WIN32)
#include <io.h>
#include <windows.h>
inline void setConsoleColor(WORD c)
{
  HANDLE hConsole = GetStdHandle(STD_OUTPUT_HANDLE);
  SetConsoleTextAttribute(hConsole, c);
}
#endif

/*Pcap*/
#include <pcap.h>

/*Camera*/
typedef std::pair<std::string, double> CameraTrigger;

/*Packet Length*/
const size_t MECH_PKT_LEN = 1248;
const size_t MEMS_MSOP_LEN = 1210;
const size_t MEMS_DIFOP_LEN = 256;
/*Output style*/
#ifndef RS_INFOL
#if defined(_WIN32)
inline std::ostream& _RS_INFOL()
{
  setConsoleColor(FOREGROUND_GREEN);
  return std::cout;
}
#define RS_INFOL _RS_INFOL()
#else
#define RS_INFOL (std::cout << "\033[32m")
#endif
#endif

#ifndef RS_INFO
#if defined(_WIN32)
inline std::ostream& _RS_INFO()
{
  setConsoleColor(FOREGROUND_GREEN | FOREGROUND_INTENSITY);
  return std::cout;
}
#define RS_INFO _RS_INFO()
#else
#define RS_INFO (std::cout << "\033[1m\033[32m")
#endif
#endif

#ifndef RS_WARNING
#if defined(_WIN32)
inline std::ostream& _RS_WARNING()
{
  setConsoleColor(FOREGROUND_GREEN | FOREGROUND_RED | FOREGROUND_INTENSITY);
  return std::cout;
}
#define RS_WARNING _RS_WARNING()
#else
#define RS_WARNING (std::cout << "\033[1m\033[33m")
#endif
#endif

#ifndef RS_ERROR
#if defined(_WIN32)
inline std::ostream& _RS_ERROR()
{
  setConsoleColor(FOREGROUND_RED | FOREGROUND_INTENSITY);
  return std::cout;
}
#define RS_ERROR _RS_ERROR()
#else
#define RS_ERROR (std::cout << "\033[1m\033[31m")
#endif
#endif

#ifndef RS_DEBUG
#if defined(_WIN32)
inline std::ostream& _RS_DEBUG()
{
  setConsoleColor(FOREGROUND_GREEN);
  return std::cout;
}
#define RS_DEBUG _RS_DEBUG()
#else
#define RS_DEBUG (std::cout << "\033[1m\033[36m")
#endif
#endif

#ifndef RS_TITLE
#if defined(_WIN32)
inline std::ostream& _RS_TITLE()
{
  setConsoleColor(FOREGROUND_BLUE | FOREGROUND_RED | FOREGROUND_INTENSITY);
  return std::cout;
}
#define RS_TITLE _RS_TITLE()
#else
#define RS_TITLE (std::cout << "\033[1m\033[35m")
#endif
#endif

#ifndef RS_MSG
#if defined(_WIN32)
inline std::ostream& _RS_MSG()
{
  setConsoleColor(FOREGROUND_GREEN | FOREGROUND_RED | FOREGROUND_BLUE | FOREGROUND_INTENSITY);
  return std::cout;
}
#define RS_MSG _RS_MSG()
#else
#define RS_MSG (std::cout << "\033[1m\033[37m")
#endif
#endif

#ifndef RS_REND
#if defined(_WIN32)
inline std::ostream& RS_REND(std::ostream& stream)
{
  stream << std::endl;
  setConsoleColor(-1);
  return stream;
}
#else
#define RS_REND "\033[0m" << std::endl
#endif
#endif
