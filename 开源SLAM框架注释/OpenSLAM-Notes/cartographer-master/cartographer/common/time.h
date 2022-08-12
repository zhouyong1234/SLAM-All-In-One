/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef CARTOGRAPHER_COMMON_TIME_H_
#define CARTOGRAPHER_COMMON_TIME_H_
// 定义的一些时间操作的变量和函数
// chrono 时c++ 11 中的新特性，计时更为精确且使用更简单
#include <chrono>
#include <ostream>
#include <ratio>

#include "cartographer/common/port.h"

namespace cartographer {
namespace common {

constexpr int64 kUtsEpochOffsetFromUnixEpochInSeconds =
    (719162ll * 24ll * 60ll * 60ll);

struct UniversalTimeScaleClock {
  using rep = int64;
  using period = std::ratio<1, 10000000>;
  using duration = std::chrono::duration<rep, period>;
  using time_point = std::chrono::time_point<UniversalTimeScaleClock>;
  static constexpr bool is_steady = true;
};

// Represents Universal Time Scale durations and timestamps which are 64-bit
// integers representing the 100 nanosecond ticks since the Epoch which is
// January 1, 1 at the start of day in UTC.
using Duration = UniversalTimeScaleClock::duration;// 一段时间
using Time = UniversalTimeScaleClock::time_point;// 时刻

// Convenience functions to create common::Durations.
Duration FromSeconds(double seconds); // 给定秒数据，获得时间结构体
Duration FromMilliseconds(int64 milliseconds);// 给定ms数据，获得时间结构体

// Returns the given duration in seconds.
double ToSeconds(Duration duration);// 时间结构体转换为秒量级
double ToSeconds(std::chrono::steady_clock::duration duration);

// Creates a time from a Universal Time Scale.
Time FromUniversal(int64 ticks);

// Outputs the Universal Time Scale timestamp for a given Time.
int64 ToUniversal(Time time);

// For logging and unit tests, outputs the timestamp integer.
// 输出当前时间
std::ostream& operator<<(std::ostream& os, Time time);

// CPU time consumed by the thread so far, in seconds.
// 返回cpu处理当前线程的时间，单位为秒
double GetThreadCpuTimeSeconds();

}  // namespace common
}  // namespace cartographer

#endif  // CARTOGRAPHER_COMMON_TIME_H_
