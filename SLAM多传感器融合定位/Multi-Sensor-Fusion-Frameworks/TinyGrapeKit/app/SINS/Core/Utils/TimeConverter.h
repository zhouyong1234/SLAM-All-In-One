#pragma once

namespace SINS {

/// Copy from apollo
class TimeUtil {
 public:
  // @brief: UNIX timestamp to GPS timestamp, in seconds.
  static double Unix2Gps(double unix_time) {
    double gps_time = unix_time - UNIX_GPS_DIFF;
    if (unix_time < LEAP_SECOND_TIMESTAMP) {
      gps_time -= 1.0;
    }
    return gps_time;
  }

  // @brief: GPS timestamp to UNIX timestamp, in seconds.
  static double Gps2Unix(double gps_time) {
    double unix_time = gps_time + UNIX_GPS_DIFF;
    if (unix_time + 1 < LEAP_SECOND_TIMESTAMP) {
      unix_time += 1.0;
    }
    return unix_time;
  }

 private:
  // unix timestamp(1970.01.01) is different from gps timestamp(1980.01.06)
  static const int UNIX_GPS_DIFF = 315964782;
  // unix timestamp(2016.12.31 23:59:59(60) UTC/GMT)
  static const int LEAP_SECOND_TIMESTAMP = 1483228799;
};

}  // namespace SINS