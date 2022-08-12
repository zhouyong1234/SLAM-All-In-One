//
// Created by kdq on 2021/6/15.
//
#pragma once
#include "time.h"
class SystemTimer {
 public:
  SystemTimer() {

  }
  static double now() {
    struct timespec ts;
    clock_gettime(CLOCK_BOOTTIME, &ts);
    return (ts.tv_sec * 1.0 + ts.tv_nsec * 1e-9);
  }

};
