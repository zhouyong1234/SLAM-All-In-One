#pragma once
#include <map>
#include <stdio.h>
#include <sys/time.h>
#include <string>

class Tictoc {
 public:
  Tictoc(std::string set_name) : name(set_name) {}

  virtual ~Tictoc() {}

  void tic() {
    gettimeofday(&time, NULL);
    time1 = time.tv_sec * 1000.0 + (time.tv_usec / 1000.0);
    n = 0;
  }

  double toc() {
    n++;
    gettimeofday(&time, NULL);
    time2 = time.tv_sec * 1000.0 + (time.tv_usec / 1000.0);
    return time2 - time1;
  }

  void print() {
    n++;
    gettimeofday(&time, NULL);
    time2 = time.tv_sec * 1000.0 + (time.tv_usec / 1000.0);
    printf("%s.%d %9.3f\n", name.c_str(), n, time2 - time1);
  }

  void print_tic() {
    n++;
    gettimeofday(&time, NULL);
    time2 = time.tv_sec * 1000.0 + (time.tv_usec / 1000.0);
    printf("%s.%d %9.3f\n", name.c_str(), n, time2 - time1);
    time1=time2;
  }
  void print_tic(const char* str) {
    n++;
    gettimeofday(&time, NULL);
    time2 = time.tv_sec * 1000.0 + (time.tv_usec / 1000.0);
    printf("%s.%s %9.3f\n", name.c_str(), str, time2 - time1);
    time1=time2;
  }
 private:
  struct timeval time;
  std::string name;
  int n = 0;
  double time1;
  double time2;
};

class AdvanceTimer {
 public:
  AdvanceTimer(): timer("timer") {
    timer.tic();
  }
  void toc(std::string name) {
    costMs_[name] = timer.toc();
    timer.tic();
  }
  double costMs(std::string name) {
    if (costMs_.count(name)) {
      return costMs_[name];
    }
    return 0;
  }
 private:
  Tictoc timer;
  std::map<std::string,double> costMs_;
};
