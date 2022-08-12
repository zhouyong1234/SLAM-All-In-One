//
// Created by kdq on 2021/6/15.
//
#pragma once
#include "systemTimer.hpp"
#include "Config.hpp"
enum LogType {
  Info,
  Warning,
  Error
};
class FileSystem {
 public:
  FileSystem() {
  }
  static void printInfos(LogType type,std::string module,const char *fmt, ...) {
    if (fInfo == nullptr) {
      std::cout << "Info file is not exsit!" << std::endl;
      return;
    }
    std::string infoType = "";
    switch (type) {
      case Info: infoType = "I";break;
      case Warning: infoType = "W";break;
      case Error: infoType = "E";break;
      default: break;
    }
    va_list args;
    va_start(args,fmt);
    char buf[256];
    int len = vsnprintf(buf,256,fmt,args);
    if (len <= 0) {
      return;
    }
    std::string infos = '[' + std::to_string(SystemTimer::now()) + "]-[" + infoType + '|' + module + "]:" + buf + "\n";
    fprintf(fInfo,infos.c_str());
    fflush(fInfo);
  }
  static FILE * fInfo;
};

