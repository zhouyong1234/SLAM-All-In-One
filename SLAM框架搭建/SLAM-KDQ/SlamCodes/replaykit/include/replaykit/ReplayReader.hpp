#pragma once

#include <stdio.h>
#include <unistd.h>
#include <fstream>
#include <iostream>
#include <map>
#include <string>
#include <thread>
#include <type_traits>

#include <google/protobuf/io/zero_copy_stream_impl.h>

#include "ReplayKit.hpp"
#include "replaykit.pb.h"

namespace zz {
namespace replaykit {

class ReplayReaderBase {
 public:
  void SetReplaySpeed(float replay_speed);
  void SetStartTime(double starttime) { starttime_ = starttime; }

  virtual void Start() = 0;

  void StartAsync() {
    read_thread_ = ::std::thread([this]() { Start(); });
  }

  virtual void Publish(const ::replaykit::ReplayItem &item) = 0;

  void AddStarttimeFilterType(int type) {
    starttime_filter_types_[type] = type;
  }

  void AddFilterType(int type) { filter_types_[type] = type; }

  ::std::map<int, int> starttime_filter_types_;
  ::std::map<int, int> filter_types_;

 protected:
  float replay_speed_ = -1;
  double starttime_ = -1;
  ::std::thread read_thread_;
};

template <typename ReplayKit>
class ReplayReader : public ReplayReaderBase {
 public:
  static constexpr double MIN_REPLAY_SPEED = 0.1;
  double replay_speed_ = 1.0;
  double last_timestamp_ = 0.0;
  double current_timestamp_ = -1;
  static constexpr int num_topics_ = ReplayKit::TopicsType::num_topics_;

  ReplayReader(ReplayKit &replaykit, double speed = 1.0)
      : replaykit_(replaykit) {
    if (speed >= 0) {
      replay_speed_ = speed > MIN_REPLAY_SPEED ? speed : MIN_REPLAY_SPEED;
    } else {
      replay_speed_ = speed;
    }
  }

  virtual void Publish(const ::replaykit::ReplayItem &item) {
    current_timestamp_ = item.timestamp();

    if (filter_types_.count(item.type()) > 0) {
      return;
    }

    if (current_timestamp_ < starttime_ &&
        starttime_filter_types_.count(item.type()) == 0) {
      last_timestamp_ = current_timestamp_;
      return;
    }
    if (replay_speed_ > 0 && last_timestamp_ > 0) {
      int32_t waittime_us =
          (1 / replay_speed_) * (current_timestamp_ - last_timestamp_) * 1e6;
      if (waittime_us < 0) {
        waittime_us = 0;
      }
      usleep(waittime_us);
    }
    PublishToTopic<0>(item);
    last_timestamp_ = current_timestamp_;
  }

  template <int i = 0, /* */
            typename std::enable_if<i<num_topics_, int>::type = 0>
            //
            void PublishToTopic(const ::replaykit::ReplayItem &item) {
    if (i == item.type()) {
      typename ::std::tuple_element<
          i, typename ReplayKit::TopicsType::topic_types_>::type v;
      ::std::string buffer = item.buffer();
      if (ReplayableFactory::Parse(v, buffer.c_str(), buffer.size())) {
        replaykit_.template Publish<i>(item.timestamp(), v);
      }
      return;
    }

    PublishToTopic<i + 1>(item);
  }

  template <int i, typename std::enable_if<i >= num_topics_, int>::type = 0>
  void PublishToTopic(const ::replaykit::ReplayItem &item) {}

 protected:
  ReplayKit &replaykit_;
};

template <typename ReplayKit>
class FileReplayReader : public ReplayReader<ReplayKit> {
 public:
  FileReplayReader(const std::string path, ReplayKit &replaykit,
                   double speed = 1.0)
      : path_(path), ReplayReader<ReplayKit>(replaykit, speed) {
    OpenFile();
  }

  bool OpenFile() {
    replay_log_file_.open(path_);
    if (!replay_log_file_.is_open()) {
      printf("FileReplayReader::OpenFile(): %s cannot be opened!\n", path_.c_str());
      return false;
    }
    return true;
  }

  virtual void Start() {
    if (!replay_log_file_.is_open()) {
      printf("FileReplayReader::Start(): %s is not opened yet!\n", path_.c_str());
      return;
    }

    while (true) {
      if (replay_log_file_.eof()) {
        break;
      }

      int len = 0;
      replay_log_file_.read(reinterpret_cast<char *>(&len), sizeof(len));

      if (replay_log_file_.eof()) {
        break;
      }

      char buffer[len];
      replay_log_file_.read(buffer, len);
      ::replaykit::ReplayItem item;
      bool succeed = item.ParseFromArray(buffer, len);

      this->Publish(item);

      if (!succeed) {
        break;
      }
    }
  }

 protected:
  ::std::string path_;
  ::std::ifstream replay_log_file_;
};

}  // namespace replaykit
}  // namespace zz
