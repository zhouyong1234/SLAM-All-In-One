#pragma once

#include <condition_variable>
#include <deque>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <thread>
#include <tuple>

#include <google/protobuf/message_lite.h>

#include "Recorder.hpp"
#include "Replayable.hpp"
#include "Topic.hpp"
#include "replaykit.pb.h"

namespace zz {
namespace replaykit {

template <typename Topics, typename Commands>
class ReplayKit {
 public:
  using TopicsType = Topics;
  using CommandsType = Commands;

 public:
  ReplayKit() { InitTopic(); }
  void EnableRecord() { has_record_enabled_ = true; }
  void DisableRecord() { has_record_enabled_ = false; }

  double timestamp_ = 0.0;

  double starttime_ = 0.0;

  void SetStarttime(double timestamp) { starttime_ = timestamp; }

  template <int i = 0, /* */
            typename std::enable_if<i<Topics::num_topics_, int>::type = 0> void
                InitTopic() {
    ::std::get<i>(topics_).topic_id_ = i;
    InitTopic<i + 1>();
  }

  template <int i,
            typename std::enable_if<i >= Topics::num_topics_, int>::type = 0>
  void InitTopic() {}

  template <int i>
  bool Publish(const double timestamp,
               const typename ::std::tuple_element<
                   i, typename Topics::topic_types_>::type& data,
               ::std::function<void(void)> on_processed = nullptr) {
    try {
      ::std::lock_guard<::std::mutex> lock(topic_queue_mutex_);
      auto replayable = ReplayableFactory::Create(timestamp, data, ::std::get<i>(topics_));
      replayable->on_processed = on_processed;
      replay_queue_.push_back(replayable);
    } catch (const std::exception& e) {
      std::cerr << e.what() << '\n';
    }
    topic_condition_variable_.notify_one();
    return true;
  }

  template <int i>
  bool PublishBuffer(const double timestamp, const char* buffer,
                     const int size) {
    ::replaykit::Buffer buf;
    buf.set_buffer(buffer, size);
    Publish<i>(timestamp, buf);
    return true;
  }

  template <int i>
  void Subscribe(
      const std::function<void(const double,
                               const typename ::std::tuple_element<
                                   i, typename Topics::topic_types_>::type&)>&
          subscriber) {
    ::std::get<i>(topics_).subscriber_ = subscriber;
  }

  void Start() {
    bool running = true;
    while (running) {
      ::std::shared_ptr<Replayable> replayable;
      {
        ::std::unique_lock<::std::mutex> lock(topic_queue_mutex_);
        topic_condition_variable_.wait(lock, [this]() { return !replay_queue_.empty(); });
        replayable = replay_queue_.front();
        replay_queue_.pop_front();
      }

      if (replayable->timestamp_ < starttime_) {
        printf("Skip repalyable %+7.3f\n", replayable->timestamp_);
        continue;
      }

      timestamp_ = replayable->timestamp_;

      replayable->Execute();

      if (replayable->on_processed) {
        replayable->on_processed();
      }

      if (has_record_enabled_ && recorder_ != nullptr) {
        replayable->Record(recorder_);
      }
    }
  }

  double Now() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (ts.tv_sec * 1.0 + ts.tv_nsec * 1e-9);
  }

  void SetRecorder(const ::std::shared_ptr<Recorder> recorder) {
    recorder_ = recorder;
  }

 public:
  const int num_topics_ = Topics::num_topics_;
  const int num_commanders_ = Commands::num_commanders_;

 protected:
  ::std::shared_ptr<Recorder> recorder_ = nullptr;
  bool has_record_enabled_ = false;
  typename Topics::topics_type_ topics_;

  ::std::deque<::std::shared_ptr<Replayable>> replay_queue_;
  ::std::mutex topic_queue_mutex_;
  ::std::condition_variable topic_condition_variable_;
  ::std::shared_ptr<::std::thread> recorder_thread_;
  bool is_running_ = false;
};

}  // namespace replaykit
}  // namespace zz
