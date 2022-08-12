#pragma once

#include <google/protobuf/message.h>
#include <functional>
#include <memory>
#include <type_traits>

#include "Recorder.hpp"
#include "Topic.hpp"

namespace zz {
namespace replaykit {

class Replayable {
 public:
  double timestamp_;

  Replayable(const float timestamp) : timestamp_(timestamp) {}

  virtual void Execute() = 0;

  virtual void Record(const ::std::shared_ptr<Recorder> recorder){};

  virtual ~Replayable() {}

  ::std::function<void(void)> on_processed = nullptr;
};

template <typename TopicType, typename T>
class ReplayableTopic : public Replayable {
 public:
  ReplayableTopic(const float timestamp, const T &v, const TopicType &topic)
      : Replayable(timestamp), topic_(topic), value_(v) {}

  virtual void Execute() {
    if (!topic_.subscriber_) {
      return;
    }
    topic_.subscriber_(timestamp_, value_);
  }

 protected:
  const TopicType &topic_;
  T value_;
};

template <typename T>
class ProtobufMessageReplayable : public ReplayableTopic<Topic<T>, T> {
 public:
  ProtobufMessageReplayable(const double timestamp, const T &v,
                            const Topic<T> &topic)
      : ReplayableTopic<Topic<T>, T>(timestamp, v, topic) {}

  virtual void Record(const ::std::shared_ptr<Recorder> recorder) {
    int size = this->value_.ByteSize();
    uint8_t buffer[size];
    this->value_.SerializeToArray(buffer, size);
    recorder->Record(this->topic_.topic_id_, this->timestamp_, (void *)buffer,
                     size);
  };

  static bool Parse(T &v, const char *buffer, int len) {
    return v.ParseFromArray(buffer, len);
  }

  ~ProtobufMessageReplayable() {}
};

template <typename T>
class PrimitiveReplayable : public ReplayableTopic<Topic<T>, T> {
 public:
  explicit PrimitiveReplayable(const double timestamp, const T &v,
                               const Topic<T> &topic)
      : ReplayableTopic<Topic<T>, T>(timestamp, v, topic) {}

  virtual void Record(const ::std::shared_ptr<Recorder> recorder) {
    recorder->Record(this->topic_.topic_id_, this->timestamp_,
                     (void *)&this->value_, sizeof(T));
  };

  static bool Parse(T &v, const char *buffer, int len) {
    if (len != sizeof(T)) {
      return false;
    }
    v = *reinterpret_cast<const T *>(buffer);
    return true;
  }

  ~PrimitiveReplayable() {}
};

class ReplayableFactory {
 public:
  template <typename T, typename ::std::enable_if<
                            std::is_base_of<::google::protobuf::Message, T>{},
                            int>::type = 0>
  static ::std::shared_ptr<Replayable> Create(const double timestamp,
                                              const T &t,
                                              const Topic<T> &topic) {
    return ::std::make_shared<ProtobufMessageReplayable<T>>(timestamp, t,
                                                            topic);
  }

  template <typename T, typename ::std::enable_if<::std::is_fundamental<T>{},
                                                  int>::type = 0>
  static ::std::shared_ptr<Replayable> Create(const double timestamp,
                                              const T &t,
                                              const Topic<T> &topic) {
    return ::std::make_shared<PrimitiveReplayable<T>>(timestamp, t, topic);
  }

  template <typename T, typename ::std::enable_if<
                            std::is_base_of<::google::protobuf::Message, T>{},
                            int>::type = 0>
  static bool Parse(T &v, const char *buffer, int len) {
    return ProtobufMessageReplayable<T>::Parse(v, buffer, len);
  }

  template <typename T, typename ::std::enable_if<::std::is_fundamental<T>{},
                                                  int>::type = 0>
  static bool Parse(T &v, const char *buffer, int len) {
    return PrimitiveReplayable<T>::Parse(v, buffer, len);
  }
};

}  // namespace replaykit
}  // namespace zz
