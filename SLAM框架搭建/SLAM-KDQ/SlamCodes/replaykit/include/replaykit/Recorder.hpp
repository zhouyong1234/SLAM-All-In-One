#pragma once

#include <fstream>
#include <iostream>
#include <string>

namespace zz {
namespace replaykit {

class Recorder {
 public:
  virtual void Record(const uint32_t type, const double timestamp,
                      const void *buf, size_t len) = 0;

 protected:
  uint64_t seq_num_ = 0;
};

class FileRecorder : public Recorder {
 public:
  explicit FileRecorder(const std::string &path);
  virtual void Record(const uint32_t type, const double timestamp,
                      const void *buf, size_t len);

  ~FileRecorder();

 protected:
  ::std::ofstream file_;
  ::std::string path_;

  void OpenFile();
  bool IsFileExists(const ::std::string &path) const;
};

}  // namespace replaykit
}  // namespace zz
