#include <fstream>
#include <iostream>
#include <sys/stat.h>

#include <replaykit.pb.h>
#include <replaykit/Recorder.hpp>

namespace zz {
namespace replaykit {

using ::std::ios;

/**
 * @todo 未来的改进方向包括，自动添加后缀打开一个新的文件.
 */
FileRecorder::FileRecorder(const std::string &path) : path_(path) {
  OpenFile();
}

void FileRecorder::Record(const uint32_t type, const double timestamp,
                          const void *buf, size_t len) {
  if (!file_.is_open()) {
    OpenFile();
  }

  if (!file_.is_open()) {
    return;
  }

  ::replaykit::ReplayItem item;
  item.set_seq(seq_num_++);
  item.set_type(type);
  item.set_timestamp(timestamp);
  item.set_buffer(buf, len);
  int serialized_len = item.ByteSize();
  file_.write(reinterpret_cast<char *>(&serialized_len),
              sizeof(serialized_len));
  file_ << item.SerializeAsString();
  file_.flush();
}

void FileRecorder::OpenFile() {
  if (!IsFileExists(path_)) {
    file_.open(path_, ios::in | ios::out | ios::ate | ios::app | ios::binary);
    ::std::cout << "Record replay log to file " << path_ << ::std::endl;
    return;
  }

  for (int i = 1; i < 100; i++) {
    ::std::string log_path = path_ + "." + ::std::to_string(i);
    if (!IsFileExists(log_path)) {
      file_.open(log_path, ios::in | ios::out | ios::ate | ios::app | ios::binary);
      ::std::cout << "Record replay log to file " << log_path << ::std::endl;
      return;
    }
  }
}

bool FileRecorder::IsFileExists(const ::std::string &path) const {
  struct stat file_stat;
  return stat(path.c_str(), &file_stat) == 0;
}

FileRecorder::~FileRecorder() {
  if (file_.is_open()) {
    file_.close();
  }
}

}  // namespace replaykit
}  // namespace zz
