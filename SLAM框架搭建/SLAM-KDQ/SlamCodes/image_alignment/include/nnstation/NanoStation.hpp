/**
 * @file NanoStation.hpp
 * @author  Zhang Haiyang <zhanghaiyang@zerozero.cn>
 * @version 1.1.1
 * @section DESCRIPTION
 * This provides a nanomsg client and server template.
 *
 * @section Release Notes
 * v1.0 Initial version.
 * v1.1 Fix replay option bug. By default, parse as protobuf.
 * v1.1.1 Change parse funtion to pure virtual.
 *
 */

#pragma once

#include <unistd.h>
#include <iostream>
#include <fstream>
#include <mutex>
#include <queue>
#include <thread>
#include <string>
#include <functional>

#include <nanomsg/nn.h>
#include <nanomsg/pubsub.h>

namespace nnstation {

template<typename PARSED>
class NanoClient {
 public:
  typedef PARSED mtParsed;
  typedef std::function<void(const mtParsed&)> onRecv;

  NanoClient() : fd_(-1), msgLen_(-1) {};

  virtual ~NanoClient() = default;

  inline int &msgLen() {
    return msgLen_;
  }

  inline const int &msgLen() const {
    return msgLen_;
  }

  void subscribe(onRecv func) {
    dataCallback = func;
  }

  bool connect(const std::string &url) {
    url_ = url;
    fd_ = nn_socket(AF_SP, NN_SUB);
    if (fd_ < 0) {
      fprintf(stderr, "nn_socket: %s\n", nn_strerror(nn_errno()));
      return false;
    }
    if (nn_connect(fd_, url_.c_str()) < 0) {
      fprintf(stderr, "nn_socket: %s\n", nn_strerror(nn_errno()));
      nn_close(fd_);
      return false;
    }
    /*  We want all messages, so just subscribe to the empty value. */
    if (nn_setsockopt(fd_, NN_SUB, NN_SUB_SUBSCRIBE, "", 0) < 0) {
      fprintf(stderr, "nn_setsockopt: %s\n", nn_strerror(nn_errno()));
      nn_close(fd_);
      return false;
    }
    return true;
  }

  bool enableRecord(const std::string &file) {
    ofs_.open(file);
    recordEnabled_ = ofs_.is_open();
    return recordEnabled_;
  }

  void startRecv() {
    running_ = true;
    recvThread_ = std::thread(&NanoClient::recvLoop, this);
  }

  bool getOldestReplayTime(const std::string &file, double &t) {
    std::ifstream ifs;
    ifs.open(file);
    if (!ifs.good() || ifs.eof()) {
      ifs.close();
      return false;
    }
    double replay_t = 0.;
    ifs.read((char*)(&replay_t), sizeof(replay_t));
    t = replay_t;
    return true;
  }

  bool startReplay(const std::string &file, 
                   const double &start_real_t,
                   const double &start_replay_t) {
    start_real_t_ = start_real_t;
    start_replay_t_ = start_replay_t;
    ifs_.open(file);
    if (!ifs_.good()) {
      running_ = false;
      return false;
    }
    running_ = true;
    replayThread_ = std::thread(&NanoClient::replayLoop, this);
  }

  virtual bool parseData(char *pMsg, size_t len, mtParsed &parsed) = 0;

 private:
  onRecv dataCallback;
  int fd_;
  std::string url_;
  int msgLen_;
  std::thread recvThread_, replayThread_;
  bool running_ = false;
  bool recordEnabled_ = false;
  std::ofstream ofs_;
  std::ifstream ifs_;
  double start_real_t_{};
  double start_replay_t_{};

  void recvLoop() {
    mtParsed parsed;
    while (running_) {
      char *pMsg = nullptr;
      int rc = nn_recv(fd_, &pMsg, NN_MSG, 0);
      if (rc < 0) {
        fprintf(stderr, "nn_recv: %s\n", nn_strerror(nn_errno()));
        nn_freemsg(pMsg);
        continue;
      }
      if (msgLen_ > 0 && rc != msgLen_) {
        fprintf(stderr, "nn_recv: got %d bytes, wanted %d\n", rc, msgLen_);
        nn_freemsg(pMsg);
        continue;
      }
      double current_t = 0.;
      if (parseData(pMsg, rc, parsed)) {
        current_t = std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
        dataCallback(parsed);
      }
      if (recordEnabled_) {
        ofs_.write((const char*)(&current_t), sizeof(current_t));
        ofs_.write((const char*)(&rc), sizeof(rc));
        ofs_.write((const char*)pMsg, size_t(rc));
      }
      nn_freemsg(pMsg);
    }
  }

  void replayLoop() {
    mtParsed parsed;
    while (running_) {
      if (ifs_.eof()) {
        break;
      }
      double replay_t = 0.;
      ifs_.read((char *) (&replay_t), sizeof(replay_t));
      if (replay_t < start_replay_t_) {
        int len = 0;
        ifs_.read((char *) (&len), sizeof(len));
        char buffer[len];
        ifs_.read(buffer, len);
        continue;
      }
      auto current_t = std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
      double wait_t = (replay_t - start_replay_t_) - (current_t - start_real_t_);
      if (wait_t > 0) {
        usleep(static_cast<uint32_t>(wait_t * 1e6));
      }
      int len = 0;
      ifs_.read((char *) (&len), sizeof(len));
      char buffer[len];
      ifs_.read(buffer, len);
      if (parseData(buffer, len, parsed)) {
        dataCallback(parsed);
      }
    }
  }
};

/**
 * @brief Basic nanomsg server. Blocked version(Currently).
 * @tparam PARSED
 */
template<typename PARSED>
class NanoServer {
 public:
  typedef PARSED mtParsed;

  NanoServer() : fd_(-1) {};

  virtual ~NanoServer() = default;

  bool bind(const std::string &url) {
    url_ = url;
    fd_ = nn_socket(AF_SP, NN_PUB);
    if (fd_ < 0) {
      fprintf(stderr, "nn_socket: %s\n", nn_strerror(nn_errno()));
      return false;
    }
    if (nn_bind(fd_, url_.c_str()) < 0) {
      fprintf(stderr, "nn_bind: %s\n", nn_strerror(nn_errno()));
      nn_close(fd_);
      return false;
    }
    return true;
  }

  void close() {
    nn_close(fd_);
  }

  bool sendMsg(const void *pMsg, size_t len) {
    int rc = nn_send(fd_, pMsg, len, 0);
    if (rc < 0) {
      fprintf(stderr, "nn_send: %s (ignoring). rc = %d\n", nn_strerror(nn_errno()), rc);
      return false;
    }
    return true;
  }

  virtual bool send(const mtParsed &parsed) = 0;

 private:
  int fd_;
  std::string url_;
};

}