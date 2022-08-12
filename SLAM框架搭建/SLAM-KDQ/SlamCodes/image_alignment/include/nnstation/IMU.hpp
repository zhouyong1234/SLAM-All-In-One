#pragma once

#include "NanoStation.hpp"
#include "rovio.pb.h"

namespace nnstation {

class ImuClient : public NanoClient<rovio::InputInfoPack> {
 public:
  typedef NanoClient<rovio::InputInfoPack> Base;
  typedef Base::mtParsed mtParsed;
  using Base::connect;
  using Base::msgLen;
  using Base::startRecv;
  using Base::subscribe;
  using Base::getOldestReplayTime;

  ImuClient() = default;

  ~ImuClient() override = default;

 private:
  bool parseData(char *pMsg, size_t len, mtParsed &parsed) override {
    return parsed.ParseFromArray(pMsg, len);
  }
};

class ImuServer : NanoServer<rovio::InputInfo> {
 public:
  typedef NanoServer<rovio::InputInfo> Base;
  typedef Base::mtParsed mtParsed;
  using Base::bind;
  using Base::close;

  ImuServer() = default;

  ~ImuServer() override = default;

  bool send(const mtParsed &parsed) override {
    return sendMsg(parsed.SerializeAsString().c_str(), static_cast<size_t>(parsed.ByteSize()));
  }
};

}