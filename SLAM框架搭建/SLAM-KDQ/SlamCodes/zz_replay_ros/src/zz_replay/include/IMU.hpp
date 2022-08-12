#pragma once

#include "NanoStation.hpp"
#include "rovio.pb.h"
#include "Struct_definition.hpp"

namespace nnstation {

struct TimedImuDataPack : std::vector<TimedImuData> {
  double t;
};

class ImuClient : public NanoClient<TimedImuDataPack> {
 public:
  typedef NanoClient<TimedImuDataPack> Base;
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
    parsed.clear();
    rovio::InputInfoPack info_pack;
    info_pack.ParseFromArray(pMsg, static_cast<int>(len));
    for (size_t i = 0;i<info_pack.info_size();i++) {
      mtParsed::value_type element{};
      element.t = info_pack.info(i).t();
      element.gyr.x() = info_pack.info(i).gyr().x();
      element.gyr.y() = info_pack.info(i).gyr().y();
      element.gyr.z() = info_pack.info(i).gyr().z();
      element.acc.x() = info_pack.info(i).acc().x();
      element.acc.y() = info_pack.info(i).acc().y();
      element.acc.z() = info_pack.info(i).acc().z();
      element.init_quat.w() = info_pack.info(i).quat().w();
      element.init_quat.x() = info_pack.info(i).quat().x();
      element.init_quat.y() = info_pack.info(i).quat().y();
      element.init_quat.z() = info_pack.info(i).quat().z();
      element.init_vel.x() = 0.;
      element.init_vel.y() = 0.;
      element.init_vel.z() = 0.;
      element.proxi = info_pack.info(i).proxi();
//      printf("InputInfo: %11.6f, %7.3f,%7.3f,%7.3f, %7.3f,%7.3f,%7.3f, %7.4f,%7.4f,%7.4f,%7.4f\n",
//             parsed.t,
//             parsed.gyr[0], parsed.gyr[1], parsed.gyr[2],
//             parsed.acc[0], parsed.acc[1], parsed.acc[2],
//             parsed.init_quat.w(), parsed.init_quat.x(), parsed.init_quat.y(), parsed.init_quat.z());
      parsed.push_back(element);
      parsed.t = element.t;
    }
    return true;
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