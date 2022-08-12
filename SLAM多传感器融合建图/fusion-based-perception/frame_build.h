#pragma once
#include <string>
#include <list>
#include <tuple>
#include "input_data_type.h"
#include <boost/filesystem.hpp>
#include "fusion/object.h"

namespace proto_input
{
    enum FileTag
    {
        LOCATION,
        CAMERA,
        LIDAR,
        RADAR
    };
    std::list<std::tuple<double, FileTag, std::string> > buildSensorInputSequence(boost::filesystem::path data_dir, const char* data_name, FileTag tag);

    std::vector<std::tuple<double, FileTag, std::string> > frameTimeMatch(std::list<std::tuple<double, FileTag, std::string> >&& data, size_t type_num);

    std::vector<std::tuple<double, FileTag, std::string> > mergeInputSequence(std::vector<std::list<std::tuple<double, FileTag, std::string> > >&& data);

    kit::perception::fusion::LiDARObjectPtr makeLiDARObjectPtr(LidarObject raw_lo, double time);

    kit::perception::fusion::CameraObjectPtr makeCameraObjectPtr(CameraObject raw_co, double time);

}