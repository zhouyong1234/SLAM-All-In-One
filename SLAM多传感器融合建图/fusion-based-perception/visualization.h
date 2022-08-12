#pragma once
#include "fusion/object.h"
#include <sensor_msgs/PointCloud.h>
#include "input_data_type.h"

sensor_msgs::PointCloud transToPointCloud(const kit::perception::fusion::FusionObjectList& data);
sensor_msgs::PointCloud transToPointCloud(const kit::perception::fusion::LiDARObjectList& data);
