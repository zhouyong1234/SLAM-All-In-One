#include "sbg_device.h"

using sbg::SbgDevice;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "sbg_device_mag");
  ros::NodeHandle node_handle;

  try
  {
    ROS_INFO("SBG DRIVER - Init node, load params and connect to the device");
    SbgDevice sbg_device(node_handle);

    sbg_device.initDeviceForMagCalibration();

    ros::spin();

    return 0;
  }
  catch (ros::Exception const& refE)
  {
    ROS_ERROR("SBG_DRIVER - [MagNode] Error - %s.", refE.what());
  }

  return 0;
}
