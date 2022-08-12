#include "scan_to_cloud_converter/scan_to_cloud_converter.h"

int main (int argc, char **argv)
{
  ros::init (argc, argv, "ScanToCloudConverter");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  scan_tools::ScanToCloudConverter ltcc(nh, nh_private);
  ros::spin();
  return 0;
}
