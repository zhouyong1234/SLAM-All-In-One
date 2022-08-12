^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package scan_tools
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.3.3 (2021-02-15)
------------------
* First release for ROS Melodic, Noetic (supporting Ubuntu 18.04, 20.04)
* [maintenance] update to use non deprecated pluginlib macro

0.3.2 (2016-03-19)
------------------
* [feat][laser_scan_matcher] Publish Poses with Covariance (`#44 <https://github.com/ccny-ros-pkg/scan_tools/pull/44>`_)
* [sys][laser_scan_matcher] Remove csm cmake macro; csm is now built upstream since (`#31 <https://github.com/ccny-ros-pkg/scan_tools/pull/45>`_)
* Contributors: Eric Tappan, Isaac I.Y. Saito

0.3.1 (2015-12-18)
------------------
* [sys] Remove obsolete dependency. Alphabetize.
* Contributors: Isaac I.Y. Saito

0.3.0 (2015-11-10)
------------------
* [feat] Allow choosing between geometry_msgs/Twist and geometry_msgs/TwistStamped (fix `#21 <https://github.com/ccny-ros-pkg/scan_tools/issues/21>`_)
* [sys][laser_scan_matcher] Depends on DEB version of CSM; it is no longer built upon compile time
* [sys][laser_scan_matcher] Add simplest unit test
* [feat][laser_scan_matcher, demo.launch] Arg for whether to use RViz or not
* Contributors: Kei Okada, Jorge Santos Simón, Isaac I.Y. Saito

0.2.1 (2015-10-14)
------------------
* [feat] Released into ROS Indigo and Jade
* [feat] Update gmapping demo as well.
* [feat] added support for IMU message for orientation in laser_ortho_projector. The imu topic is imu/data* [feat] removed vgf filter, added xy filtering. added vgf on point cloud input for scan_matcher, vgf on point cloud input for scan_matcher
* [feat] removing pose2dstamped message type, now using posestamped
* [feat] added polar scan matcher
* [feat] added imu option to PSM, made CSM and PSM look similar
* [feat] added demo dir to CSM, 
* [fix] rotation update check
* [fix] possible memory leak introduced by keyframe feature
* [fix] PSM launch file
* [doc] updated manifest of scan_to_cloud_converter
* [doc] README files to contain catkin instructions
* [doc] replaces tabs and fixes imu <-> odom topic comment
* [sys] Catkinization
* [sys] Moved from robotics.ccny.cuny.edu
* [sys] ROS Hydro compatible
* [sys] correct license and references to PSM and CSM
* [sys] Small tweaks to CMakeLists.txt and package.xml files
* [sys] renamed skip to step parameter in scan sparsifier
* [sys] cleaning
* [sys] Replace demo.vcg with a demo.rviz
* [sys] Small tweaks to CMakeLists.txt and package.xml files
* [sys] laser_scan_matcher: Remove CSM search in CMakeLists.txt
  Since the csm package exports the CFLAGS and LFLAGS, we do not need to
  explicitly search for the paths.
* [sys] launch files use rosbag api for fuerte
* [sys] imu topic renamed to imu/data
* Contributors: Ivan Dryanovski, Miguel Sarabia, Daniel Axtens, Kartik Mohta, Miguel Sarabia, Stephan Wirth, Enrique Fernandez, Isaac I.Y. Saito
