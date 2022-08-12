^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package laser_scan_matcher
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.3.3 (2021-02-15)
------------------
* update to use non deprecated pluginlib macro

0.3.2 (2016-03-19)
------------------
* [feat] Publish Poses with Covariance (`#44 <https://github.com/ccny-ros-pkg/scan_tools/pull/44>`_)
* [sys] Remove csm cmake macro; csm is now built upstream since (`#31 <https://github.com/ccny-ros-pkg/scan_tools/pull/45>`_)
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
* [feat] Update gmapping demo as well.
* [feat] removed vgf filter, added xy filtering. added vgf on point cloud input for scan_matcher, vgf on point cloud input for scan_matcher
* [feat] removing pose2dstamped message type, now using posestamped
* [fix] PCL dependency Hydro onward
* [fix] rotation update check
* [fix] possible memory leak introduced by keyframe feature
* [doc] replaces tabs and fixes imu <-> odom topic comment
* [sys] cleaning
* [sys] Replace demo.vcg with a demo.rviz
* [sys] Small tweaks to CMakeLists.txt and package.xml files
* [sys] Catkinization: laser_scan_matcher (csm integrated as external project
* [sys] laser_scan_matcher: Remove CSM search in CMakeLists.txt
  Since the csm package exports the CFLAGS and LFLAGS, we do not need to
  explicitly search for the paths.
* [sys] launch files use rosbag api for fuerte
* [sys] imu topic renamed to imu/data
* Contributors: Daniel Axtens, Ivan Dryanovski, Kartik Mohta, Miguel Sarabia, Stephan Wirth, Enrique Fernández Perdomo
