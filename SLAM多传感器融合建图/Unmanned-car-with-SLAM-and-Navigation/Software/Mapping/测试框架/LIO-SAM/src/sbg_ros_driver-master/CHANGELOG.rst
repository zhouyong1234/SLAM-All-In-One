^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package sbg_driver
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

3.0.0 (2021-08-31)
------------------
* Merge pull request `#62 <https://github.com/SBG-Systems/sbg_ros_driver/issues/62>`_ from SBG-Systems/devel
  Devel
* Merge pull request `#61 <https://github.com/SBG-Systems/sbg_ros_driver/issues/61>`_ from SBG-Systems/fix-frame-convention
  Fix frame convention errors
* Make sure only delta angles/velocities are used
  + syntax fixes
* Improved comments & README file
* Fix missing checks on UTC / Unix time computation
* Updated sbgECom messages definitions
* Update README.md and releasePackage.md
* Clean up readme
* Fix position accuracy
* Replace pow by multiplication
* Complete README.md
* Fix bugs in message wrapper and complete message definitions
* Fix Z GPS velocity
* Fix frame convention errors
* Merge pull request `#59 <https://github.com/SBG-Systems/sbg_ros_driver/issues/59>`_ from SBG-Systems/fix-frame-id
  Fix param name
* Fix param name
* Fix inconsistent tab and spaces usages.
  Now all files are using spaces
* Add wrap 2PI
* Fix bugs
* Fix matrix accuracy
* Fix course
* wrapper: Fix ENU->NED for GPS messages
* warpper: Change createrVector3 names
* wrapper: Fix setter
* wrapper: Fix missing ENU conversion an add comment for cov
* wrapper: Fix regresion
* coding style : Remove tables
* publisher: Configure the wrapper before init the publisher
* main: Fix loopFrequency type
* config: Restore output configuration
* config: Replace enu by use_enu
* enu: Replace enu by use_enu and rename frame convertion function
* frame_id: Change default value
* clean up
* `#51 <https://github.com/SBG-Systems/sbg_ros_driver/issues/51>`_ - Correctly fill convariances parameters
* project: Update maintainer
* config: Update parameters
* `#45 <https://github.com/SBG-Systems/sbg_ros_driver/issues/45>`_-`#50 <https://github.com/SBG-Systems/sbg_ros_driver/issues/50>`_ - Add parameters to set frame ID and ENU convention
* `#48 <https://github.com/SBG-Systems/sbg_ros_driver/issues/48>`_ - # 52 Add a parameter to select header stamp source and read ROS time when publishing the message
* `#47 <https://github.com/SBG-Systems/sbg_ros_driver/issues/47>`_ Remove node ros::Rate period auto computation and only read it from a node parameter
* clean up: Remove blank spaces
* Merge pull request `#35 <https://github.com/SBG-Systems/sbg_ros_driver/issues/35>`_ from ShepelIlya/patch-1
  Fixed time_stamp value initializing in SbgEkfNavMessage.
* Added/updated license to MIT
* Updated maintainer
* Fixed timeStamp value initializing in SbgEkfNavMessage.
* Contributors: Michael Zemb, Raphael Siryani, ShepelIlya, bsaussay, rsiryani

2.0.3 (2020-04-01)
------------------
* Improve matrix handling
* Fix body velocity computation (Issue `#31 <https://github.com/SBG-Systems/sbg_ros_driver/issues/31>`_)

2.0.1 (2020-01-24)
------------------
* Improve vector handling
* Fix include file (Issue `#26 <https://github.com/SBG-Systems/sbg_ros_driver/issues/26>`_)

2.0.0 (2020-01-06)
------------------
* Fix integer type
* Update sbgECom messages (AirData, ImuShort)
* Update sbgECom library to 1.11.920-stable
* Improve numeric type
* Improve configuration applier
* Improve error handling
* Code improvement
* Improve device configuration
* Update changelog
* Update and improve README.md
* Update magnetic services
* Improve message timestamping
* Add some ROS standard sensor messages (Issue `#17 <https://github.com/SBG-Systems/sbg_ros_driver/issues/17>`_)
* Comply file structure to ROS best pratices
* Add a processing time to improve message handling
* Add udev rules to documentation (Issue `#21 <https://github.com/SBG-Systems/sbg_ros_driver/issues/21>`_)
* Improve magnetometers calibration
* Update maintainer of the package (Issue `#20 <https://github.com/SBG-Systems/sbg_ros_driver/issues/20>`_)
* Enable/Disable the configuration of the device (Issue `#19 <https://github.com/SBG-Systems/sbg_ros_driver/issues/19>`_)
* Define unified class and launch files for all SBG devices
* Define classes for device configuration
* Merge pull request `#18 <https://github.com/SBG-Systems/sbg_ros_driver/issues/18>`_ from SBG-Systems/messagePublisherRework
* Integrate new message publisher to the Ellipse class (Issue `#15 <https://github.com/SBG-Systems/sbg_ros_driver/issues/15>`_)
* Define a class to publish messages
* Define class to wrap SBG logs to Ros messages
* Merge pull request `#16 <https://github.com/SBG-Systems/sbg_ros_driver/issues/16>`_ from SBG-Systems/v4.3
* [src] Update SDK version + add LogE support
* Merge pull request `#13 <https://github.com/SBG-Systems/sbg_ros_driver/issues/13>`_ from nicolaje/remove-non-ascii-char
* [conf] Removed non-ASCII characters, (Issue `#8 <https://github.com/SBG-Systems/sbg_ros_driver/issues/8>`_)
* [msg] Remove non ascii characters

1.1.7 (2018-07-19)
------------------
* [src] Change SbgEkfEuler comments
* [src] Move .h to include folder + test new method for time saving in calib

1.1.6 (2018-03-18)
------------------
* [config, src] Update default port for gps aiding (Ellipse-E) + add save & reboot for mag calibration
* [build] Add include for debian jessie arm64 build issue

1.1.5 (2018-03-12 23:49)
------------------------
* [src] Update mag calibration

1.1.4 (2018-03-12 23:10)
------------------------
* [catkin] Update install launch & config
* [src] Update library + Correction bugs

1.1.3 (2018-03-12 11:46)
------------------------
* Update dependencies to std_srvs

1.1.2 (2018-03-12 09:54)
------------------------
* [ChangeLog] Remove
* [ChangeLog] Update
* [Changelog] Test
* [test] Changelog
* [Changelog] Update
* [CMake] Correction of message dependency

1.1.1 (2018-03-11)
------------------
* [xml] Update version number
* [src] Correction of small bugs + add publisher only on activated log
* [merge] Finalize merge from devel branch (master divergence issue)
* [lib] Update the library sbgECom version after merging from devel
* [Merge]
* Merge branch 'master' of https://github.com/ENSTABretagneRobotics/sbg_ros_driver
* [src] Update doc
* [src] Update magnetic calibration node
* Revert "1.0.7"
  This reverts commit 8f57f9e578937ac23383e39ebf616d1039384b09.
* Update README
* Merge pull request `#2 <https://github.com/SBG-Systems/sbg_ros_driver/issues/2>`_ from rpng/master
  Upgrade sbg_ros_driver
* Moved the logging function into the class
* Added - Start of heading code
* Refactor and added new publishers
* Increased rates
* refactoring
  use a class for callbacks
  changed callbacks around a bit, now shows raw data rather than ekf logs
* use gps log message for NavSatFix message
* add extra debug messages
* use private namespace
* modified launch file
  moved to launch folder and added optional arguments
* updated sbgECom library

1.1.0 (2018-03-10)
------------------
* [src] Update Events
* [src] Add params
* [src] Update (add configuration of the ellipse)
* [src] Update messages
* [src] Start creating sbg messages

1.0.7 (2017-04-01)
------------------
* [src][minor] Correct launch file

1.0.6 (2017-03-31)
------------------
* CHANGELOG
* [src] Add launch example
* [src] Change imu data & add gyroscopes

1.0.5 (2016-11-17 00:04)
------------------------

1.0.4 (2016-11-17 00:02)
------------------------

1.0.3 (2016-11-16 23:59)
------------------------
* [src][minor] Correction of Project name in CmakeList

1.0.2 (2016-11-16 22:58)
------------------------
* [doc] minor
* [doc] Update Package

1.0.1 (2016-11-16 22:30)
------------------------
* [doc] Update package version to 1.0.0
* [doc] Add Changelog
* [src] Update of deprecated function
* [src] Update (correcting cmake sub project)
* [src] Correct cmake subdirectory issue
* Initial commit
