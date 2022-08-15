# hdl_global_localization

![hdl_global_localization](https://user-images.githubusercontent.com/31344317/105116113-71fc6180-5b0d-11eb-9d85-bbea922dde84.gif)

[![Build Status](https://travis-ci.org/koide3/hdl_global_localization.svg?branch=master)](https://travis-ci.org/koide3/hdl_global_localization) on ROS melodic and noetic

## Requirements
***hdl_global_localization*** requires the following libraries:
- PCL
- OpenCV
- OpenMP
- Teaser++ [optional]

## Services

- ***/hdl_global_localization/set_engine*** (hdl_global_localization::SetGlobalLocalizationEngine)
  - Available global localization engines: BBS, FPFH_RANSAC, FPFH_TEASER
- ***/hdl_global_localization/set_global_map*** (hdl_global_localization::SetGlobalMap)
- ***/hdl_global_localization/query*** (hdl_global_localization::QueryGlobalLocalization)


## Algorithms

- 2D Grid Map-based Branch-and-Bound Search
  - Real-time loop closure in 2D LIDAR SLAM, ICRA, 2016
- FPFH + RANSAC (based on pcl::SampleConsensusPrerejective)
  - Fast Point Feature Histograms (FPFH) for 3D registration, ICRA, 2009
  - Pose Estimation using Local Structure-Specific Shape and Appearance Context, ICRA, 2013
- FPFH + Teaser++
  - TEASER: Fast and Certifiable Point Cloud Registration, T-RO, 2020

## Related packages

- [interactive_slam](https://github.com/koide3/interactive_slam)
- [hdl_graph_slam](https://github.com/koide3/hdl_graph_slam)
- [hdl_localization](https://github.com/koide3/hdl_localization">hdl_localization)
- [hdl_people_tracking](https://github.com/koide3/hdl_people_tracking">hdl_people_tracking)

## Contact
Kenji Koide, k.koide@aist.go.jp

Human-Centered Mobility Research Center, National Institute of Advanced Industrial Science and Technology (AIST), Japan
