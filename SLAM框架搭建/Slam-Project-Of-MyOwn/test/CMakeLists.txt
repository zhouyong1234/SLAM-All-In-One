CMAKE_MINIMUM_REQUIRED( VERSION 3.12.2 )

PROJECT( test )

ADD_SUBDIRECTORY( gridMapBase_test )

#ADD_SUBDIRECTORY( icp_test )

#ADD_SUBDIRECTORY( scan_match_test )

ADD_SUBDIRECTORY( slam_simulation_test )

#ADD_SUBDIRECTORY( scan_context_test )

#ADD_SUBDIRECTORY( slam_simulation_without_g2o )

ADD_SUBDIRECTORY( slam_simulation_with_g2o )

ADD_SUBDIRECTORY( g2o_test )

#ADD_SUBDIRECTORY( load_map_test )

#ADD_SUBDIRECTORY( localization_test )
