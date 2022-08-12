cmake_minimum_required(VERSION 2.8.3)
project(live_slam_player)

add_definitions(-std=c++14)

if (NOT CMAKE_BUILD_TYPE)
  set (CMAKE_BUILD_TYPE release)
endif ()

find_package(catkin REQUIRED cmake_modules
  COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  message_generation
  rosbag
  image_transport
  cv_bridge
  dynamic_reconfigure
  pcl_ros
  pcl_conversions
  pcl_msgs
  irp_sen_msgs
  camera_info_manager
  tf
)
set(CMAKE_AUTOMOC ON)

find_package(Eigen REQUIRED)

#set (CMAKE_PREFIX_PATH /opt/Qt5.6.1/5.6/gcc_64/lib/cmake)

#find_package(Qt5Core)
find_package(Qt5Widgets)
find_package(Qt5Gui)
find_package(Qt5OpenGL)


add_message_files(
  DIRECTORY msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

generate_dynamic_reconfigure_options(
  cfg/live_slam_player.cfg
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
    live_slam_player
  CATKIN_DEPENDS 
    roscpp rospy 
    std_msgs 
    geometry_msgs 
    message_runtime 
    image_transport 
    cv_bridge 
    dynamic_reconfigure 
    pcl_ros
    pcl_conversions  
    pcl_msgs
    irp_sen_msgs
    camera_info_manager
    tf

  DEPENDS 
    Eigen
)


set (SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)

set (live_slam_player_QTLib_src ${SRC_DIR}/mainwindow.cpp ${SRC_DIR}/ROSThread.cpp)
set (live_slam_player_QTLib_hdr ${SRC_DIR}/mainwindow.h ${SRC_DIR}/ROSThread.h)
set (live_slam_player_QTLib_ui  ${SRC_DIR}/mainwindow.ui)
set (live_slam_player_QTBin_src ${SRC_DIR}/main.cpp)

#find_package(GLEW REQUIRED)
#find_package(GLUT REQUIRED)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${live_slam_player_INCLUDE_DIRS}
  ${SRC_DIR}
  ${Qt5Widgets_INCLUDE_DIRS}
  ${PROJECT_BINARY_DIR}
  include
  ${Eigen_INCLUDE_DIRS}
)

#qt5_wrap_cpp(File_Player_QTLib_hdr_moc ${File_Player_QTLib_hdr})
qt5_wrap_ui (live_slam_player_QTLib_ui_moc  ${live_slam_player_QTLib_ui})
qt5_add_resources(SHADER_RSC_ADDED ${SRC_DIR}/resources.qrc)

###########
## Build ##
###########

add_executable(live_slam_player ${live_slam_player_QTLib_src} ${live_slam_player_QTLib_hdr} ${live_slam_player_QTBin_src} ${SHADER_RSC_ADDED} ${live_slam_player_QTLib_ui_moc})

add_dependencies(live_slam_player live_slam_player_msgs_generate_messages_cpp ${PROJECT_NAME}_gencfg)
add_dependencies(live_slam_player ${catkin_EXPORTED_TARGETS})

target_link_libraries(live_slam_player

  ${catkin_LIBRARIES}
  ${QT_LIBRARIES} 
  ${OPENGL_LIBRARIES}
  Qt5::Widgets 
  Qt5::Gui 
  Qt5::OpenGL
  GL
  ${BOOST_CUSTOM_LIBS}
  ${Eigen_LIBRARIES}
)
