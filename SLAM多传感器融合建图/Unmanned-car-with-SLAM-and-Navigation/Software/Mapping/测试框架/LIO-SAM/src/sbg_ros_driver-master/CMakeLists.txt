cmake_minimum_required(VERSION 2.8.3)
project(sbg_driver)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  std_srvs
  geometry_msgs
  message_generation
)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(FILES
  SbgGpsVelStatus.msg
  SbgGpsPosStatus.msg
  SbgMagStatus.msg
  SbgShipMotionStatus.msg
  SbgUtcTimeStatus.msg

  SbgAirData.msg
  SbgAirDataStatus.msg
  SbgEkfEuler.msg
  SbgGpsPos.msg
  SbgMag.msg
  SbgStatusCom.msg
  SbgEkfNav.msg
  SbgGpsRaw.msg
  SbgOdoVel.msg
  SbgStatusGeneral.msg
  SbgEkfQuat.msg
  SbgGpsVel.msg
  SbgStatus.msg
  SbgEvent.msg
  SbgImuStatus.msg
  SbgImuData.msg
  SbgImuShort.msg
  SbgShipMotion.msg
  SbgUtcTime.msg
  SbgGpsHdt.msg
  SbgMagCalib.msg
  SbgStatusAiding.msg
  SbgEkfStatus.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES
  std_msgs
  geometry_msgs
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS src
  #LIBRARIES ${PROJECT_NAME} 
  CATKIN_DEPENDS 
    message_runtime 
    roscpp
    sensor_msgs
    std_msgs
    std_srvs
    geometry_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
add_subdirectory(external/sbgECom)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include/sbg_driver/
  ${PROJECT_SOURCE_DIR}/external/sbgECom/common
  ${PROJECT_SOURCE_DIR}/external/sbgECom/src
)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(sbg ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Define common resources.
set (SBG_COMMON_RESOURCES
  src/config_applier.cpp
  src/message_publisher.cpp
  src/message_wrapper.cpp
  src/config_store.cpp
  src/sbg_device.cpp
)

## Declare a C++ executable
add_executable(sbg_device ${SBG_COMMON_RESOURCES} src/main.cpp)
add_dependencies(sbg_device ${sbg_driver_EXPORTED_TARGETS})
target_compile_options(sbg_device PRIVATE -Wall -Wextra)

add_executable(sbg_device_mag ${SBG_COMMON_RESOURCES} src/main_mag.cpp)
add_dependencies(sbg_device_mag ${sbg_driver_EXPORTED_TARGETS})
target_compile_options(sbg_device_mag PRIVATE -Wall -Wextra)

## Specify libraries to link a library or executable target against
target_link_libraries(sbg_device ${catkin_LIBRARIES} sbgECom)
target_link_libraries(sbg_device_mag ${catkin_LIBRARIES} sbgECom)

set_property(TARGET sbg_device PROPERTY CXX_STANDARD 11)
set_property(TARGET sbg_device_mag PROPERTY CXX_STANDARD 11)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executables and/or libraries for installation
install(TARGETS sbg_device sbg_device_mag
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
  PATTERN ".git" EXCLUDE
)

# Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".git" EXCLUDE)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
  PATTERN ".git" EXCLUDE)

#############
## Testing ##
#############