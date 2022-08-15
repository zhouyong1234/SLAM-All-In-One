cmake_minimum_required( VERSION 2.8 )
project( registration )

set( CMAKE_BUILD_TYPE "Release" )
set( CMAKE_CXX_FLAGS "-std=c++11 -O3" )



find_package( OpenCV 3.1 REQUIRED )
# find_package( OpenCV REQUIRED ) # use this if in OpenCV2 

include_directories( 
    ${OpenCV_INCLUDE_DIRS} 
)

add_executable( registration registration.cpp  )
target_link_libraries( registration ${OpenCV_LIBS} )