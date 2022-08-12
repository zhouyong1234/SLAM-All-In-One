cmake_minimum_required(VERSION 3.5)
project(rs_driver_demos)
message(=============================================================)
message("-- Ready to compile demos")
message(=============================================================)
include_directories(${DRIVER_INCLUDE_DIRS})
add_executable(demo_online
              demo_online.cpp
              )
target_link_libraries(demo_online
                    ${EXTERNAL_LIBS}       
)
add_executable(demo_pcap
              demo_pcap.cpp
              )
target_link_libraries(demo_pcap
                ${EXTERNAL_LIBS}       
)

        