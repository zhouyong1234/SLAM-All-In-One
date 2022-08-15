add_library(OpticalSLAM 
			OpticalSLAM.cpp
			StateEstimator.cpp
			ImageManager.cpp
			params.cpp)

target_link_libraries(
        OpticalSLAM
        ${OpenCV_LIBS}
)