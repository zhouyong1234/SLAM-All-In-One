#include <iostream>
#include "laserSimulation.h"

#include "scanContext.h"
#include "scanContainer.h"

#include <opencv2/opencv.hpp>

#include "odomSimulation.h"

#include "readScanContext.h"

#include <unistd.h>
#include "loadMap.h"
#include "localization.h"
#include "slamProcessor.h"


void laserData2Container( const slam::sensor::LaserScan &scan, slam::ScanContainer &container )
{
        size_t size = 1440;

        float angle = -3.14159f;
        container.clear();

        for( int i = 0; i < size; i ++ ){ 
                float dist = scan.ranges[ i ];

                if( dist >= 0.00999999f && dist <= 15.0000000000f ){
                        //dist *= scaleToMap;
                        container.addData( Eigen::Vector2f( cos(angle) * dist, sin(angle) * dist ) );
                }

                angle += 0.0043633231f;
        }

        std::cout<<"Scan Container Size: "<<container.getSize()<<std::endl;
}


void displayMap( slam::OccupiedMap &occupiedGridMap, cv::Mat &image )
{

        int occupiedCount = 0;

        // display the map
        for( int i = 0; i < occupiedGridMap.getSizeX(); i ++ ){
                for( int j = 0; j < occupiedGridMap.getSizeY(); j ++ ){
                        if( occupiedGridMap.isCellFree( i, j ) ){
                                cv::circle(image, cv::Point2d(i, j), 1, cv::Scalar(255, 255, 255), -1);

                        }
                        else if( occupiedGridMap.isCellOccupied( i, j ) ){
                                occupiedCount ++;
                                cv::circle(image, cv::Point2d(i, j), 1, cv::Scalar(0, 0, 255), -1);
                        }

                }
        }
 
        std::cout<<"---------------- Result --------------------"<<std::endl;
        std::cout<<"Occupied Points Number: "<<occupiedCount<<std::endl;
}

/*void dispalyScans( slam::OccupiedMap &occupiedGridMap, slam::ScanContainer &container, cv::Mat &image )
{
	for( size_t i = 0; i < container.size(); i ++ ){
		Eigen::Vector2f pointInWorld;
		occupiedGridMap.observedPointPoseWorld2Map(  );
	}
}*/


int main()
{
	std::cout<<"--------------------- Load Map Test --------------------"<<std::endl;

	// init a image
    	cv::Mat image = cv::Mat::zeros(1200, 1200, CV_8UC3);

    	// ScanContext instance
    	slam::ScanContext<float, 20> scanContext;
    	slam::simulation::Simulation simulation;

    	// open the simulation file
    	std::string file_name = "../../../simulation_file/laser_data2.txt";
    	simulation.openSimulationFile(file_name);

    	// LaserScan instance & ScanContainer instance
    	slam::sensor::LaserScan scan;
    	slam::ScanContainer scanContainer;


	// 初始定位
    	localization::ReadScanContext readScanContext("../../../simulation_file/scanContext_data.txt");
    	scanContext.setScanContexts(readScanContext.scanContexts);
    	scanContext.setRingKeys(readScanContext.ringKeys);
    	scanContext.setSectorKeys(readScanContext.sectorKeys);
    	std::vector<Eigen::Vector3f> keyPoses(readScanContext.keyPoses);
    	bool init_location = false,glob_location = false;

	Eigen::Vector3f robotPose;

	// 加载地图
    	slam::OccupiedMap occupiedMap;

    	occupiedMap = slam::LoadMap()("../../../simulation_file/test.map");
    	occupiedMap.setMapInfo(1001, 1001, 10);

    	std::cout << "-------------------- Map Information -----------------" << std::endl;
    	std::cout << "map sizeX : " << occupiedMap.getSizeX() << std::endl;
    	std::cout << "map sizeY : " << occupiedMap.getSizeY() << std::endl;
    	std::cout << "map cell length : " << occupiedMap.getCellLength() << std::endl;
    	std::cout << "------------------------- END -----------------------" << std::endl;

    	cv::Mat image1 = cv::Mat::zeros(occupiedMap.getSizeX(), occupiedMap.getSizeY(), CV_8UC3);
    	displayMap(occupiedMap, image1);
    	cv::imshow("map", image1);

	// 定位
    	slam::SlamProcessor slam;

    	while (!simulation.endOfFile()){

        	// read a frame of data
        	simulation.readAFrameData(scan);
        	std::cout << "frame count: " << simulation.getFrameCount() << std::endl;

        	laserData2Container(scan, scanContainer); // convert the laser data to scanContainer type
        	// scanContainer.displayAFrameScan( simulation.getFrameCount(), 20.0f );
        	// 初始定位
	
		if (!init_location){
            		std::cerr << "------------------ UPDATE ----------------" << std::endl;
            		Eigen::MatrixXf sc = scanContext.makeScanContext(scan);
	            	scanContext.displayAScancontext(sc);
        	    	scanContext.makeAndSaveScancontextAndKeys(scan);
            		slam::yaw_dist_id yaw_dist_id;
	            	scanContext.detectScancontextID(yaw_dist_id);
            	
			if (yaw_dist_id.pose_id != -1 && !glob_location){
               	 		init_location = true;
                		glob_location = true;
	                	float dx = yaw_dist_id.dist * cos(yaw_dist_id.yaw);
        	        	float dy = yaw_dist_id.dist * sin(yaw_dist_id.yaw);
                		float dyaw = yaw_dist_id.yaw;
                		
				robotPose = keyPoses[yaw_dist_id.pose_id] + Eigen::Vector3f(dx, dy, dyaw);
	                	std::cout << "pose: " << robotPose.transpose() << "pose_id: " << yaw_dist_id.pose_id << std::endl;
        	        	// 保存到文件pose.txt
                		// std::ofstream outfile("../simulation_file/pose.txt", std::ios::app);
                		// outfile << robotPose.transpose() << std::endl;
            		}	
        	}
		// 全局定位 scan_to_map
        	else{
            		// 时间戳
	           	slam.update(robotPose, scanContainer);
        	    	robotPose = slam.getLastScanMatchPose();
            		std::cout << "robot pose now: " << std::endl;
	            	std::cout << robotPose << std::endl;
        	    	std::cout << "------------------" << std::endl;
            		std::ofstream outfile("../pose.txt", std::ios::app);
	            	outfile << robotPose.transpose() << std::endl;
        	}
        	cv::waitKey(60);
    	}
	
	simulation.closeSimulationFile();
	return 0;
}
