#include "slamProcessor.h"
#include "laserSimulation.h"
#include <unistd.h>

#include "odomSimulation.h"

#include "saveMap.h"

void laserData2Container( const slam::sensor::LaserScan &scan, slam::ScanContainer &container )
{
        size_t size = 1440;

        float angle = -3.12413907051f;
        container.clear();

        for( int i = 0; i < size; i ++ ){
                float dist = scan.ranges[ i ];

                if( dist >= 0.25f && dist <= 15.0000000000f ){
                        //dist *= scaleToMap;
                        container.addData( Eigen::Vector2f( cos(angle) * dist, sin(angle) * dist ) );
                }

                angle += 0.00435422640294f;
        }

        std::cout<<"Scan Container Size: "<<container.getSize()<<std::endl;
}


int main()
{
	std::cout<<"--------------------- SLAM Simulation --------------------"<<std::endl;
	
	// slam classes instances
	slam::SlamProcessor slam;
	slam::simulation::Simulation simulation;
	slam::MapInfo mapInfo = slam.getMapInfo();
	
	slam::simulation::OdomSimulation ouputOdom;	


	// print the map information
	std::cout<<"------------- Map Information ----------------"<<std::endl;
	std::cout<<"Map SizeX: "<<mapInfo.getSizeX()<<std::endl;
	std::cout<<"Map SizeY: "<<mapInfo.getSizeY()<<std::endl;
	std::cout<<"Map Center: ( "<<mapInfo.getMapCenter()[0]<<", "<<mapInfo.getMapCenter()[1]<<" )"<<std::endl;
	std::cout<<"Map Scale: "<<mapInfo.getScale()<<std::endl;
	std::cout<<"Map Cell Length: "<<mapInfo.getCellLength()<<std::endl;
	std::cout<<"----------------------------------------------"<<std::endl;		
	
	// init the image
	cv::Mat image = cv::Mat::zeros(mapInfo.getSizeX(), mapInfo.getSizeY(), CV_8UC3);
        cv::Point2d center( mapInfo.getMapCenter()[0], mapInfo.getMapCenter()[1] );
        std::cout<<"center: ( "<<mapInfo.getMapCenter()[0]<<", "<<mapInfo.getMapCenter()[1]<<" )"<<std::endl;
        cv::circle(image, center, 3, cv::Scalar(0, 0, 255), 3);
	cv::imshow("map", image);
	
	// open the simulation file
	std::string file_name = "../../../simulation_file/laser_data.txt";
	simulation.openSimulationFile( file_name );


	// added for recording the laser odometry data
	ouputOdom.openOutputSimulationFile( "../../../simulation_file/odometry2.txt" );
		
	// convarince
	Eigen::Matrix3f covarince;

	// robot pose
	Eigen::Vector3f robotPosePrev( 0.0f, 0.0f, 0.0f );
//	Eigen::Vector3f robotPoseCurr( 0.0f, 0.0f, 0.0f );

	// slam process
	// while it is not the end of the simulation file
	int count = 0;

	// key poses
	std::vector<Eigen::Vector3f> keyPoses;

	while( !simulation.endOfFile() ){
		// 1. get the laser data
		slam::sensor::LaserScan scan;
	
		slam::ScanContainer scanContainer;
		simulation.readAFrameData( scan ); // read the laser data
		
		laserData2Container( scan, scanContainer );// convert the laser data to scanContainer type	
	
		std::cout<<"frame count: "<<simulation.getFrameCount()<<std::endl;	
		if( simulation.getFrameCount() <= 10  ){
			slam.processTheFirstScan( robotPosePrev, scanContainer );
			slam.displayMap( image );
			
			if( simulation.getFrameCount() == 10 ){
				keyPoses.push_back( robotPosePrev );
			}

			// added 
			ouputOdom.writeAFrameData( robotPosePrev );
		}
		else{
	
			// 1. Update by Scan Match, get the estimated pose 
			slam.update( robotPosePrev, scanContainer );
	
			// 2. get the newest robot pose in world coordinate	
			robotPosePrev = slam.getLastScanMatchPose();
			std::cout<<"robot pose now: "<<std::endl;
			std::cout<<robotPosePrev<<std::endl;
			std::cout<<"------------------"<<std::endl;
	
			// record
			//ouputOdom.writeAFrameData( robotPosePrev );
		
			if( slam.isKeyFrame() ){
				// added for recording the laser odometry data
				//ouputOdom.writeAFrameData( robotPosePrev );
	
				keyPoses.push_back( robotPosePrev );
				slam.displayMap( image, keyPoses );
			}
		}
		// 3. display the map
		//slam.displayMap( image );
		
		cv::waitKey(5);	
		count ++;
	}
	
	// finally, save the occupied grid map
	slam::SaveMap()( "./test.map", slam.getOccupiedMap() );
	

	// close the simulation file
	simulation.closeSimulationFile();


	return 0;
}
