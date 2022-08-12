#include "slamProcessor.h"
#include "laserSimulation.h"

#include "loopClosureScanContext.h"

#include "graphOptimize.h"

#include <unistd.h>

void laserData2Container( const slam::sensor::LaserScan &scan, slam::ScanContainer &container )
{
        size_t size = 1440;

        float angle = -3.14159f;
        container.clear();

        for( int i = 0; i < size; i ++ ){
                float dist = scan.ranges[ i ];

                if( dist >= 0.0099999998f && dist <= 15.0000000000f ){
                        //dist *= scaleToMap;
                        container.addData( Eigen::Vector2f( cos(angle) * dist, sin(angle) * dist ) );
                }

                angle += 0.0043633231f;
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
	
	// loop closure instance
	slam::LoopClosureBase *loopDetect = new slam::ScanContextLoopClosure();
	
	// g2o instance
	slam::optimizer::GraphOptimize optimizer;

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

	// convarince
	Eigen::Matrix3f covarince;

	// robot pose
	Eigen::Vector3f robotPosePrev( 0.0f, 0.0f, 0.0f );
//	Eigen::Vector3f robotPoseCurr( 0.0f, 0.0f, 0.0f );

	// slam process
	// while it is not the end of the simulation file
	int keyFrameCount = 0;
	
	std::vector<slam::sensor::LaserScan> keyScans;
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
			if( simulation.getFrameCount() == 10 ){
				// initial key pose and key scan
				keyScans.push_back( scan );
                        	keyPoses.push_back( robotPosePrev );
				
				optimizer.addVertex( robotPosePrev, keyFrameCount );
			}
		}
		else{
	
			// 1. Update by Scan Match, get the estimated pose 
			slam.update( robotPosePrev, scanContainer );
		
			// 2. get the newest robot pose in world coordinate	
			robotPosePrev = slam.getLastScanMatchPose();
			std::cout<<"robot pose now: "<<std::endl;
			std::cout<<robotPosePrev<<std::endl;
			std::cout<<"------------------"<<std::endl;
			
			// 3. if this is a key scan frame
			if( slam.isKeyFrame() ){
				keyFrameCount ++;			
	
				keyScans.push_back( scan );
				keyPoses.push_back( robotPosePrev );
				
				optimizer.addVertex( robotPosePrev, keyFrameCount ); // add a vertex
				Eigen::Matrix3d information = Eigen::Matrix3d::Identity(); //information matrix
		
				optimizer.addEdge( slam.getPoseDifferenceValue(), keyFrameCount - 1, keyFrameCount, information ); // add a edge
		
				loopDetect->detectLoop( scan ); // loop detect
				
				int loopId = loopDetect->detectedALoop();
				if( loopId != -1 ){
					loopDetect->caculateTransformByICP();
	
					// TODO ...
					Eigen::Vector3f loopPoseDiff;

					optimizer.addEdge( loopPoseDiff, keyFrameCount, loopId, information );
				
					optimizer.execuateGraphOptimization();
					
					optimizer.getOptimizedResults();
		
					std::vector<Eigen::Vector3f> estimatedPoses = optimizer.getEstimatedPoses();
					
					keyPoses.erase( keyPoses.begin() + loopId, keyPoses.end() );
	
					keyPoses.insert( keyPoses.end(), estimatedPoses.begin(), estimatedPoses.end() );
				
				}
			}
			

		}
		// 3. display the map
		slam.displayMap( image );
		
		cv::waitKey(5);	
	}
		

	// close the simulation file
	simulation.closeSimulationFile();


	return 0;
}
