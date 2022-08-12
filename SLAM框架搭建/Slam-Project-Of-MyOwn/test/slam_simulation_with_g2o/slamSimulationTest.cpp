#include "slamProcessor.h"
#include "laserSimulation.h"

#include "loopClosureScanContext.h"

#include "graphOptimize.h"

#include <unistd.h>

#include <fstream>

#include "saveMap.h"


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
	optimizer.createOptimizer();
        optimizer.setMaxIeration(10);
		

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
	std::string file_name = "../../../../simulation_file/laser_data2.txt";
	simulation.openSimulationFile( file_name );

	// convarince
	Eigen::Matrix3f covarince;

	// robot pose
	Eigen::Vector3f robotPosePrev( 0.0f, 0.0f, 0.0f );
	Eigen::Vector3f robotPoseCurr( 0.0f, 0.0f, 0.0f );

	// slam process
	// while it is not the end of the simulation file
	int keyFrameCount = 0;
	

	std::vector<slam::ScanContainer> keyScanContainers;
	std::vector<Eigen::Vector3f> keyPoses;

	while( !simulation.endOfFile() ){
		// 1. get the laser data
		slam::sensor::LaserScan scan;
	
		slam::ScanContainer scanContainer;
		simulation.readAFrameData( scan ); // read the laser data
		
		laserData2Container( scan, scanContainer );// convert the laser data to scanContainer type	
	
		std::cout<<"frame count: "<<simulation.getFrameCount()<<std::endl;	
		if( simulation.getFrameCount() <= 10  ){
			slam.processTheFirstScan( robotPoseCurr, scanContainer );

			if( simulation.getFrameCount() == 10 ){
				// initial key pose and key scan
                        	
				keyPoses.push_back( robotPoseCurr ); // initial key pose
				keyScanContainers.push_back( scanContainer ); // initial key scan container		
		
				optimizer.addVertex( robotPoseCurr, keyFrameCount ); // the first vertex of the pose graph
		
				robotPosePrev = robotPoseCurr; // update the pose of robot

				slam.displayMap( image ); // display the map
			}
		}
		else {
	
			// 1. Update by Scan Match, get the estimated pose 
			slam.update( robotPoseCurr, scanContainer );
		
			// 2. get the newest robot pose in world coordinate	
			robotPoseCurr = slam.getLastScanMatchPose();
			std::cout<<"robot pose now: "<<std::endl;
			std::cout<<robotPoseCurr<<std::endl;
			std::cout<<"------------------"<<std::endl;
			
			// 3. if this is a key scan frame
			if( slam.isKeyFrame() ){
				std::cerr<<"--------------------- Key Frame : "<<keyFrameCount <<"-----------------------"<<std::endl;

				keyFrameCount ++;			
				std::cout<<"key frame count : "<<keyFrameCount<<std::endl;			

				keyPoses.push_back( robotPoseCurr ); // key pose 
				keyScanContainers.push_back( scanContainer ); // key scan container		
			
				// ------------------------------ For G2O ------------------------------------//
				optimizer.addVertex( robotPoseCurr, keyFrameCount ); // add a vertex
				Eigen::Matrix3d information = Eigen::Matrix3d::Identity(); //information matrix
	
				Eigen::Matrix<float, 3, 3> T1 = slam.v2t( robotPosePrev );
                                Eigen::Matrix<float, 3, 3> T2 = slam.v2t( robotPoseCurr );

                                Eigen::Matrix<float, 3, 3> T = T1.inverse() * T2;
                                Eigen::Vector3f V = slam.t2v( T );
                                std::cout<<"V = "<<std::endl<<V<<std::endl<<std::endl;
	
				std::cout<<"edge: "<<keyFrameCount - 1<<" to "<<keyFrameCount<<std::endl;
                                optimizer.addEdge( V, keyFrameCount - 1, keyFrameCount, information ); // add a edge


				// -------------------------------- END ------------------------------------//
		
				// ---------------------------- For Loop Closure ---------------------------//
				loopDetect->detectLoop( scan ); // loop detect
				
				int loopId = loopDetect->detectedALoop();
			
				// if find a loop closure
				if( loopId != -1 ){
					cv::line( image, cv::Point2f( robotPoseCurr(0), robotPoseCurr(1) ),
                                                 cv::Point2f( keyPoses[loopId](0), keyPoses[loopId](1) ),
                                                 cv::Scalar( 0, 255, 0 ), 1 );

					// filtering the wrong loop closure according to the distance
					if( ( robotPosePrev - keyPoses[loopId] ).head<2>().norm() > 2.0f ){
						std::cout<<"loop closure is too large ..."<<std::endl;
						continue;
					}

					std::cout<<"-------------------------- Find A Loop Closure --------------------------"<<std::endl;
					std::cout<<"Pose Current: "<<std::endl<<robotPoseCurr<<std::endl;
					std::cout<<"Pose Loop: "<<std::endl<<keyPoses[loopId]<<std::endl;
					std::cout<<"-------------------------------------------------------------------------"<<std::endl;
				
					Eigen::Matrix3d information = 1 * Eigen::Matrix3d::Identity(); //information matrix
	                                Eigen::Matrix<float, 3, 3> T1 = slam.v2t( robotPoseCurr );
        	                        Eigen::Matrix<float, 3, 3> T2 = slam.v2t( keyPoses[loopId] );

                	                Eigen::Matrix<float, 3, 3> T = T1.inverse() * T2;
                        	        Eigen::Vector3f V = slam.t2v( T );

                                	optimizer.addEdge( V, keyFrameCount, loopId, information ); // add a loop constraint
	                                std::cout<<"Add A Loop Correlation ..."<<std::endl;
					std::cout<<"----------- execuate the graph optimization ----------"<<std::endl;

	                                optimizer.execuateGraphOptimization(); // execuate the graph optimization

        	                        optimizer.getOptimizedResults(); // get the optimized results

                	                std::vector<Eigen::Vector3f> estimatedPoses = optimizer.getEstimatedPoses();
                        	        std::cout<<"keyPoses.size  = "<<keyPoses.size()<<std::endl;
                                	std::cout<<"estimatedPoses.size = "<<estimatedPoses.size()<<std::endl;

                                	// TODO ... process the estimated results
					// ---------------- ReConstruct the Occupied grid Map ---------------//
					slam.reconstructMap( keyPoses, keyScanContainers );
					slam.displayMap( image, keyPoses );
					cv::waitKey(0);
					// ------------------------------- END ------------------------------//
					
                                	break;
				}
				// ---------------------- END ----------------------//
			
				robotPosePrev = robotPoseCurr;
			
				slam.displayMap( image );
			}
			

		}
		// 3. display the map
		//slam.displayMap( image );
		
		cv::waitKey(5);	
	}
		
//        std::string map_name = "test1.map";
//        slam::SaveMap()(map_name, slam.getOccupiedMap());
	slam.saveMap( "test1.map" );

	//loopDetect->saveScanContext( "scanContext_data.txt", keyPoses );
        cv::waitKey(0);


	// close the simulation file
	simulation.closeSimulationFile();


	return 0;
}
