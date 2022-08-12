#include <iostream>
#include "laserSimulation.h"

#include "scanContext.h"
#include "scanContainer.h"

#include <opencv2/opencv.hpp>

#include "odomSimulation.h"

bool poseDiffLargerThan( Eigen::Vector3f &poseOld, Eigen::Vector3f &poseNew )
{

        if( ( ( poseNew.head<2>() - poseOld.head<2>() ).norm() ) > 0.4f ){
                return true;
        }

        float angleDiff = ( poseNew.z() - poseOld.z() );

        if( angleDiff > M_PI ){
                angleDiff -= M_PI * 2.0f;
        }
        else if( angleDiff < -M_PI ){
                angleDiff += M_PI * 2.0f;
        }

        if( ::abs( angleDiff ) > 0.9f ){
                return true;
        }

        return false;
}


void laserData2Container( const slam::sensor::LaserScan &scan, slam::ScanContainer &container )
{
        size_t size = 901;

        float angle = -3.14159f;
        container.clear();

        for( int i = 0; i < size; i ++ ){ 
                float dist = scan.ranges[ i ];

                if( dist >= 0.2000000030f && dist <= 50.0000000000f ){
                        //dist *= scaleToMap;
                        container.addData( Eigen::Vector2f( cos(angle) * dist, sin(angle) * dist ) );
                }

                angle += 0.0069800001f;
        }

        std::cout<<"Scan Container Size: "<<container.getSize()<<std::endl;
}

void dispalyOdom( cv::Mat &image, Eigen::Vector3f &pose )
{
	cv::Point2d point( pose(0) * 10 + 400, pose(1) * 10 + 700 );
        cv::circle(image, point, 3, cv::Scalar(0, 0, 255), 1);

	cv::imshow( "odom", image );
}


int main()
{
	std::cout<<" ------------- Scan Context Test ---------------"<<std::endl;
	// init a image
	cv::Mat image = cv::Mat::zeros( 1200, 1200, CV_8UC3 );
	
	// ScanContext instance
	slam::ScanContext<float, 20> scanContext;
	slam::simulation::Simulation simulation;
	slam::simulation::OdomSimulation odomSim;	

	// open the simulation file
        std::string file_name = "laser_data2.txt";
        simulation.openSimulationFile( file_name );
	
	std::string odom_file_name = "odom_data.txt";
	odomSim.openSimulationFile( odom_file_name );

	// LaserScan instance & ScanContainer instance
	slam::sensor::LaserScan scan;
	slam::ScanContainer scanContainer;

	Eigen::Vector3f poseOld = Eigen::Vector3f::Zero();
	Eigen::Vector3f poseNew = poseOld;

	while( !simulation.endOfFile() ){	

		// read a frame of data
		simulation.readAFrameData( scan );	
		std::cout<<"frame count: "<<simulation.getFrameCount()<<std::endl;

		laserData2Container( scan, scanContainer );// convert the laser data to scanContainer type
		scanContainer.displayAFrameScan( simulation.getFrameCount(), 20.0f );

		odomSim.readAFrameData( poseNew );
		std::cout<<"pose: "<<std::endl<<poseNew<<std::endl;	
		
		if( poseDiffLargerThan( poseOld, poseNew ) ){	
			std::cerr<<"------------------ UPDATE ----------------"<<std::endl;
				
			dispalyOdom( image, poseNew );

	
			Eigen::MatrixXf sc = scanContext.makeScanContext( scan );
			scanContext.displayAScancontext( sc );
		//	scanContext.makeRingkeyFromScancontext( sc );
		
			scanContext.makeAndSaveScancontextAndKeys( scan );	
			scanContext.detectLoopClosureID();	
		
			poseOld = poseNew;
		}
		
		cv::waitKey( 60 );	
	
	}
		
	simulation.closeSimulationFile();

	return 0;
}
