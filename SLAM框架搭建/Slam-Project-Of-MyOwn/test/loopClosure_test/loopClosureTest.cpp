#include "loopClosureScanContext.h"
#include <opencv2/opencv.hpp>

#include <iostream>
#include "laserSimulation.h"
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

void dispalyOdom( cv::Mat &image, Eigen::Vector3f &pose )
{
        cv::Point2d point( pose(0) * 10 + 400, pose(1) * 10 + 700 );
        cv::circle(image, point, 3, cv::Scalar(0, 0, 255), 1);

        cv::imshow( "odom", image );
}


int main()
{
	std::cout<<" ------------------- Loop Closure Test ------------------- "<<std::endl;

	cv::Mat image = cv::Mat::zeros( 1200, 1200, CV_8UC3 );
	
	// 
	slam::LoopClosureBase *detect = new slam::ScanContextLoopClosure();
	
	slam::simulation::Simulation simulation;
        slam::simulation::OdomSimulation odomSim;

	// open the simulation file
        std::string file_name = "laser_data2.txt";
        simulation.openSimulationFile( file_name );

        std::string odom_file_name = "odom_data.txt";
        odomSim.openSimulationFile( odom_file_name );
	
	// LaserScan instance & ScanContainer instance
        slam::sensor::LaserScan scan;
	scan.angle_min = -3.14159f;
	scan.angle_max = 3.14159f;
	scan.angle_increment = 0.0069800001f;
	scan.range_min = 0.2000000030f;
	scan.range_max = 50.0000000000f;
	

        Eigen::Vector3f poseOld = Eigen::Vector3f::Zero();
        Eigen::Vector3f poseNew = poseOld;

	while( !simulation.endOfFile() ){

                // read a frame of data
                simulation.readAFrameData( scan );
                std::cout<<"frame count: "<<simulation.getFrameCount()<<std::endl;
		odomSim.readAFrameData( poseNew );
                std::cout<<"pose: "<<std::endl<<poseNew<<std::endl;

		if( poseDiffLargerThan( poseOld, poseNew ) ){
                        std::cerr<<"------------------ UPDATE ----------------"<<std::endl;
			dispalyOdom( image, poseNew );
	
		 	detect->detectLoop( scan );
			//detect->caculateTransformByICP();
			//Eigen::Matrix<float, 2, 2> R = detect->getRotateMatrix();
			//Eigen::Vector2f T = detect->getTransformVector();

			//std::cout<<"Rotation: "<<std::endl<<R<<std::endl;
			//std::cout<<"Transform: "<<std::endl<<T<<std::endl;	
		
			poseOld= poseNew;
		}
		
		cv::waitKey( 60 );
	}
	
	simulation.closeSimulationFile();
	odomSim.closeSimulationFile();

	return 0;
}
