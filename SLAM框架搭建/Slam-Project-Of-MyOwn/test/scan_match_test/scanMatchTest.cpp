#include "scanMatch.h"
#include "laserSimulation.h"

#include "scanContainer.h"

#include "occupiedGridMap.h"
#include "gridBase.h"
#include "utils.h"
#include <opencv2/opencv.hpp>

#include "dataType.h"

void laserData2Container( const slam::simulation::Laser &scan, slam::ScanContainer &container )
{
	container.clear();

	double theta = -std::fabs(-3.12144 - 3.14159) / 360;
		
	for( int i = 0; i < 360; i ++ ){
		float dist = scan.range[i];

		if( dist > 0.1f && dist < 10.0f ){
			Eigen::Vector2f point(::cos(theta) * dist, ::sin(theta) * dist);
			std::cout<<"laser point: ( "<<point[0]<<", "<<point[1]<<" )"<<std::endl;
			container.addData(point);
		}
	
		theta += std::fabs( 0.0174533f );
	}
	
	std::cout<<"Scan Container Size: "<<container.getSize()<<std::endl;
}

void displayAFrame( cv::Mat &image, slam::OccupiedGridMap<slam::GridBase> &occumap )
{
	int occupiedCount = 0;

	for( int i = 0; i < occumap.getSizeX(); i ++ ){
		for( int j = 0; j < occumap.getSizeY(); j ++ ){
			if( occumap.isCellFree( i, j ) ){
                                cv::circle(image, cv::Point2d(i, j), 1, cv::Scalar(255, 255, 255), 1);
                                //std::cout<<"Free Point: ( "<<i<<", "<<j<<" )"<<std::endl;
                                //std::cout<<"prob: "<<occumap.getCellOccupiedProbability( i, j )<<std::endl;
                        }
                        else if( occumap.isCellOccupied( i, j ) ){
                                occupiedCount ++;
                                //std::cout<<"Occupied Point: ( "<<i<<", "<<j<<" )"<<std::endl;
                                //std::cout<<"prob: "<<occumap.getCellOccupiedProbability( i, j )<<std::endl;
                                //std::cout<<"log Odds value: "<<occumap.getCellLogOdds(i, j)<<std::endl;
                                cv::circle(image, cv::Point2d(i, j), 1, cv::Scalar(0, 0, 255), 1);
                        }

		}
	}

	std::cout<<"---------------- Result --------------------"<<std::endl;
        std::cout<<"Occupied Points Number: "<<occupiedCount<<std::endl;

        cv::imshow( "test", image );
}

int main()
{
	std::cout<<"---------------------- Scan Match Test --------------------"<<std::endl;

	//---------------- Init the objects --------------//
	slam::OccupiedGridMap<slam::GridBase> occumap;
	slam::ScanMatchMethod scanMatch;
	slam::simulation::Laser scanArray[2];

	//----------------- Init the Map Picture --------------//
	cv::Mat image = cv::Mat::zeros(occumap.getSizeX(), occumap.getSizeY(), CV_8UC3);
        cv::Point2d center( occumap.getMapCenter()[0], occumap.getMapCenter()[1] );
        std::cout<<"center: ( "<<occumap.getMapCenter()[0]<<", "<<occumap.getMapCenter()[1]<<" )"<<std::endl;
        cv::circle(image, center, 3, cv::Scalar(0, 0, 255), 3);
	//cv::imshow( "test", image );
	
	
	//---------------- Input the first scan to initialize the map ----------------//
	slam::ScanContainer laserPoints;
	slam::simulation::readLaserTXT( "laser_data.txt", scanArray );
	
	laserData2Container( scanArray[0], laserPoints );
	
	Eigen::Vector3f robotPose( 0.0f, 0.0f, 0.0f );
        occumap.updateByScan( laserPoints, robotPose );
	
	//------------- Display the first Map ------------//
	displayAFrame( image, occumap );

	//------------- Input the second scan to scan to map -------------//
	Eigen::Matrix3f covarince;
	laserData2Container( scanArray[1], laserPoints );
	std::cout<<"Doing the Scan TO Map Operation ......"<<std::endl;
	Eigen::Vector3f newRobotPose = scanMatch.scanToMap( occumap, robotPose, laserPoints, covarince, 100 );// scan to map to caclulat the new robot pose in world coordinate
	std::cout<<"------------------ New Robot Pose ---------------"<<std::endl;
	std::cout<<newRobotPose<<std::endl;
	
	//------------ updated the map ------------//
	occumap.updateByScan( laserPoints, newRobotPose );	
	//displayAFrame( image, occumap );
	

	cv::waitKey(0);		

	return 0;
}
