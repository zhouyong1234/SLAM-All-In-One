#include "slamProcessor.h"
#include <fstream>

namespace slam {

SlamProcessor::SlamProcessor(): minDistanceDiffForMapUpdate( 0.4 ),
				minAngleDiffForMapUpdate( 0.9 ) // default parameter
{
	occupiedGridMap = new OccupiedMap();
	if( occupiedGridMap == nullptr ){
		std::cerr<<"Construct Occupied Grid Map Failed ..."<<std::endl;
		exit(-1);
	}
	std::cerr<<"Construct Occupied Grid Map Successfully ..."<<std::endl;

	scanMatch = new ScanMatchMethod();
	if( scanMatch == nullptr ){
		std::cout<<"Construct Scan Match Method Failed ..."<<std::endl;
		exit(-1);
	}	
	std::cerr<<"Construct Scan Match Method Successfully ..."<<std::endl;

	// ------------- initialize some parameters --------------//
	covarianceMatrix = Eigen::Matrix3f::Zero();
	lastScanMatchPose = Eigen::Vector3f::Zero();
	lastMapUpdatePose = Eigen::Vector3f::Zero();

	// init the image size
	//image = cv::Mat::zeros(occupiedGridMap->getSizeX(), occupiedGridMap->getSizeY(), CV_8UC3);

}

SlamProcessor::~SlamProcessor()
{
	if( occupiedGridMap != nullptr ){
		delete occupiedGridMap;
	}
	
	if( scanMatch != nullptr ){
		delete scanMatch;
	}
}

SlamProcessor::SlamProcessor( int sizeX_, int sizeY_, float cellLength_ ): minDistanceDiffForMapUpdate( 0.4 ),
                                					   minAngleDiffForMapUpdate( 0.9 ) // default parameter
{
	occupiedGridMap = new OccupiedMap( sizeX_, sizeY_, cellLength_ );	
	if( occupiedGridMap == nullptr ){
                std::cerr<<"Construct Occupied Grid Map Failed ..."<<std::endl;
                exit(-1);
        }
        std::cerr<<"Construct Occupied Grid Map Successfully ..."<<std::endl;

	scanMatch = new ScanMatchMethod();
	if( scanMatch == nullptr ){
                std::cout<<"Construct Scan Match Method Failed ..."<<std::endl;
                exit(-1);
        }
        std::cerr<<"Construct Scan Match Method Successfully ..."<<std::endl;
	
	// ------------- initialize some parameters --------------//
        covarianceMatrix = Eigen::Matrix3f::Zero();
        lastScanMatchPose = Eigen::Vector3f::Zero();
        lastMapUpdatePose = Eigen::Vector3f::Zero();
	
	// init the image size
        //image = cv::Mat::zeros(occupiedGridMap->getSizeX(), occupiedGridMap->getSizeY(), CV_8UC3);

}

void SlamProcessor::setUpdateLogOddsPoccValue( float Pocc )
{
	return occupiedGridMap->setLogOddsPoccValue( Pocc );
}

void SlamProcessor::setUpdateLogOddsPfreeValue( float Pfree )
{
	return occupiedGridMap->setLogOddsPfreeValue( Pfree );
}

void SlamProcessor::setMinDistanceDiffForMapUpdate( float minDist )
{
	minDistanceDiffForMapUpdate = minDist;
}

void SlamProcessor::setMinAngleDiffForMapUpdate( float minAngle )
{
	minAngleDiffForMapUpdate = minAngle;
}

void SlamProcessor::update( Eigen::Vector3f &robotPoseInWorld, 
			    ScanContainer &scanContainer,
			    bool mapWithoutMatching )
{
	// a state variable for judging if this is a key scan frame
	keyFrame = false;	

	// 1. Pose Scan Match
	Eigen::Vector3f newPoseEstimated;// estimated pose in world coordinate
	
	if( !mapWithoutMatching ){
		newPoseEstimated = scanMatch->scanToMap( *occupiedGridMap, 
							 robotPoseInWorld, 
							 scanContainer, 
							 covarianceMatrix, 
							 100 );
	}
	else {
		newPoseEstimated = robotPoseInWorld;
	}
	
	lastScanMatchPose = newPoseEstimated;
	
	// 2. Map Update
	if( poseDiffLargerThan( lastMapUpdatePose, newPoseEstimated ) ){
		// if pose change is greater than the threshold, then this is a Key scan Frame
		keyFrame = true;


		// update the map only when the pose change is greater than the threshol
		occupiedGridMap->updateByScan( scanContainer, newPoseEstimated );
		// occupiedGridMap->onMapUpdate();
		lastMapUpdatePose = newPoseEstimated;
	}
	
}


bool SlamProcessor::poseDiffLargerThan( Eigen::Vector3f &poseOld, Eigen::Vector3f &poseNew )
{
	// get the difference value between old pose and new pose
	poseDiff = poseNew - poseOld;

	if( ( ( poseNew.head<2>() - poseOld.head<2>() ).norm() ) > minDistanceDiffForMapUpdate ){
		return true;
	}

	float angleDiff = ( poseNew.z() - poseOld.z() );

	if( angleDiff > M_PI ){
		angleDiff -= M_PI * 2.0f;
	}
	else if( angleDiff < -M_PI ){
		angleDiff += M_PI * 2.0f;
	}

	if( ::abs( angleDiff ) > minAngleDiffForMapUpdate ){
		return true;
	}
	
	return false;
}

MapInfo SlamProcessor::getMapInfo() const
{
	return occupiedGridMap->getMapInfo();
}

void SlamProcessor::processTheFirstScan( Eigen::Vector3f &robotPoseInWorld, 
                                  	 ScanContainer &scanContainer )
{
	// if it is the first laser scan, just update to initialize the map 
	occupiedGridMap->updateByScan( scanContainer, robotPoseInWorld );
}

void SlamProcessor::laserData2Container( const slam::sensor::LaserScan &scan, slam::ScanContainer &container )
{
	std::cout<<"------------------ Laser Data To Container -----------------"<<std::endl;
	container.clear();

	//float theta =  -3.12414f ;
	float theta = -std::fabs( scan.angle_min );
#ifdef TERMINAL_LOG
	std::cout<<"theta = "<<theta<<std::endl;
#endif	

	for( int i = 0; i < scan.size(); ++ i ){
		float dist = scan.ranges[ i ];

#ifdef TERMINAL_LOG
		std::cout<<"distance = "<<dist<<std::endl;		
#endif
		if( dist >= scan.range_min && dist <= scan.range_max ){
		//if( dist >= 0.1f && dist <= 12.0f ){
			Eigen::Vector2f point( ::cos( theta ) * dist, ::sin( theta ) * dist );

#ifdef TERMINAL_LOG
			std::cout<<"laser point: ( "<<point[0]<<", "<<point[1]<<" )"<<std::endl;
#endif	
			container.addData( point );
		}
		
		theta += std::fabs( scan.angle_increment );
		//theta += std::fabs( 0.0174533f );
	}
	
	 std::cout<<"Scan Container Size: "<<container.getSize()<<std::endl;
}



/*Eigen::Vector3f SlamProcessor::getLastScanMatchPose() const
{
	return lastScanMatchPose;
}*/

const Eigen::Vector3f SlamProcessor::getLastScanMatchPose() const
{
        return lastScanMatchPose;
}

/*Eigen::Vector3f SlamProcessor::getLastMapUpdatePose() const
{
	return lastMapUpdatePose;
}*/

const Eigen::Vector3f SlamProcessor::getLastMapUpdatePose() const
{
        return lastMapUpdatePose;
}

const Eigen::Matrix3f SlamProcessor::getCovarianceMatrix() const 
{
	return covarianceMatrix;
}

void SlamProcessor::displayMap( cv::Mat &image ) 
{
	
	int occupiedCount = 0;

	// display the map
	for( int i = 0; i < occupiedGridMap->getSizeX(); i ++ ){
		for( int j = 0; j < occupiedGridMap->getSizeY(); j ++ ){
			if( occupiedGridMap->isCellFree( i, j ) ){
                                cv::circle(image, cv::Point2d(i, j), 1, cv::Scalar(255, 255, 255), -1);
                        
#ifdef TERMINAL_LOG
			        std::cout<<"Free Point: ( "<<i<<", "<<j<<" )"<<std::endl;
					std::cout << "prob: " << occupiedGridMap->getCellOccupiedProbability(i, j) << std::endl;
#endif
                        }
                        else if( occupiedGridMap->isCellOccupied( i, j ) ){
                                occupiedCount ++;
#ifdef TERMINAL_LOG
	                        std::cout<<"Occupied Point: ( "<<i<<", "<<j<<" )"<<std::endl;
                                std::cout<<"prob: "<<occupiedGridMap->getCellOccupiedProbability( i, j )<<std::endl;
                                std::cout<<"log Odds value: "<<occupiedGridMap->getCellLogOdds(i, j)<<std::endl;
#endif
				cv::circle(image, cv::Point2d(i, j), 1, cv::Scalar(0, 0, 255), -1);
                        }

		}
	}

	// display the robot pose
	Eigen::Vector3f poseVec = occupiedGridMap->robotPoseWorld2Map( lastMapUpdatePose );
	cv::Point2d pose( poseVec(0), poseVec(1) );
	cv::circle(image, pose, 3, cv::Scalar(0, 255, 0), -1);

	std::cout<<"---------------- Result --------------------"<<std::endl;
        std::cout<<"Occupied Points Number: "<<occupiedCount<<std::endl;

        cv::imshow( "map", image );
}

const Eigen::Vector3f SlamProcessor::getPoseDifferenceValue() const
{
	return poseDiff;
}

bool SlamProcessor::isKeyFrame() const
{
	return keyFrame;
}

void SlamProcessor::reconstructMap( std::vector<Eigen::Vector3f> &keyPoses, std::vector<slam::sensor::LaserScan> &keyScans )
{
	std::cerr<<" -------------- ReConstruct the Map According to the estimated Key Poses --------------"<<std::endl;
	// reset the occupied map
	occupiedGridMap->resetGridMap();

	for( size_t i = 0; i < keyPoses.size(); i ++ ){
		slam::ScanContainer scanContainer;
		
		// TODO ... change
		scanContainer.pointTransform2LaserCoords( keyScans[i] );

		// reconstruct the map	
		occupiedGridMap->updateByScan( scanContainer, keyPoses[i] );
		
	}

}

void SlamProcessor::reconstructMap( std::vector<Eigen::Vector3f> &keyPoses, std::vector<slam::ScanContainer> &scanContainers )
{
	std::cerr<<"-------------- ReConstruct the Map According to the estimated Key Poses ---------------"<<std::endl;
	
	occupiedGridMap->resetGridMap();
	
	occupiedGridMap->updateByScan( scanContainers[0], keyPoses[0] );

	for( size_t i = 0; i < keyPoses.size(); i ++ ){
		// reconstruct the map  
	        occupiedGridMap->updateByScan( scanContainers[i], keyPoses[i] );
	}
}

const Eigen::Matrix<float, 3, 3> SlamProcessor::v2t(const Eigen::Vector3f &v)
{
	float c = ::cos( v(2) );
        float s = ::sin( v(2) );

        Eigen::Matrix<float, 3, 3> A;
        A << c, -s, v(0),
             s,  c, v(1),
             0,  0,  1;

        return A;
}

const Eigen::Vector3f SlamProcessor::t2v(const Eigen::Matrix<float, 3, 3> &A)
{
	Eigen::Vector3f v;
        
	v(0) = A(0, 2);
        v(1) = A(1, 2);
        v(2) = ::atan2( A( 1, 0 ), A(0, 0) );

        return v;
}


void SlamProcessor::displayMap( cv::Mat &image, const std::vector<Eigen::Vector3f> &poses )
{
        int occupiedCount = 0;

        // display the map
        for( int i = 0; i < occupiedGridMap->getSizeX(); i ++ ){
                for( int j = 0; j < occupiedGridMap->getSizeY(); j ++ ){
                        if( occupiedGridMap->isCellFree( i, j ) ){
                                cv::circle(image, cv::Point2d(i, j), 1, cv::Scalar(255, 255, 255), -1);

#ifdef TERMINAL_LOG
                                std::cout<<"Free Point: ( "<<i<<", "<<j<<" )"<<std::endl;
                                std::cout<<"prob: "<<occupiedGridMap->getCellOccupiedProbability( i, j )<<std::endl;
#endif
                        }
                        else if( occupiedGridMap->isCellOccupied( i, j ) ){
                                occupiedCount ++;
#ifdef TERMINAL_LOG
                                std::cout<<"Occupied Point: ( "<<i<<", "<<j<<" )"<<std::endl;
                                std::cout<<"prob: "<<occupiedGridMap->getCellOccupiedProbability( i, j )<<std::endl;
                                std::cout<<"log Odds value: "<<occupiedGridMap->getCellLogOdds(i, j)<<std::endl;
#endif
                                cv::circle(image, cv::Point2d(i, j), 1, cv::Scalar(0, 0, 255), -1);
                        }

                }
        }

        // display the robot pose
	for( auto it : poses ){
        	Eigen::Vector3f poseVec = occupiedGridMap->robotPoseWorld2Map( it );
	        cv::Point2d pose( poseVec(0), poseVec(1) );
        	cv::circle( image, pose, 2, cv::Scalar(0, 255, 0), -1 );
	}
        std::cout<<"---------------- Result --------------------"<<std::endl;
        std::cout<<"Occupied Points Number: "<<occupiedCount<<std::endl;

        cv::imshow( "map", image );

}


const Eigen::Vector3f SlamProcessor::getContraintBetweenTwoPoints( const Eigen::Vector3f &pose1, const Eigen::Vector3f &pose2 )
{
	Eigen::Matrix<float, 3, 3> T1 = v2t( pose1 );
        Eigen::Matrix<float, 3, 3> T2 = v2t( pose2 );

        Eigen::Matrix<float, 3, 3> T = T1.inverse() * T2;
        Eigen::Vector3f V = t2v( T );
        
	std::cout<<"V = "<<std::endl<<V<<std::endl<<std::endl;

	return V;
}

const OccupiedMap SlamProcessor::getOccupiedMap() const
{
	return *occupiedGridMap;
}

void SlamProcessor::saveMapAsBMP( const std::string &fileName )
{
	cv::Mat image = cv::Mat(occupiedGridMap->getSizeX(), occupiedGridMap->getSizeY(), CV_8UC1, cv::Scalar::all(125));


        for( int i = 0; i < occupiedGridMap->getSizeX(); i ++ ){
                for( int j = 0; j < occupiedGridMap->getSizeY(); j ++ ){
                        if( occupiedGridMap->isCellFree( i, j ) ){ // free for 255, white    
                                image.at<uchar>(i, j) = 255;
                        }
                        else if( occupiedGridMap->isCellOccupied( i, j ) ){
                                image.at<uchar>(i, j) = 0; // occupied for 0, black
                        }
                }
        }

        cv::imshow( "bmp map", image );

        cv::imwrite( fileName, image );
	std::cout<<"Save the Map to The BMP file ..."<<std::endl;
}

bool SlamProcessor::saveMap( const std::string &fileName )
{
	std::ofstream outfile;
	outfile.open( fileName, std::ios::binary | std::ios::out );
	if( !outfile.is_open() ){
		std::cerr<<"can not open map file !"<<std::endl;
		return false;
	}

	// 1. first write the map information
	int sizeX = occupiedGridMap->getSizeX();
	int sizeY = occupiedGridMap->getSizeY();
	Eigen::Vector2i center = occupiedGridMap->getMapCenter();

	int centerX = center[0];
	int centerY = center[1];

	float cellLength = occupiedGridMap->getCellLength();
        float scale = occupiedGridMap->getScale();

	// cells' number
        int cellsNumber = occupiedGridMap->getCellsNumber();

        outfile.write( reinterpret_cast<char *>( &sizeX ), sizeof( sizeX ) );
        outfile.write( reinterpret_cast<char *>( &sizeY ), sizeof( sizeY ) );
        outfile.write( reinterpret_cast<char *>( &centerX ), sizeof( centerX ) );
        outfile.write( reinterpret_cast<char *>( &centerY ), sizeof( centerY ) );
        outfile.write( reinterpret_cast<char *>( &cellLength ), sizeof( cellLength ) );
        outfile.write( reinterpret_cast<char *>( &scale ), sizeof( scale ) );
        outfile.write( reinterpret_cast<char *>( &cellsNumber ), sizeof( cellsNumber ) );

	// 2. write the GridCell Operations information
        float logOddsPocc = occupiedGridMap->getLogOddsPoccValue();
        float logOddsPfree = occupiedGridMap->getLogOddsPfreeValue();

        outfile.write( reinterpret_cast<char *>( &logOddsPocc ), sizeof( logOddsPocc ));
        outfile.write( reinterpret_cast<char *>( &logOddsPfree ), sizeof( logOddsPfree ) );

	// 3. write the Occupied Grid Map information

        // 4. write the GridCell information, it is a std::vector
        std::vector<GridCell> mapArray = occupiedGridMap->getMapArray();

        outfile.write( reinterpret_cast<char *>( mapArray.data() ), mapArray.size() * sizeof( GridCell ) );

	outfile.close();

	return true;
}


} // end of namespace slam



























