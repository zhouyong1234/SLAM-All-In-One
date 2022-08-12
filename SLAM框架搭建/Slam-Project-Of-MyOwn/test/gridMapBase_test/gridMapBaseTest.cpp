#include <iostream>
#include "gridMapBase.h"

#include "gridCellOperations.h"
#include "gridCell.h"


int main()
{
	std::cout<<"---------- GridMapBase class Test -----------"<<std::endl;
	slam::GridMapBase<slam::GridCell, slam::GridCellOperations> mapBase(100, 100, 0.1f);

	std::cout<<"---------- Map Information ----------"<<std::endl;
	std::cout<<"Map Size X: "<<mapBase.getSizeX()<<std::endl;
	std::cout<<"Map Size Y: "<<mapBase.getSizeY()<<std::endl;
	std::cout<<"Map Cell Length: "<<mapBase.getCellLength()<<std::endl;
	std::cout<<"Map Center: ( "<<mapBase.getMapCenter()[0]<<", "<<mapBase.getMapCenter()[1]<<" )"<<std::endl;
	std::cout<<"Map Scale: "<<mapBase.getScale()<<std::endl;
	std::cout<<"Cells Number: "<<mapBase.getCellsNumber()<<std::endl;
	
	std::cout<<"---------- Coordinates Transform ------------"<<std::endl;
	Eigen::Vector2f pointInWorld( 5.0f, 5.0f );
	std::cout<<"Point in World Coords: "<<std::endl;
	std::cout<<pointInWorld<<std::endl;
	Eigen::Vector2f transformedPointInMap = mapBase.observedPointPoseWorld2Map( pointInWorld );
	std::cout<<"Transformed Point int Map Coords: "<<std::endl;
	std::cout<<transformedPointInMap<<std::endl;
	std::cout<<std::endl;

	Eigen::Vector2f pointInWorld2( 5.764f, 5.803f );
        std::cout<<"Point in World Coords: "<<std::endl;
        std::cout<<pointInWorld2<<std::endl;
        Eigen::Vector2f transformedPointInMap2 = mapBase.observedPointPoseWorld2Map( pointInWorld2 );
        std::cout<<"Transformed Point int Map Coords: "<<std::endl;
        std::cout<<transformedPointInMap2<<std::endl;
        std::cout<<std::endl;
	
	Eigen::Vector2f pointInMap( 100, 100 );
	std::cout<<"Point in Map Coords: "<<std::endl;
	std::cout<<pointInMap<<std::endl;
	Eigen::Vector2f transformedPointInWorld = mapBase.observedPointPoseMap2World( pointInMap );
	std::cout<<"Transformed Point int World Coords: "<<std::endl;
	std::cout<<transformedPointInWorld<<std::endl;	

	Eigen::Vector3f poseInWorld( 5, 5, M_PI / 2 );
	std::cout<<"Pose in World Coords:"<<std::endl;
	std::cout<<poseInWorld<<std::endl;
	Eigen::Vector3f transformedPoseInMap = mapBase.robotPoseWorld2Map( poseInWorld );
	std::cout<<"Transformed Pose in Map Coords: "<<std::endl;
	std::cout<<transformedPoseInMap<<std::endl;
	
	return 0;
}
