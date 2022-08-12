#ifndef __OCCUPIED_GRID_MAP_H_
#define __OCCUPIED_GRID_MAP_H_

#include <iostream>

#include "gridCell.h"
#include "mapInfo.h"
#include <cmath>

#include "scanContainer.h"

#include <memory>

namespace slam {

template<typename MapBaseType>
class OccupiedGridMap : public MapBaseType
{
public:
	using MapType = MapBaseType;
	using OccupiedGridMapType = OccupiedGridMap<MapBaseType>;
	
	using Ptr = std::shared_ptr<OccupiedGridMap<MapBaseType>>;	

public:
	OccupiedGridMap();
	
	OccupiedGridMap( const OccupiedGridMap &rhs );
	OccupiedGridMap& operator=( const OccupiedGridMap &rhs );

	OccupiedGridMap( int sizeX_, int sizeY, float cellLength_ );	
	OccupiedGridMap( const MapInfo &mapInfo );

	~OccupiedGridMap();

	const Eigen::Vector2f observedPointPoseLaser2World( Eigen::Vector2f &poseInLaser, Eigen::Vector3f &robotPoseInWorld ) const;
	const Eigen::Vector2f observedPointPoseWorld2Laser( Eigen::Vector2f &poseInWorld, Eigen::Vector3f &robotPoseInWorld ) const;

	void inverseModel( int x0, int y0, int x1, int y1 );
	void inverseModel( Eigen::Vector2i &p0, Eigen::Vector2i &p1 );
	
	void updateByScan_test( ScanContainer &points, Eigen::Vector3f &robotPoseInWorld );
	
	void updateByScan( ScanContainer &points, Eigen::Vector3f &robotPoseInWorld );
	void resetGridMap();

private:
	void bresenHam( int x0, int y0, int x1, int y1 );
	void bresenHam( Eigen::Vector2i &p0, Eigen::Vector2i &p1);

	void bresenhamCellFree( int index );
	void bresenhamCellOccupied( int index );
	void bresenhamCellFree( int mapX, int mapY );
        void bresenhamCellOccupied( int mapX, int mapY );

private:
	int currUpdateIndex;
	int currMarkOccIndex;
	int currMarkFreeIndex;
};

template<typename MapBaseType>
OccupiedGridMap<MapBaseType>::OccupiedGridMap() : MapBaseType(), currUpdateIndex(0), currMarkOccIndex(-1), currMarkFreeIndex(-1)
{

}

template<typename MapBaseType>
OccupiedGridMap<MapBaseType>::OccupiedGridMap( int sizeX_, int sizeY_, float cellLength_ ) : MapBaseType( sizeX_, sizeY_, cellLength_ ), 
											     currUpdateIndex( 0 ), 
											     currMarkOccIndex(-1), 
											     currMarkFreeIndex(-1)
{

}

template<typename MapBaseType>
OccupiedGridMap<MapBaseType>::OccupiedGridMap( const MapInfo &mapInfo ) : MapBaseType( mapInfo ), 
									  currUpdateIndex(0), 
									  currMarkOccIndex(-1), 
									  currMarkFreeIndex(-1)
{

}

template<typename MapBaseType>
OccupiedGridMap<MapBaseType>::OccupiedGridMap( const OccupiedGridMap &rhs ): MapBaseType( rhs ),
									     currUpdateIndex( rhs.currUpdateIndex ),
									     currMarkOccIndex(rhs.currMarkOccIndex), 
									     currMarkFreeIndex(rhs.currMarkFreeIndex)
{

} 

template<typename MapBaseType>
OccupiedGridMap<MapBaseType>& OccupiedGridMap<MapBaseType>::operator=( const OccupiedGridMap &rhs )
{
	if( &rhs == this ){
		return *this;
	}

	currUpdateIndex = rhs.currUpdateIndex;
	currMarkOccIndex = rhs.currMarkOccIndex;
	currMarkFreeIndex = rhs.currMarkFreeIndex;
	
	this->MapBaseType::operator=( rhs );

	return *this;	
}


template<typename MapBaseType>
OccupiedGridMap<MapBaseType>::~OccupiedGridMap()
{

}

template<typename MapBaseType>
void OccupiedGridMap<MapBaseType>::bresenHam( int x0, int y0, int x1, int y1 )
{
	int dx = ::abs( x1 - x0 );
	int dy = ::abs( y1 - y0 );
	
	bool interChange = false;
	
	int e = -dx;// error

	int signX = x1 > x0 ? 1 : ( ( x1 < x0 ) ? -1 : 0 );
	int signY = y1 > y0 ? 1 : ( ( y1 < y0 ) ? -1 : 0 );

	if (dy > dx) {
		int temp = dx;
		dx = dy;
		dy = temp;
		interChange = true;
	}

	int x = x0, y = y0;
	for (int i = 1; i <= dx; i++) { // not include the end point
		//operate( x, y, img1 );
		// add operations for the point 
		bresenhamCellFree( x, y );		
		
		///////////////////////////////
		if (!interChange)
			x += signX;
		else
			y += signY;

		e += 2 * dy;

		if (e >= 0) {
			if (!interChange)
				y += signY;
			else
				x += signX;

			e -= 2 * dx;
		}
	}
}

template<typename MapBaseType>
void OccupiedGridMap<MapBaseType>::bresenHam( Eigen::Vector2i &p0, Eigen::Vector2i &p1 )
{
	int x0 = p0[0];
	int y0 = p0[1];
	int x1 = p1[0];
	int y1 = p1[1];

	bresenHam( x0, y0, x1, y1 );

}

template<typename MapBaseType>
void OccupiedGridMap<MapBaseType>::inverseModel( int x0, int y0, int x1, int y1 )
{
	// 1. set the end point occupied first
	bresenhamCellOccupied( x1, y1 );

#ifdef TERMINAL_LOG
	std::cout<<"Occupied Cell Pose In Map: ( "<< x1 <<", "<< y1<<" )"<<std::endl;
#endif

	// 2. execute the bresenham algorithm, find the points of free, and set them free
	bresenHam( x0, y0, x1, y1 );
}

template<typename MapBaseType>
void OccupiedGridMap<MapBaseType>::inverseModel( Eigen::Vector2i &p0, Eigen::Vector2i &p1 )
{
	int x0 = p0[0];
        int y0 = p0[1];
        int x1 = p1[0];
        int y1 = p1[1];
	
	inverseModel( x0, y0, x1, y1 );
}

template<typename MapBaseType>
void OccupiedGridMap<MapBaseType>::bresenhamCellFree( int index )
{
	GridCell &cell = this->getCell( index );	
	
	if( cell.updateIndex < currMarkFreeIndex ){
		this->setCellFree( index );
		
		cell.updateIndex = currMarkFreeIndex; // avoid reUpdate
	}
}

template<typename MapBaseType>
void OccupiedGridMap<MapBaseType>::bresenhamCellOccupied( int index )
{
	GridCell &cell = this->getCell( index );
	
	if( cell.updateIndex < currMarkOccIndex ){
		if( cell.updateIndex == currMarkFreeIndex ){
			this->setCellUnFree();	
		}
                this->setCellOccupied( index );

                cell.updateIndex = currMarkOccIndex; // avoid reUpdate
        }

}

template<typename MapBaseType>
void OccupiedGridMap<MapBaseType>::bresenhamCellFree( int mapX, int mapY )
{
        GridCell &cell = this->getCell( mapX, mapY );
	
	if( cell.updateIndex < currMarkFreeIndex ){
                this->setCellFree( mapX, mapY );

                cell.updateIndex = currMarkFreeIndex; // avoid reUpdate
        }

}

template<typename MapBaseType>
void OccupiedGridMap<MapBaseType>::bresenhamCellOccupied( int mapX, int mapY )
{
	GridCell &cell = this->getCell( mapX, mapY );

        if( cell.updateIndex < currMarkOccIndex ){
		if( cell.updateIndex == currMarkFreeIndex ){
			this->setCellUnFree( mapX, mapY );
		}
                this->setCellOccupied( mapX, mapY );

                cell.updateIndex = currMarkOccIndex; // avoid reUpdate
        }

}

template<typename MapBaseType>
const Eigen::Vector2f OccupiedGridMap<MapBaseType>::observedPointPoseLaser2World( Eigen::Vector2f &poseInLaser, Eigen::Vector3f &robotPoseInWorld ) const
{
	//Eigen::Vector2f poseInWorld_temp( ( ::cos( robotPoseInWorld[2] ) * poseInLaser[0] - ::sin( robotPoseInWorld[2] ) * poseInLaser[1] ), ( ::sin( robotPoseInWorld[2] ) * poseInLaser[0] + ::cos( robotPoseInWorld[2] ) * poseInLaser[1] ) );

	//return poseInWorld_temp + robotPoseInWorld.head<2>();	

	Eigen::Matrix2f rotateMat;
	rotateMat << ::cos( robotPoseInWorld[2] ), -(::sin( robotPoseInWorld[2] )),
			::sin( robotPoseInWorld[2] ), ::cos( robotPoseInWorld[2] );

	return rotateMat * poseInLaser + robotPoseInWorld.head<2>();
}

template<typename MapBaseType>
const Eigen::Vector2f OccupiedGridMap<MapBaseType>::observedPointPoseWorld2Laser( Eigen::Vector2f &poseInWorld, Eigen::Vector3f &robotPoseInWorld ) const
{
	Eigen::Matrix2f rotateMat;
        rotateMat << ::cos( robotPoseInWorld[2] ), -(::sin( robotPoseInWorld[2] )),
                        ::sin( robotPoseInWorld[2] ), ::cos( robotPoseInWorld[2] );

	return rotateMat.inverse() * poseInWorld - robotPoseInWorld.head<2>();
}


template<typename MapBaseType>
void OccupiedGridMap<MapBaseType>::updateByScan_test( ScanContainer &points, Eigen::Vector3f &robotPoseInWorld )
{
	currUpdateIndex ++;
	
	// 1. Transform robot Pose In world Coordinate to Map Coordinate
	Eigen::Vector3f robotPoseInMap = this->robotPoseWorld2Map( robotPoseInWorld );

#ifdef TERMINAL_LOG
	std::cout<<"Robot Pose In Map Coordinate: "<<std::endl;
	std::cout<<robotPoseInMap<<std::endl;
#endif

	// 2. Get the start point of the laser data in Map Coordinate
	Eigen::Vector2i scanBeginMapI( robotPoseInMap.head<2>().cast<int>() );

#ifdef TERMINAL_LOG
	std::cout<<"Robot Pose In Map Coordinate(Interger): "<<std::endl;
	std::cout<<scanBeginMapI<<std::endl;
#endif

	size_t numberOfBeams = points.getSize();

#ifdef TERMINAL_LOG
	std::cout<<"Number Of Beams: "<<numberOfBeams<<std::endl;
#endif	

	for( size_t i = 0; i < numberOfBeams; i ++ ){
		// 3. Get the End point of Every Laser Beam in Laser Coordinate
		Eigen::Vector2f scanEndInLaser( points.getIndexData( i ) );

#ifdef TERMINAL_LOG
		std::cout<<"Occupied Point In World ( "<<scanEndInLaser[0]<<", "<<scanEndInLaser[1]<<" )"<<std::endl;		
#endif

		// 4. Transform the End Point from Laser Coordinate to World Coordinate
		Eigen::Vector2f scanEndInWorld( this->observedPointPoseLaser2World( scanEndInLaser, robotPoseInWorld ) );

		// 5. Transform the End Point from World Coordinate to Map Coordinate
		Eigen::Vector2f scanEndInMap( this->observedPointPoseWorld2Map( scanEndInWorld ) );

		// 6. Convert float to interger
		//Eigen::Vector2i scanEndInMapI( scanEndInMap.cast<int>() );
		Eigen::Vector2i scanEndInMapI( static_cast<int>( ::round( scanEndInMap[0] ) ), static_cast<int>( ::round( scanEndInMap[1] ) ) );

		// 7. execuate Inverse Model algorithm
		if( scanEndInMapI != scanBeginMapI ){
			this->inverseModel( scanBeginMapI, scanEndInMapI );
		}
	}
	
}

template<typename MapBaseType>
void OccupiedGridMap<MapBaseType>::updateByScan( ScanContainer &points, Eigen::Vector3f &robotPoseInWorld )
{
	// P(m | x_1:t, z_1:t)
	// m = { m1, m2, ..., m_n }
	// P(m | x_1:t, z_1:t) = P(m1, m2, ..., m_n | x_1:t, z_1:t)
	// Pi(i = 1 to n){ P(m_i | x_1:t, z_1:t) }
	//
	// grid occupied probability: 
	// bel_t(m_i) = P( m_i | x_1:t, z_1:t )
	// 	      = { P( z_t | m_i, x_1:t, z_1:t-1 ) * P( m_i | x_1:t, z_1:t-1 ) } / P( z_t | x_1:t, z_1:t-1 ) 
	//	      = { P( z_t | m_i, x_t ) * P( m_i | x_1:t-1, z_1:t-1 ) } / P( z_t | x_1:t, z_1:t-1 ) 
	// 	      = { P( m_i | z_t, x_t ) * P( z_t | x_t ) * P( m_i | x_1:t-1, z_1:t-1 ) } / { P( m_i | x_t ) * P( z_t | x_1:t, z_1:t-1 ) }
	// 	      = { P( m_i | z_t, x_t ) * P( z_t | x_t ) * P( m_i | x_1:t-1, z_1:t-1 ) } / { P( m_i ) * P( z_t | x_1:t, z_1:t-1 ) } 
	// 	      = { P( m_i | z_t, x_t ) * P( z_t | x_t ) * bel_t-1(m_i) } / { P( m_i ) * P( z_t | x_1:t, z_1:t-1 ) }
	//
	// grid free probability:
	// bel_t(~m_i) =  1 - bel_t(m_i)
	//	       = { P( ~m_i | z_t, x_t ) * P( z_t | x_t ) * bel_t-1(~m_i) } / { P( ~m_i ) * P( z_t | x_1:t, z_1:t-1 ) }	
	//
	// bel_t(m_i) / bel_t(~m_i) = bel_t(m_i) / 1 - bel_t(m_i)
	// 			    = { P( m_i | z_t, x_t ) * bel_t(m_i) * [ 1 - P(m_i) ] } / { [ 1 - P( m_i | z_t, x_t ) ] * [ 1 - bel_t(m_i) ] * p(m_i) }
	//
	// l_t,i = log{ bel_t(m_i) / 1 - bel_t(m_i) }
	//	 = log{ P( m_i | z_1:t, x_1:t ) / { 1 - P( m_i | z_1:t, x_1:t ) } }
	//
	// inverse model:
	// l_inv,i = log{ P( m_i | z_t, x_t ) / { 1 - P( m_i | z_t, x_t ) } }
	// 
	// Prior probability:
	// l_0 = log{ P( m_i ) / { 1 - P( m_i ) } }
	//
	// l_t,i = l_t-1,i + l_inv,i - l_0
	//
	// bel_t(m_i) = 1 - { 1 / ( 1 + exp( l_t,i ) ) } 
	
        //currUpdateIndex ++;
	currMarkFreeIndex = currUpdateIndex + 1;
	currMarkOccIndex = currUpdateIndex + 2;
	
        // 1. Transform robot Pose In world Coordinate to Map Coordinate
        Eigen::Vector3f robotPoseInMap = this->robotPoseWorld2Map( robotPoseInWorld );

#ifdef TERMINAL_LOG 
        std::cout<<"Robot Pose In Map Coordinate: "<<std::endl;
        std::cout<<robotPoseInMap<<std::endl;
#endif

        // 2. Get the start point of the laser data in Map Coordinate
        Eigen::Vector2i scanBeginMapI( robotPoseInMap.head<2>().cast<int>() );
        
#ifdef TERMINAL_LOG
	std::cout<<"Robot Pose In Map Coordinate(Interger): "<<std::endl;
        std::cout<<scanBeginMapI<<std::endl;
#endif

        size_t numberOfBeams = points.getSize();
        std::cout<<"Number Of Beams: "<<numberOfBeams<<std::endl;

        for( size_t i = 0; i < numberOfBeams; i ++ ){
                // 3. Get the End point of Every Laser Beam in Laser Coordinate
                Eigen::Vector2f scanEndInLaser( points.getIndexData( i ) );

#ifdef TERMINAL_LOG
                std::cout<<"Occupied Point In World ( "<<scanEndInLaser[0]<<", "<<scanEndInLaser[1]<<" )"<<std::endl;
#endif

                // 4. Transform the End Point from Laser Coordinate to World Coordinate
                Eigen::Vector2f scanEndInWorld( this->observedPointPoseLaser2World( scanEndInLaser, robotPoseInWorld ) );

                // 5. Transform the End Point from World Coordinate to Map Coordinate
                Eigen::Vector2f scanEndInMap( this->observedPointPoseWorld2Map( scanEndInWorld ) );

                // 6. Convert float to interger
                //Eigen::Vector2i scanEndInMapI( scanEndInMap.cast<int>() );
                Eigen::Vector2i scanEndInMapI( static_cast<int>( ::round( scanEndInMap[0] ) ), static_cast<int>( ::round( scanEndInMap[1] ) ) );

                // 7. execuate Inverse Model algorithm
                if( scanEndInMapI != scanBeginMapI ){
                        this->inverseModel( scanBeginMapI, scanEndInMapI );
                }
        }
	
	currUpdateIndex += 3;

}

template<typename MapBaseType>
void OccupiedGridMap<MapBaseType>::resetGridMap()
{
	return this->clear();
}

}



#endif
