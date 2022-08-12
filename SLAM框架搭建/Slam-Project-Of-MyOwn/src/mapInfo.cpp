#include "mapInfo.h"
#include <math.h>

namespace slam {

MapInfo::MapInfo(): sizeX( 1001 ),
		    sizeY( 1001 ),
		    mapCenter( 500, 500 ),
		    cellLength( 0.1f ),
		    scale( 10.0f )
{

}

MapInfo::MapInfo( int sizeX_, int sizeY_, float cellLength_ ):sizeX( sizeX_ ),
							      sizeY( sizeY_  ),
							     cellLength( cellLength_ )
{
	mapCenter = Eigen::Vector2i( static_cast<int>( ::floor( sizeX / 2.0f ) ), static_cast<int>( ::floor( sizeY / 2.0f ) ) );

        scale = 1.0f / this->cellLength;

}

MapInfo::~MapInfo()
{

}

MapInfo::MapInfo( const MapInfo &rhs ) : sizeX( rhs.sizeX ),
					 sizeY( rhs.sizeY ),
					 mapCenter( rhs.mapCenter ),
					 cellLength( rhs.cellLength ),
					 scale( rhs.scale )
{

}

const MapInfo& MapInfo::operator=( const MapInfo &rhs )
{
	if( &rhs == this )
		return *this;
		
	sizeX = rhs.sizeX;
	sizeY = rhs.sizeY;
	mapCenter = rhs.mapCenter;
	cellLength = rhs.cellLength;
	scale = rhs.scale;
	
	return *this;
}

void MapInfo::setMapInfo( int sizeX, int sizeY, float cellLength )
{
	this->sizeX = sizeX;
	this->sizeY = sizeY;
	this->cellLength = cellLength;
	
	mapCenter = Eigen::Vector2i( static_cast<int>( ::floor( sizeX / 2.0f ) ), static_cast<int>( ::floor( sizeY / 2.0f ) ) );

	scale = 1.0f / this->cellLength;
}

void MapInfo::setMapInfo( const MapInfo &newMapInfo )
{
	sizeX = newMapInfo.sizeX;
        sizeY = newMapInfo.sizeY;
        mapCenter = newMapInfo.mapCenter;
        cellLength = newMapInfo.cellLength;
        scale = newMapInfo.scale;

}

MapInfo MapInfo::getMapInfo() const
{
	MapInfo mapInfo;

	mapInfo.sizeX = sizeX;
	mapInfo.sizeY = sizeY;
	mapInfo.mapCenter = mapCenter;
	mapInfo.cellLength = cellLength;
	mapInfo.scale = scale;
	
	return mapInfo;
}

int MapInfo::getSizeX() const
{
	return sizeX;
}

int MapInfo::getSizeY() const
{
	return sizeY;
}

Eigen::Vector2i MapInfo::getMapCenter() const
{
	return mapCenter;
}

float MapInfo::getCellLength() const
{
	return cellLength;
}

float MapInfo::getScale() const
{
	return scale;
}



}
