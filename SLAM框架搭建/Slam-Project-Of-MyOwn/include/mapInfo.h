#ifndef __MAP_INFO_H_
#define __MAP_INFO_H_

#include <Eigen/Dense>

namespace slam {

class MapInfo
{
public:
	MapInfo();
	MapInfo( int sizeX_, int sizeY_, float cellLength_ );
	~MapInfo();
	MapInfo( const MapInfo &rhs );
	const MapInfo& operator=( const MapInfo &rhs );

	void setMapInfo( int sizeX, int sizeY, float cellLength );
	void setMapInfo( const MapInfo &newMapInfo );
	
	MapInfo getMapInfo() const;
	int getSizeX() const;
	int getSizeY() const;
	
	Eigen::Vector2i getMapCenter() const;
	float getCellLength() const;
	float getScale() const;	
	
private:
	int sizeX;
	int sizeY;
	Eigen::Vector2i mapCenter;
	float cellLength;
	float scale;
};

}

#endif
