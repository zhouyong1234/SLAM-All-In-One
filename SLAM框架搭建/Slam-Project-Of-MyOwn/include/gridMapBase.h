#ifndef __GRID_MAP_BASE_H_
#define __GRID_MAP_BASE_H_

#include "mapInfo.h"
#include <vector>
#include <iostream>

#include <memory>

namespace slam {

template<typename CellType, typename CellOperations>
class GridMapBase
{
public:
	using GridType = CellType;
	using GridOperations = CellOperations;
	using Ptr = std::shared_ptr<GridMapBase<CellType, CellOperations>>;

public:
	GridMapBase();

	GridMapBase( const GridMapBase &rhs );
	const GridMapBase& operator=( const GridMapBase &rhs );	

	GridMapBase( const MapInfo &mapInfoCpy );
	GridMapBase( int sizeX, int sizeY, float cellLength );
	~GridMapBase();
	
	void setMapInfo( int sizeX, int sizeY, int cellLength );	
	void setMapInfo( const MapInfo &newMapInfo );

	const MapInfo getMapInfo() const;
        const int getSizeX() const;
        const int getSizeY() const;

	const Eigen::Vector2i getMapCenter() const;
        const float getCellLength() const;
        const float getScale() const;	

	CellType &getCell( int x, int y ) ;
	const CellType &getCell( int x, int y ) const;
	
	CellType &getCell( int index );
	const CellType &getCell( int index ) const;

	const Eigen::Vector2f observedPointPoseWorld2Map( Eigen::Vector2f &poseInWorld ) const;
	const Eigen::Vector2f observedPointPoseMap2World( Eigen::Vector2f &poseInMap ) const;
	
	const Eigen::Vector3f robotPoseWorld2Map( Eigen::Vector3f &poseInWorld ) const;
	const Eigen::Vector3f robotPoseMap2World( Eigen::Vector3f &poseInMap ) const;

	bool isPointOutOfRange( float x, float y ) const;
	bool isPointOutOfRange( const Eigen::Vector2f point ) const;

	void printMapInfo() const;	

	const size_t getCellsNumber() const;

	void setCellOccupied( int index );
	void setCellOccupied( int mapX, int mapY );
	void setCellFree( int index );
	void setCellFree( int mapX, int mapY );
	
	void setCellUnFree( int index );
	void setCellUnFree( int mapX, int mapY );

	const float getCellOccupiedProbability( int index ) const;
	const float getCellOccupiedProbability( int mapX, int mapY ) const;

	void setLogOddsPoccValue( float Pocc );
	void setLogOddsPfreeValue( float Pfree );
	const float getLogOddsPoccValue() const;
	const float getLogOddsPfreeValue() const;
	

	bool isCellOccupied( int index );
	bool isCellOccupied( int mapX, int mapY );
	bool isCellFree( int index );
	bool isCellFree( int mapX, int mapY );

	const float getCellLogOdds(int mapX, int mapY);	

	const std::vector<CellType> getMapArray() const;
	
	void setMapArray( const std::vector<CellType> &mapArray );

protected:
	void allocateMapArray();
	void deleteMapArray();

	void resizeMap();
	
	void clear();	

protected:
	std::vector<CellType> mapArray;
	
	CellOperations cellOperate;
	
	MapInfo mapInfo;
};


template<typename CellType, typename CellOperations>
GridMapBase<CellType, CellOperations>::GridMapBase()
{
	allocateMapArray();
}

template<typename CellType, typename CellOperations>
GridMapBase<CellType, CellOperations>::GridMapBase( const MapInfo &mapInfoCpy ): mapInfo( mapInfoCpy )
{
        allocateMapArray();
}


template<typename CellType, typename CellOperations>
GridMapBase<CellType, CellOperations>::GridMapBase( int sizeX, int sizeY, float cellLength ) : mapInfo( sizeX, sizeY, cellLength )
{
        allocateMapArray();
}

template<typename CellType, typename CellOperations>
GridMapBase<CellType, CellOperations>::GridMapBase( const GridMapBase &rhs ): mapInfo( rhs.mapInfo ),
									      cellOperate( rhs.cellOperate ),
									      mapArray( rhs.mapArray )
{
	
}

template<typename CellType, typename CellOperations>
const GridMapBase<CellType, CellOperations>& GridMapBase<CellType, CellOperations>::operator=( const GridMapBase &rhs )
{
	if( &rhs == this ){
		return *this;
	}

	mapInfo = rhs.mapInfo;
	cellOperate = rhs.cellOperate;
	mapArray = rhs.mapArray;
		
	return *this;
}


template<typename CellType, typename CellOperations>
GridMapBase<CellType, CellOperations>::~GridMapBase()
{

}

template<typename CellType, typename CellOperations>
void GridMapBase<CellType, CellOperations>::setMapInfo( int sizeX, int sizeY, int cellLength )
{
	return mapInfo.setMapInfo( sizeX, sizeY, cellLength );
}

template<typename CellType, typename CellOperations>
void GridMapBase<CellType, CellOperations>::setMapInfo( const MapInfo &newMapInfo )
{
	return mapInfo.setMapInfo( newMapInfo );
}

template<typename CellType, typename CellOperations>
const MapInfo GridMapBase<CellType, CellOperations>::getMapInfo() const
{
	return mapInfo.getMapInfo();
}

template<typename CellType, typename CellOperations>
const int GridMapBase<CellType, CellOperations>::getSizeX() const
{
	return mapInfo.getSizeX();
}

template<typename CellType, typename CellOperations>
const int GridMapBase<CellType, CellOperations>::getSizeY() const
{
	return mapInfo.getSizeY();
}

template<typename CellType, typename CellOperations>
const Eigen::Vector2i GridMapBase<CellType, CellOperations>::getMapCenter() const
{
	return mapInfo.getMapCenter();
}

template<typename CellType, typename CellOperations>
const float GridMapBase<CellType, CellOperations>::getCellLength() const
{
	return mapInfo.getCellLength();
}

template<typename CellType, typename CellOperations>
const float GridMapBase<CellType, CellOperations>::getScale() const
{
	return mapInfo.getScale();
}

template<typename CellType, typename CellOperations>
void GridMapBase<CellType, CellOperations>::allocateMapArray()
{
	int size = getSizeX() * getSizeY();
	
	//CellType cell( 0.0f, 0 );
	//mapArray.resize( size, cell );
	mapArray.resize( size );	

	if( mapArray.size() != size ){
		std::cerr<<"allocate the memory for the map failed ..."<<std::endl;
                exit(-1);
	}
	std::cout<<"allocate the memory for the map ..."<<std::endl;
}

template<typename CellType, typename CellOperations>
void GridMapBase<CellType, CellOperations>::deleteMapArray()
{
	if( mapArray.empty() ){
		mapArray.clear();
		
		std::vector<CellType>().swap( mapArray );
		
		std::cerr<<"delete the memory for the map Array ..."<<std::endl;
	}
}

template<typename CellType, typename CellOperations>
void GridMapBase<CellType, CellOperations>::clear()
{
	mapArray.clear();
}

template<typename CellType, typename CellOperations>
CellType& GridMapBase<CellType, CellOperations>::getCell( int x, int y )
{
	//return mapArray[ (y - 1) * this->getSizeX() + ( x - 1 ) ];
	return mapArray[ y * this->getSizeX() + x ];
}

template<typename CellType, typename CellOperations>
const CellType& GridMapBase<CellType, CellOperations>::getCell( int x, int y ) const
{
	//return mapArray[ (y - 1) * this->getSizeX() + ( x - 1 ) ];
	return mapArray[ y * this->getSizeX() + x ];
}

template<typename CellType, typename CellOperations>
CellType& GridMapBase<CellType, CellOperations>::getCell( int index )
{
	return mapArray[ index ];
}

template<typename CellType, typename CellOperations>
const CellType& GridMapBase<CellType, CellOperations>::getCell( int index ) const
{
	return mapArray[ index ];
}


template<typename CellType, typename CellOperations>
const Eigen::Vector2f GridMapBase<CellType, CellOperations>::observedPointPoseWorld2Map( Eigen::Vector2f &poseInWorld ) const
{
	Eigen::Vector2f temp( static_cast<float>( this->getMapCenter()[0] ), static_cast<float>( this->getMapCenter()[1] )  );

	return poseInWorld * this->getScale() + temp;
}

template<typename CellType, typename CellOperations>
const Eigen::Vector2f GridMapBase<CellType, CellOperations>::observedPointPoseMap2World( Eigen::Vector2f &poseInMap ) const
{
	Eigen::Vector2f temp( static_cast<float>( this->getMapCenter()[0] ), static_cast<float>( this->getMapCenter()[1] )  );

	return ( poseInMap - temp ) * this->getCellLength();
}

template<typename CellType, typename CellOperations>
const Eigen::Vector3f GridMapBase<CellType, CellOperations>::robotPoseWorld2Map( Eigen::Vector3f &poseInWorld ) const
{
	Eigen::Vector2f temp( static_cast<float>( this->getMapCenter()[0] ), static_cast<float>( this->getMapCenter()[1] )  );	

	Eigen::Vector2f mapCoords( poseInWorld.head<2>() * this->getScale() + temp );
	
	return Eigen::Vector3f( mapCoords[0], mapCoords[1], poseInWorld[2] );
}

template<typename CellType, typename CellOperations>
const Eigen::Vector3f GridMapBase<CellType, CellOperations>::robotPoseMap2World( Eigen::Vector3f &poseInMap ) const
{
	Eigen::Vector2f temp( static_cast<float>( this->getMapCenter()[0] ), static_cast<float>( this->getMapCenter()[1] )  );	

	Eigen::Vector2f worldCoords( ( poseInMap.head<2>() - temp ) * this->getCellLength() );

	return Eigen::Vector3f( worldCoords[0], worldCoords[1], poseInMap[2] );
}

template<typename CellType, typename CellOperations>
bool GridMapBase<CellType, CellOperations>::isPointOutOfRange(float x, float y) const
{
	return ( x < 0.0f || y < 0.0f || x > static_cast<float>( this->getSizeX() ) || y > static_cast<float>( this->getSizeY() ) );
}

template<typename CellType, typename CellOperations>
bool GridMapBase<CellType, CellOperations>::isPointOutOfRange( const Eigen::Vector2f point ) const
{
	return ( point[0] < 0.0f || point[1] < 0.0f || point[0] > static_cast<float>( this->getSizeX() ) || point[1] > static_cast<float>( this->getSizeY() ) );
}

template<typename CellType, typename CellOperations>
void GridMapBase<CellType, CellOperations>::printMapInfo() const
{
	std::cout<<"---------- Map Information ----------"<<std::endl;
        std::cout<<"Map Size X: "<<this->getSizeX()<<std::endl;
        std::cout<<"Map Size Y: "<<this->getSizeY()<<std::endl;
        std::cout<<"Map Cell Length: "<<this->getCellLength()<<std::endl;
        std::cout<<"Map Center: ( "<<this->getMapCenter()[0]<<", "<<this->getMapCenter()[1]<<" )"<<std::endl;
        std::cout<<"Map Scale: "<<this->getScale()<<std::endl;

	std::cout<<"Cells Number: "<<this->getCellsNumber()<<std::endl;
}

template<typename CellType, typename CellOperations>
const size_t GridMapBase<CellType, CellOperations>::getCellsNumber() const
{
	return mapArray.size();
}

template<typename CellType, typename CellOperations>
void GridMapBase<CellType, CellOperations>::setCellOccupied( int index )
{
	//CellType &cell = this->getCell( index );
        //cellOperate.setCellOccupied( cell );

	return cellOperate.setCellOccupied( this->getCell( index ) );
}

template<typename CellType, typename CellOperations>
void GridMapBase<CellType, CellOperations>::setCellOccupied( int mapX, int mapY ) 
{
	//CellType &cell = this->getCell( mapX, mapY );
	//cellOperate.setCellOccupied( cell );
	return cellOperate.setCellOccupied( this->getCell( mapX, mapY ) );
}

template<typename CellType, typename CellOperations>
void GridMapBase<CellType, CellOperations>::setCellFree( int index )
{
	//CellType &cell = this->getCell( index );
	//cellOperate.setCellFree( cell );
	return cellOperate.setCellFree( this->getCell( index ) );
}

template<typename CellType, typename CellOperations>
void GridMapBase<CellType, CellOperations>::setCellFree( int mapX, int mapY )
{
	//CellType &cell = this->getCell( mapX, mapY );
        //cellOperate.setCellFree( cell );
	return cellOperate.setCellFree(  this->getCell( mapX, mapY ));
}	

template<typename CellType, typename CellOperations>
void GridMapBase<CellType, CellOperations>::setCellUnFree( int index )
{
	return cellOperate.setCellUnFree( this->getCell( index ) );
}

template<typename CellType, typename CellOperations>
void GridMapBase<CellType, CellOperations>::setCellUnFree( int mapX, int mapY )
{
	return cellOperate.setCellUnFree( this->getCell( mapX, mapY ) );
}


template<typename CellType, typename CellOperations>
const float GridMapBase<CellType, CellOperations>::getCellOccupiedProbability( int index ) const
{
	return cellOperate.getCellProbability( this->getCell( index ) );
}

template<typename CellType, typename CellOperations>
const float GridMapBase<CellType, CellOperations>::getCellOccupiedProbability( int mapX, int mapY ) const
{
	return cellOperate.getCellProbability( this->getCell( mapX, mapY ) );
}

template<typename CellType, typename CellOperations>
void GridMapBase<CellType, CellOperations>::setLogOddsPoccValue( float Pocc )
{
	return cellOperate.setLogOddsPocc( Pocc );
}

template<typename CellType, typename CellOperations>
void GridMapBase<CellType, CellOperations>::setLogOddsPfreeValue( float Pfree )
{
	return cellOperate.setLogOddsPfree( Pfree );
}

template<typename CellType, typename CellOperations>
const float GridMapBase<CellType, CellOperations>::getLogOddsPoccValue() const
{
	return cellOperate.getLogOddsPocc();
}

template<typename CellType, typename CellOperations>
const float GridMapBase<CellType, CellOperations>::getLogOddsPfreeValue() const
{
	return cellOperate.getLogOddsPfree();
}

template<typename CellType, typename CellOperations>
bool GridMapBase<CellType, CellOperations>::isCellOccupied( int index )
{
	return ( this->getCell( index ).isOccupied() );
}

template<typename CellType, typename CellOperations>
bool GridMapBase<CellType, CellOperations>::isCellOccupied( int mapX, int mapY )
{
        return ( this->getCell( mapX, mapY ).isOccupied() );
}

template<typename CellType, typename CellOperations>
bool GridMapBase<CellType, CellOperations>::isCellFree( int index )
{
        return ( this->getCell( index ).isFree() );
}

template<typename CellType, typename CellOperations>
bool GridMapBase<CellType, CellOperations>::isCellFree( int mapX, int mapY )
{
        return ( this->getCell( mapX, mapY ).isFree() );
}

template<typename CellType, typename CellOperations>
const float GridMapBase<CellType, CellOperations>::getCellLogOdds( int mapX, int mapY )
{
	return (this->getCell( mapX, mapY ).logOddsValue);
}

template<typename CellType, typename CellOperations>
const std::vector<CellType> GridMapBase<CellType, CellOperations>::getMapArray() const
{
	return mapArray;
}

template<typename CellType, typename CellOperations>
void GridMapBase<CellType, CellOperations>::setMapArray( const std::vector<CellType> &mapArray )
{
	this->mapArray = mapArray;
}

}




#endif
