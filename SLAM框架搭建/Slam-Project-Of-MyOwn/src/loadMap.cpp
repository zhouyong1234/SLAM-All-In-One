#include "loadMap.h"
#include <assert.h>

namespace slam{

LoadMap::LoadMap()
{

}

LoadMap::~LoadMap()
{

}

bool LoadMap::openMapFile( const std::string &fileName )
{

	infile.open( fileName, std::ios::binary );
	if( !infile.is_open() ){
		std::cerr<<"Failed To Open The Map File ..."<<std::endl;
		
		return false;
	}

	std::cout<<"Open The Map File ..."<<std::endl;

	return true;
}

void LoadMap::closeMapFile()
{
	return infile.close();
}

void LoadMap::loadMap()
{
	// 1. read the map information
	int sizeX = -1;
	int sizeY = -1;
	Eigen::Vector2i center( 0, 0 );
	int centerX = -1;
	int centerY = -1;
	float cellLength = 0.0f;
	float scale = 0.0f;
	int cellsNumber = -1;

	infile.read( reinterpret_cast<char *>( &sizeX ), sizeof( sizeX ) );
	infile.read( reinterpret_cast<char *>( &sizeY ), sizeof( sizeY ) );
	infile.read( reinterpret_cast<char *>( &centerX ), sizeof( centerX ) );
	infile.read( reinterpret_cast<char *>( &centerY ), sizeof( centerY ) );
	infile.read( reinterpret_cast<char *>( &cellLength ), sizeof( cellLength ) );
	infile.read( reinterpret_cast<char *>( &scale ), sizeof( scale ) );
	infile.read( reinterpret_cast<char *>( &cellsNumber ), sizeof( cellsNumber ) );
	
	center = Eigen::Vector2i( centerX, centerY );
	
	occupiedMap.setMapInfo( sizeX, sizeY, cellLength );

	// 2. read the GridCell Operations information
	float logOddsPocc = 0.0f;
	float logOddsPfree = 0.0f;
	
	infile.read( reinterpret_cast<char *>( &logOddsPocc ), sizeof( logOddsPocc ) );	
	infile.read( reinterpret_cast<char *>( &logOddsPfree ), sizeof( logOddsPfree ) );

	occupiedMap.setLogOddsPoccValue( logOddsPocc );
	occupiedMap.setLogOddsPfreeValue( logOddsPfree );

	// 3. read the GridCell information
	std::vector<GridCell> mapArray;
	mapArray.resize( cellsNumber );
	
	infile.read( reinterpret_cast<char *>( mapArray.data() ), cellsNumber * sizeof( GridCell ) );
	std::cout << "cellsNumber: " << cellsNumber << std::endl;

	occupiedMap.setMapArray(mapArray);
}

const OccupiedMap& LoadMap::getMap()
{
	return occupiedMap;
}

const OccupiedMap& LoadMap::operator()( const std::string &fileName )
{
	// 1. open the occupied grid map file
	// assert( openMapFile( fileName ));
	openMapFile(fileName);

	// 2. read the occupied grid map file
	loadMap();

	// 3. return the result
	return occupiedMap;
}



}







