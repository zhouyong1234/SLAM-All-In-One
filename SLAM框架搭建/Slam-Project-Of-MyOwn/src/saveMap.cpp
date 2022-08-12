#include "saveMap.h"

namespace slam{

SaveMap::SaveMap()
{

}

SaveMap::~SaveMap()
{

}

bool SaveMap::openMapFile( const std::string &fileName )
{
	outfile.open( fileName, std::ios::binary | std::ios::out );

	if( !outfile.is_open() ){
		std::cerr<<"Failed to open the map file ..."<<std::endl;
		
		return false;
	}
	
	std::cerr<<"Open the Map File ..."<<std::endl;
	
	return true;
}

void SaveMap::closeMapFile()
{
	return outfile.close();
}

void SaveMap::setInputMapData( const OccupiedMap &map )
{
	occupiedMap = map;
		
	isSetInputMapData = true;
}

bool SaveMap::saveGridOccupiedMap()
{
	if( !isSetInputMapData ){
		return false;	
	}

	// 1. first write the map information
	int sizeX = occupiedMap.getSizeX();
	int sizeY = occupiedMap.getSizeY();
	Eigen::Vector2i center = occupiedMap.getMapCenter();
	int centerX = center(0);
	int centerY = center(1);
	float cellLength = occupiedMap.getCellLength();
	float scale = occupiedMap.getScale();
	
	// cells' number
	int cellsNumber = occupiedMap.getCellsNumber();

	outfile.write( reinterpret_cast<char *>( &sizeX ), sizeof( sizeX ) );
	outfile.write( reinterpret_cast<char *>( &sizeY ), sizeof( sizeY ) );
	outfile.write( reinterpret_cast<char *>( &centerX ), sizeof( centerX ) );
	outfile.write( reinterpret_cast<char *>( &centerY ), sizeof( centerY ) );
	outfile.write( reinterpret_cast<char *>( &cellLength ), sizeof( cellLength ) );
	outfile.write( reinterpret_cast<char *>( &scale ), sizeof( scale ) ); 	
	outfile.write( reinterpret_cast<char *>( &cellsNumber ), sizeof( cellsNumber ) );

	// 2. write the GridCell Operations information
	float logOddsPocc = occupiedMap.getLogOddsPoccValue();
	float logOddsPfree = occupiedMap.getLogOddsPfreeValue();
	
	outfile.write( reinterpret_cast<char *>( &logOddsPocc ), sizeof( logOddsPocc ));
	outfile.write( reinterpret_cast<char *>( &logOddsPfree ), sizeof( logOddsPfree ) );
	
	// 3. write the Occupied Grid Map information
	// TODO ... not important

	// 4. write the GridCell information, it is a std::vector
	std::vector<GridCell> mapArray = occupiedMap.getMapArray();
		
	outfile.write( reinterpret_cast<char *>( mapArray.data() ), mapArray.size() * sizeof( GridCell ) );

	return true;

}

bool SaveMap::operator()( const std::string &fileName, const OccupiedMap &map )
{
	// 1. open the file 
	if( !openMapFile( fileName ) ){
		return false;
	}

	// 2. set the input occupied grid map
	setInputMapData( map );
	

	// 3. save the occupied grid map
	if( !saveGridOccupiedMap() ){
		return false;
	}

	// 4. close the file
	closeMapFile();	

	return true;

}

}
