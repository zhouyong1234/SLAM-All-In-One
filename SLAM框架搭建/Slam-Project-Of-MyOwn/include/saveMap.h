#ifndef __SAVE_MAP_H_
#define __SAVE_MAP_H_

#include <iostream>
#include <fstream>

#include "occupiedMap.h"

namespace slam{

class SaveMap
{
public:
	SaveMap();
	~SaveMap();
	
	// Functor
	bool operator()( const std::string &fileName, const OccupiedMap &map );

	bool openMapFile( const std::string &fileName );
	void closeMapFile();
	
	void setInputMapData( const OccupiedMap &map );
	
	bool saveGridOccupiedMap();

private:
	// map object
	OccupiedMap occupiedMap;
	
	// file system
	std::ofstream outfile;

	// 
	bool isSetInputMapData = false; 
};

}

#endif
