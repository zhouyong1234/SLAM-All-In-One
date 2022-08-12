#ifndef __LOAD_MAP_H_
#define __LOAD_MAP_H_

#include <iostream>
#include <fstream>

#include "occupiedMap.h"


namespace slam{

class LoadMap
{
public:	
	LoadMap();
	~LoadMap();
	
	// Functor
	const OccupiedMap& operator()( const std::string &fileName);

private:
	bool openMapFile( const std::string &fileName );
    void closeMapFile();

	void loadMap();
	
	const OccupiedMap& getMap();
	
private:
	// map object
        OccupiedMap occupiedMap;

        // file system
        std::ifstream infile;	

};

}

#endif

