#ifndef __LASER_SIMULATION_H_
#define __LASER_SIMULATION_H_

#include <iostream>
#include <vector>
#include <fstream>
#include <string.h>
#include <sstream>
#include <iomanip>

#include <stdlib.h>
#include <stdint.h>

#include "dataType.h"

namespace slam{

namespace simulation{

typedef struct {
	uint32_t time_stamp;

	float range[360];
}Laser;


static int readLaserTXT(std::string fileName, Laser *scan)
{
	int count = 0;
	std::string input_file_name = fileName;
	std::ifstream input_file(input_file_name.c_str(), std::ifstream::in);

	if (!input_file.is_open()) {
		std::cout << "Failed to open file ..." << std::endl;
		exit(-1);
	}
	std::cout << "open file ..." << std::endl;

	std::string line;
	while (getline(input_file, line)) {
		//std::cout << line << std::endl;
		std::istringstream iss(line);
		std::string tag;
		iss >> tag;
		std::string num;

		// ----------------------------------------------- //

		if (tag.compare("laser") == 0) {
			
			for (int i = 0; i < 360; i++) {
				iss >> num;
				//std::cout << num << "\t";
				//iss >> scan[count].range[i];
				if (!num.compare("inf")) {
					scan[count].range[i] = 65536;
				}
				else{
					scan[count].range[i] = std::stof( num );
				}
			}
			count++;
		}
	}
	return count;
}

class Simulation
{
public:
	Simulation();
	~Simulation();
	
	bool openSimulationFile( const std::string &inputFile );
	void closeSimulationFile();

	bool readAFrameData( slam::sensor::LaserScan &scan );
	
	bool readLaserInfo( slam::sensor::LaserScan &scan );

	inline const int filePointPose()
	{
		return input_file.tellg();
	}

	inline const int endOfFile()
	{
		return input_file.eof();
	}
	
	inline const long getFrameCount() const
	{
		return count;
	}
	
private:
	std::ifstream input_file;
	long count;
};


}

}
#endif
