#ifndef __READ_CONFIG_FILE_H_
#define __READ_CONFIG_FILE_H_

#include <iostream>
#include <vector>
#include <fstream>
#include <string.h>
#include <sstream>
#include <iomanip>

#include <stdlib.h>
#include <stdint.h>

namespace slam{

namespace config{

class ReadConfig
{
public:
	ReadConfig();
	~ReadConfig();

	bool openSimulationFile( const std::string &inputFile );
        void closeSimulationFile();
	
	void getConfiguration();

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
	long count = 0;

};

}

}

#endif
