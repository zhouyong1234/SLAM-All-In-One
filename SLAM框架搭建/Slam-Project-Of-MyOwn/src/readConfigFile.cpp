#include "readConfigFile.h"

namespace slam{

namespace config{

ReadConfig::ReadConfig()
{

}

ReadConfig::~ReadConfig()
{

}

bool ReadConfig::openSimulationFile( const std::string &inputFile )
{
	input_file.open( inputFile.c_str(), std::ifstream::in );

        if( !input_file.is_open() ){
                std::cout<<"Failed to open the Configuration file ..."<<std::endl;
                return false;
        }

        std::cout<<"............Open the Configuration File ............."<<std::endl;
	
	return true;
}

void ReadConfig::closeSimulationFile()
{
	return input_file.close();
}

void ReadConfig::getConfiguration()
{
	std::string line;
	
	while (getline(input_file, line)) {
                //std::cout << line << std::endl;
                std::istringstream iss(line);
                std::string tag;
                iss >> tag;
                std::string num;

		// get every parameter
	}
}

}

}
