#include "odomSimulation.h"
#include <iostream>

namespace slam{

namespace simulation{

OdomSimulation::OdomSimulation()
{

}

OdomSimulation::~OdomSimulation()
{

}

bool OdomSimulation::openSimulationFile( const std::string &inputFile )
{
	input_file.open( inputFile.c_str(), std::ifstream::in );

        if( !input_file.is_open() ){
                std::cout<<"Failed to open the simulation file ..."<<std::endl;
                return false;
        }

        std::cout<<"............Open the Simulation File ............."<<std::endl;
	
	return true;
}

void OdomSimulation::closeSimulationFile()
{
        return input_file.close();
}

bool OdomSimulation::readAFrameData( Eigen::Vector3f &odom )
{

        std::string line;

        std::getline(input_file, line);
        {
                //std::cout << line << std::endl;
                std::istringstream iss(line);
                std::string tag;
                iss >> tag;
                std::string num;

                if (tag.compare("odom") == 0) {

                        for (int i = 0; i < 3; i++) {
                                iss >> num;
                                //std::cout << num << "\t";
                                odom[i] = std::stof( num );
                        }
                        count++;
                }
        }
}

bool OdomSimulation::openOutputSimulationFile( const std::string &outputFile )
{
	out_file.open( outputFile.c_str(), std::ios::app );
	
	if( !out_file.is_open() ){
		std::cerr<<"Cannot open the output file ..."<<std::endl;
		return false;
	}		
	
	std::cout<<" ------------------ Open the output file ------------------"<<std::endl;
	
	return true;
}


void OdomSimulation::closeOutputSimulationFile()
{
	return out_file.close();
}

bool OdomSimulation::writeAFrameData( const Eigen::Vector3f &odom )
{
	std::cerr<<"Write a frame of data into the output file now ..."<<std::endl;
#ifdef TERMINAL_LOG
	std::cout<<"odom.x     = "<<odom(0)<<std::endl;
	std::cout<<"odom.y     = "<<odom(1)<<std::endl;
	std::cout<<"odom.theta = "<<odom(2)<<std::endl;
	
#endif
	out_file << "odom ";
	out_file << std::setprecision(10) << odom(0) << " " << odom(1) << " " << odom(2);
	out_file << std::endl;
	
	return true;
}


}

}
