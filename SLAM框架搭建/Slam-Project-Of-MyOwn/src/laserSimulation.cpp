#include "laserSimulation.h"

namespace slam{

namespace simulation{

Simulation::Simulation(): count( 0 )
{

}

Simulation::~Simulation()
{

}

bool Simulation::openSimulationFile( const std::string &inputFile )
{
	input_file.open( inputFile.c_str(), std::ifstream::in );
	
	if( !input_file.is_open() ){
		std::cout<<"Failed to open the simulation file ..."<<std::endl;
		return false;
	}
	
	std::cout<<"............Open the Simulation File ............."<<std::endl;

}

void Simulation::closeSimulationFile()
{
	return input_file.close();
}

bool Simulation::readAFrameData( slam::sensor::LaserScan &scan )
{
	//memset( &scan, 0, sizeof( scan ) );	
	memset( scan.ranges, 0, scan.size() );

	std::string line;

	std::getline(input_file, line);
	{
		//std::cout << line << std::endl;
                std::istringstream iss(line);
                std::string tag;
                iss >> tag;
                std::string num;
	
                if (tag.compare("laser") == 0) {
		
			for (int i = 0; i < scan.size(); i++) {
                                iss >> num;
                                //std::cout << num << "\t";
                                //iss >> scan[count].range[i];
                                if (!num.compare("inf")) {
                                        scan.ranges[i] = 65536.0f;
                                }
                                else{
                                        scan.ranges[i] = std::stof( num );
                                }
                        }
                        count++;
			
		}
	}
}

bool Simulation::readLaserInfo( slam::sensor::LaserScan &scan )
{
	memset( &scan, 0, sizeof( scan ) );

	std::string line;
	
	for( int i = 0; i < 7; i ++ ){
		std::getline( input_file, line );

		std::istringstream iss( line );

		std::string tag;
		
		iss >> tag;
		
		std::string num;

		if( tag.compare( "angle_min:" ) == 0 ){
			iss >> num;
			std::cout<<"angle_min: "<<num<<std::endl;
			
			scan.angle_min = std::stof( num );
		}

		if( tag.compare( "angle_max:" ) == 0 ){
			iss >> num;
			std::cout<<"angle_max: "<<num<<std::endl;
	
			scan.angle_max = std::stof( num );
		}

		if( tag.compare( "angle_increment:" ) == 0 ){
			iss >> num;
			std::cout<<"angle_increment: "<<num<<std::endl;
	
			scan.angle_increment = std::stof( num );
		}

		if( tag.compare( "range_min:" ) == 0 ){
			iss >> num;
			std::cout<<"range_min: "<<num<<std::endl;
		
			scan.angle_min = std::stof( num );
		}
	
		if( tag.compare( "range_max:" ) == 0 )	{
			iss >> num;
			std::cout<<"range_max: "<<num<<std::endl;

			scan.angle_max = std::stof( num );
		}

	}

	return true;
}

}

}
