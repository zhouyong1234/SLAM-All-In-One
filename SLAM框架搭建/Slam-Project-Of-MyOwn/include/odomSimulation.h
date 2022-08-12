#ifndef __ODOM_SIMULATION_H_
#define __ODOM_SIMULATION_H_

#include <fstream>
#include <string.h>
#include <sstream>
#include <iomanip>

#include <stdlib.h>
#include <stdint.h>

#include <Eigen/Dense>

namespace slam{

namespace simulation{

class OdomSimulation
{
public:
	OdomSimulation();
	~OdomSimulation();

	bool openSimulationFile( const std::string &inputFile );
        void closeSimulationFile();

	bool readAFrameData( Eigen::Vector3f &odom );

	// added for ouputing the data of laser odometry
	bool openOutputSimulationFile( const std::string &outputFile );
	void closeOutputSimulationFile();

	bool writeAFrameData( const Eigen::Vector3f &odom );
	
	// ---------------------- END ---------------------------//
	
        inline const int inputFilePointPose()
        {
                return input_file.tellg();
        }

        inline const int endOfInputFile()
        {
                return input_file.eof();
        }

        inline const long getFrameCount() const
        {
                return count;
        }

	// added for the output file

	inline const int endOfOutputFile()
	{
		return out_file.eof();
	}
	// END

private:
	std::ifstream input_file;
	std::ofstream out_file;

        long count;

};

}

}

#endif
