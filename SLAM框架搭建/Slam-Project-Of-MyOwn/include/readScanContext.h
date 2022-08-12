#include<fstream>
#include<Eigen/Dense>

#include<vector>
#include <iostream>

#include <stdlib.h>

#include <cassert>
namespace localization{

class ReadScanContext
{
public:
	ReadScanContext( const std::string &filename);
    	~ReadScanContext();
	
    	void printScanContext() const;
private:
    	void readScanContext(const std::string &filename);

public:
    	std::vector<Eigen::Vector3f> keyPoses;
    	std::vector<Eigen::MatrixXf> ringKeys;
    	std::vector<Eigen::MatrixXf> sectorKeys;
    	std::vector<Eigen::MatrixXf> scanContexts;
};
    
}
