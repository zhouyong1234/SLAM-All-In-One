#include "readScanContext.h"

namespace localization
{
    
ReadScanContext::ReadScanContext(const std::string &filename)
{
	readScanContext( filename );
}

ReadScanContext::~ReadScanContext()
{

}

void ReadScanContext::readScanContext(const std::string &filename)
{
        std::fstream stream;
        stream.open(filename, std::ios::in);
        if (!stream.is_open()){
		exit(-1);
	}
        
	std::string line;
        int keyPoses_size, ringKeys_size, sectorKeys_size, scanContexts_size;
            
	while (std::getline(stream, line)){
                std::vector<std::string> strs;
                std::stringstream ss(line);
                std::string str;
                std::istringstream record(line);
                
		while (record >> str){
                    strs.push_back(str);
                }

                int strs_size = strs.size();
                if (strs[0].compare("keyPoses") == 0) {
                    	keyPoses_size = std::stoi(strs[1]);
                    	Eigen::Vector3f keyPose;
                    	keyPose << std::stof(strs[2]), std::stof(strs[3]), std::stof(strs[4]);
                    	keyPoses.push_back(keyPose);
                }
                else if (strs[0].compare("RingKeys") == 0){
                    	ringKeys_size = std::stoi(strs[1]);
                    	assert(ringKeys_size == strs_size - 2);
                    
			Eigen::MatrixXf ringKeys_temp(ringKeys_size, 1);
                    	for (int i = 0; i < ringKeys_size; i++){
                        	ringKeys_temp(i, 0) = std::stof(strs[i + 2]);
                    	}
                    	ringKeys.push_back(ringKeys_temp);
               	}
                else if (strs[0].compare("SectorKeys") == 0){
                    	sectorKeys_size = std::stoi(strs[1]);
                    	// 断言
                    	assert(sectorKeys_size == strs_size - 2);
                
		    	Eigen::MatrixXf sectorKeys_temp(1, sectorKeys_size);
                    	for (int i = 0; i < sectorKeys_size; i++){
                        	sectorKeys_temp(0, i) = std::stof(strs[i + 2]);
                  	}
                    	sectorKeys.push_back(sectorKeys_temp);
                }
                else if (strs[0].compare("ScanContexts") == 0){
                    	// std::cout << strs[0] << " " << strs.size() << std::endl;
                    	scanContexts_size = std::stoi(strs[1]);
                    	assert(scanContexts_size == strs_size - 2);
                    	Eigen::MatrixXf scanContexts_temp(ringKeys_size, sectorKeys_size);
                    	
			for (int i = 0; i < ringKeys_size; i++){
                        	for (int j = 0; j < sectorKeys_size; j++){
                            		scanContexts_temp(i, j) = std::stof(strs[i * sectorKeys_size + j + 2]);
                        	}
                    	}
                    	// std::cout << scanContexts_temp << std::endl;
                    	scanContexts.push_back(scanContexts_temp);
                }
                strs.clear();
	}
        assert(keyPoses.size() == keyPoses_size);
        stream.close();
}

void ReadScanContext::printScanContext() const
{
        std::cout << "keyPoses:" << std::endl;
        for (int i = 0; i < keyPoses.size(); i++){
            std::cout << keyPoses[i] << std::endl;
        }

        std::cout << "ringKeys:" << std::endl;
        for (int i = 0; i < ringKeys.size(); i++){
            std::cout << ringKeys[i] << std::endl;
        }

        std::cout << "sectorKeys:" << std::endl;
        for (int i = 0; i < sectorKeys.size(); i++){
            std::cout << sectorKeys[i] << std::endl;
        }

        std::cout << "scanContexts:" << std::endl;
        for (int i = 0; i < scanContexts.size(); i++){
            std::cout << scanContexts[i] << std::endl;
        }
}


}
