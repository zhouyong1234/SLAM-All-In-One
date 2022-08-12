#ifndef DATA_READER_H
#define DATA_READER_H

#include "common_include.h"
#include "config.h"
#include "types.h"
using namespace std;


namespace MSCKF_MINE
{
class DataReader
{
public:
    DataReader();
    ~DataReader();
    void loadImu();
    void loadCamera();

public:
    vector<IMU> mvImuData;
    vector<CAMERA> mvCameraData;


};

}

#endif // DATA_READER_H
