#ifndef IMU_DATA_READER_H
#define IMU_DATA_READER_H

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include "imu_data_structure.h"

class IMU_Reader
{
public:
    IMU_Reader();
    void readAcc();
    void readGyro();
public:
    IMU_Data data;

};


#endif // IMU_DATA_READER_H
