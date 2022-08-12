#ifndef IMU_DATA_STRUCTURE_H
#define IMU_DATA_STRUCTURE_H

#include <iostream>
#include <vector>

using namespace std;


class Sensor_data
{
public:
    Sensor_data();
    Sensor_data(vector<double> value):
        timestamp(value[0]), x_val(value[1]), y_val(value[2]), z_val(value[3])
    {}
public:
    double timestamp;
    double x_val;
    double y_val;
    double z_val;

};

class IMU_Data
{
public:
    void setImuData_Acc(vector<Sensor_data> &acc ) {Acc = acc; }
    void setImuData_Gyro(vector<Sensor_data> &gyro) {Gyro = gyro;}

public:
    vector<Sensor_data> Acc;
    vector<Sensor_data> Gyro;

};



#endif // IMU_DATA_STRUCTURE_H
