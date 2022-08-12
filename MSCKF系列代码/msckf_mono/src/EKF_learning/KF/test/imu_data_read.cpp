#include <iostream>
#include "imu_data_reader.h"

using namespace std;

int main(int argc, char *argv[])
{
    IMU_Reader IMU;
    cout << IMU.data.Acc.size() << endl;
    for(int i = 0; i < IMU.data.Acc.size(); i++)
    {
        Sensor_data &data = IMU.data.Acc[i];
        cout << "we get : " << data.timestamp << " " << data.x_val << " " << data.y_val << " " << data.z_val << endl;
    }

    cout << IMU.data.Gyro.size() << endl;

    for(int i = 0; i < IMU.data.Gyro.size(); i++)
    {
        Sensor_data &data = IMU.data.Gyro[i];
        cout << "we get : " << data.timestamp << " " << data.x_val << " " << data.y_val << " " << data.z_val << endl;
    }


    return 0;
}
