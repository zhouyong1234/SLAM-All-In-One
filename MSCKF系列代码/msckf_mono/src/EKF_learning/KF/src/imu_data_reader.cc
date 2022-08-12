#include "imu_data_reader.h"

using namespace std;

IMU_Reader::IMU_Reader()
{
    readAcc();
    readGyro();
}

void IMU_Reader::readAcc()
{
    ifstream inFile("../IMU_data/circul/circul_lacm.csv");
    string tmp,lineString;
    getline(inFile,tmp);
    getline(inFile,tmp);

    vector<Sensor_data> acc;

    while (getline(inFile,lineString))
    {

        stringstream ss(lineString);
        string str;
        vector<double> acc_value;

        while (getline(ss, str, ','))
        {
            stringstream ss1;
            ss1 << str;
            double val;
            ss1 >> val;
            acc_value.push_back(val);

        }

        Sensor_data sensor_acc(acc_value);
        acc.push_back(sensor_acc);
    }

    this->data.setImuData_Acc(acc);

}

void IMU_Reader::readGyro()
{
    ifstream inFile("../IMU_data/circul/circul_gyrm.csv");
    string tmp,lineString;
    getline(inFile,tmp);
    getline(inFile,tmp);

    vector<Sensor_data> gyro;

    while (getline(inFile,lineString))
    {

        stringstream ss(lineString);
        string str;
        vector<double> gyro_value;

        while (getline(ss, str, ','))
        {
            stringstream ss1;
            ss1 << str;
            double val;
            ss1 >> val;
            gyro_value.push_back(val);

        }

        Sensor_data sensor_gyro(gyro_value);
        gyro.push_back(sensor_gyro);

    }

    this->data.setImuData_Gyro(gyro);
}
