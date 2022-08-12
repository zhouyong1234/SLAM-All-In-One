#include "data_reader.h"
#include <fstream>

namespace MSCKF_MINE
{

DataReader::DataReader()
{
    cout << BOLDCYAN"---Load Imu and Camera data---";
    loadImu();
    loadCamera();
    cout << BOLDCYAN"---Load Imu and Camera data Suscessfully!---";
    cout << BOLDWHITE << endl;
}

DataReader::~DataReader()
{

}

void DataReader::loadImu()
{
    string imu_path = Config::get<string>("imu_path");
    ifstream inFile;
    inFile.open(imu_path.c_str());
    while (inFile.good())
    {
        IMU imu;
        inFile >> imu.time_stamp >> imu.wx >> imu.wy >> imu.wz >> imu.ax >> imu.ay >> imu.az;
        mvImuData.push_back(imu);
        inFile.get();
    }
    mvImuData.pop_back();
}

void DataReader::loadCamera()
{
    string cam_path = Config::get<string>("camera_path");
    ifstream inFile;
    inFile.open(cam_path.c_str());
    while (inFile.good())
    {
        CAMERA cam;
        inFile >> cam.time_stamp >> cam.img_name;
        cam.img_name = Config::get<string>("sequence_dir") + "cam0/data/" + cam.img_name;
        mvCameraData.push_back(cam);
        inFile.get();
    }
    mvCameraData.pop_back();
}





}
