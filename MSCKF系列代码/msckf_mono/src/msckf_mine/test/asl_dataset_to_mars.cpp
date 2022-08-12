#include "common_include.h"
#include "config.h"
#include "data_reader.h"
#include <sstream>
#include <fstream>
#include <cstdio>

using namespace MSCKF_MINE;

int flag = 0;


int main(int argc, char *argv[])
{
    Config::setParameterFile("../config/config.yaml");
    DataReader data;
    vector<CAMERA> camera = data.mvCameraData;
    vector<IMU> imu = data.mvImuData;
    ofstream out_File, imu_File;
    out_File.open("/home/m/DATASET/MAV0/img_data/right/timestamps.txt");
    imu_File.open("/home/m/DATASET/MAV0/imu_data.txt");
    int count = 0;

if(flag == 0)
{

    for(vector<CAMERA>::iterator cam_iter = camera.begin(); cam_iter!=camera.end(); cam_iter++)
    {
        long long timestamp = cam_iter->time_stamp;
        stringstream ss;
        string time;
        ss << timestamp;
        ss >> time;
        string dot = ".";
        time.insert(10,dot);

        /*for image name*/
        char image_name[10];
        sprintf(image_name, "%07d", count);
        string name = "m" + string(image_name) + ".pgm";
        cout << "image_name = " << name << endl;

        out_File << name << " " <<  time << endl;

        Mat image_png = imread(cam_iter->img_name, CV_LOAD_IMAGE_GRAYSCALE);
        Mat out = image_png(Range(0,480),Range(0,640));
        vector<int> params;
        params.push_back(1);

        imwrite("/home/m/DATASET/MAV0/img_data/right/" + name, out, params);
        count++;

    }
}

/*modified the imu*/
else
{
    for(int i = 0; i < imu.size(); i++)
    {
        long long timestamp = imu[i].time_stamp;
        stringstream ss;
        string time;
        ss << timestamp;
        ss >> time;
        string dot = ".";
        time.insert(10,dot);
        imu_File << time << " " << imu[i].wx << " " << imu[i].wy << " " << imu[i].wz << " " << imu[i].ax << " " << imu[i].ay << " " << imu[i].az << endl;

    }
}





    return 0;
}
