#include "common_include.h"
#include "config.h"
#include "data_reader.h"

using namespace MSCKF_MINE;

int main(int argc, char *argv[])
{
    Config::setParameterFile("../config/config.yaml");
    DataReader data;
    vector<CAMERA> vCameraData = data.mvCameraData;


    /*check the results*/
    cout << "data.mvCameraData.size() = " << data.mvCameraData.size() << endl;
    cout << "data.mvImuData.size() = " << data.mvImuData.size() << endl;

    for(vector<CAMERA>::const_iterator iter = vCameraData.begin(); iter!=vCameraData.end(); iter++)
    {
        CAMERA cam = *iter;
        cout << cam.time_stamp << " " << cam.img_name << endl;
    }

    return 0;
}
