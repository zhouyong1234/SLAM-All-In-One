#include "common_include.h"
#include "config.h"

#define TEST_CSV_READER

using namespace MSCKF_MINE;

int main(int argc, char *argv[])
{
    /*test the config file*/
    Config::setParameterFile("../config/config.yaml");
    string sequence_dir = Config::get<string>("sequence_dir");
    string imu_path = sequence_dir + "imu0/data.csv";
    string camera_path = sequence_dir + "cam0/data.csv";
    FileStorage fs("../config/config.yaml", FileStorage::READ);
    //Mat K;
    string K = fs["T_BS"];

    cout << "K = \n" << K << endl;
    cout << "sequence_dir = " << sequence_dir << endl;
    cout << "imu_path = " << imu_path << endl;


    return 0;
}

