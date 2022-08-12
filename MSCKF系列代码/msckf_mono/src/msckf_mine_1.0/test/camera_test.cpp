#include "common_include.h"
#include "config.h"
#include "camera.h"
#include "converter.h"

#define TEST_CSV_READER

using namespace MSCKF_MINE;

int main(int argc, char *argv[])
{
    /*test the config file*/
    Config::setParameterFile("../config/config.yaml");
    Camera camera;
    cout << "Camera.K = \n" << camera.getK() << endl;

    cout << "TBS = \n" << camera.getTBS() << endl;

    Eigen::Matrix<double, 4, 4> T_BS = Converter::toMatrix4d(camera.getTBS());

    cout << "T_BS = \n" << T_BS(0,0) << endl;


    return 0;
}

