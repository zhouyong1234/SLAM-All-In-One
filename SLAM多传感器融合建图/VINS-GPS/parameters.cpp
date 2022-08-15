//
// Created by grn on 11/2/18.
//

#include "parameters.h"

std::string IMAGE_file;
std::string IMU_file;
std::string IMAGE_data;
std::string GPS_file;
std::vector<std::string> CAM_NAMES;
std::string FISHEYE_MASK;
//GPS
int GPS_HZ;
int is_out;
double out_start;
double out_end;
double GPS_L0,GPS_L1,GPS_L2;

int MAX_CNT;
int MIN_DIST;
int FREQ;
double F_THRESHOLD;
int SHOW_TRACK;
int STEREO_TRACK;
int EQUALIZE;
int FISHEYE;
bool PUB_THIS_FRAME;

double FOCAL_LENGTH;
double INIT_DEPTH;
double MIN_PARALLAX;
double ACC_N, ACC_W;
double GYR_N, GYR_W;

std::vector<Eigen::Matrix3d> RIC;
std::vector<Eigen::Vector3d> TIC;

Eigen::Vector3d G{0.0, 0.0, 9.8};


double BIAS_ACC_THRESHOLD;
double BIAS_GYR_THRESHOLD;
double SOLVER_TIME;
int NUM_ITERATIONS;
int ESTIMATE_EXTRINSIC;
int ESTIMATE_TD;
int ROLLING_SHUTTER;
std::string EX_CALIB_RESULT_PATH;
std::string VINS_RESULT_PATH;
int ROW, COL;
double TD, TR;

void readParameters(std::string configfile)
{
    cv::FileStorage fsSettings(configfile, cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        std::cerr << "ERROR: Wrong path to settings" << std::endl;
    }

    FOCAL_LENGTH=fsSettings["focal_length"];
    GPS_HZ=fsSettings["gps_hz"];
    GPS_L0=fsSettings["gps_l0"];
    GPS_L1=fsSettings["gps_l1"];
    GPS_L2=fsSettings["gps_l2"];
    is_out=fsSettings["is_out"];
    out_start=fsSettings["out_start"];
    out_end=fsSettings["out_end"];
    if(is_out)
    {
        printf("*****GPS-OUT******start：%.3f******end：%.3f \n",out_start,out_end);
    }
    else
    {
        printf("+++++++++++++++NO--GPS--OUT+++++++++++++++ \n");
    }

    fsSettings["image_file"] >> IMAGE_file;
    fsSettings["imu_file"] >> IMU_file;
    fsSettings["image_path"] >> IMAGE_data;
    fsSettings["gps_file"] >> GPS_file;
    MAX_CNT = fsSettings["max_cnt"];
    MIN_DIST = fsSettings["min_dist"];
    ROW = fsSettings["image_height"];
    COL = fsSettings["image_width"];
    FREQ = fsSettings["freq"];
    F_THRESHOLD = fsSettings["F_threshold"];
    SHOW_TRACK = fsSettings["show_track"];
    EQUALIZE = fsSettings["equalize"];
    FISHEYE = fsSettings["fisheye"];
    if (FISHEYE == 1)
        FISHEYE_MASK = "config/fisheye_mask.jpg";
    CAM_NAMES.push_back(configfile);

    STEREO_TRACK = false;
    PUB_THIS_FRAME = false;

    if (FREQ == 0)
        FREQ = 100;


    //estimator

    SOLVER_TIME = fsSettings["max_solver_time"];
    NUM_ITERATIONS = fsSettings["max_num_iterations"];
    MIN_PARALLAX = fsSettings["keyframe_parallax"];
    MIN_PARALLAX = MIN_PARALLAX / FOCAL_LENGTH;

    std::string OUTPUT_PATH;
    fsSettings["output_path"] >> OUTPUT_PATH;
    VINS_RESULT_PATH = OUTPUT_PATH + "vins_result_no_loop.csv";
    std::cout << "result path " << VINS_RESULT_PATH << std::endl;
    std::ofstream fout(VINS_RESULT_PATH, std::ios::out);
    fout.close();

    ACC_N = fsSettings["acc_n"];
    ACC_W = fsSettings["acc_w"];
    GYR_N = fsSettings["gyr_n"];
    GYR_W = fsSettings["gyr_w"];
    G.z() = fsSettings["g_norm"];
    ROW = fsSettings["image_height"];
    COL = fsSettings["image_width"];

    ESTIMATE_EXTRINSIC = fsSettings["estimate_extrinsic"];
    if (ESTIMATE_EXTRINSIC == 2)
    {
        RIC.push_back(Eigen::Matrix3d::Identity());
        TIC.push_back(Eigen::Vector3d::Zero());
        EX_CALIB_RESULT_PATH = OUTPUT_PATH + "/extrinsic_parameter.csv";

    }
    else
    {
        if ( ESTIMATE_EXTRINSIC == 1)
        {
            EX_CALIB_RESULT_PATH = OUTPUT_PATH + "/extrinsic_parameter.csv";
        }
        if (ESTIMATE_EXTRINSIC == 0)
            std::cout<<" fix extrinsic param "<<std::endl;

        cv::Mat cv_R, cv_T;
        fsSettings["extrinsicRotation"] >> cv_R;
        fsSettings["extrinsicTranslation"] >> cv_T;
        cout<<cv_R.size()<<endl;
        cout<<cv_T.size()<<endl;
        Eigen::Matrix3d eigen_R;
        Eigen::Vector3d eigen_T;
        cv::cv2eigen(cv_R, eigen_R);
        cv::cv2eigen(cv_T, eigen_T);
        cout<<"yes"<<endl;
        Eigen::Quaterniond Q(eigen_R);
        eigen_R = Q.normalized();
        RIC.push_back(eigen_R);
        TIC.push_back(eigen_T);

    }

    INIT_DEPTH = 5.0;
    BIAS_ACC_THRESHOLD = 0.1;
    BIAS_GYR_THRESHOLD = 0.1;

    TD = fsSettings["td"];
    ESTIMATE_TD = fsSettings["estimate_td"];
    if (ESTIMATE_TD)
        cout<<"Unsynchronized sensors, online estimate time offset, initial td: " << TD<<endl;
    else
        cout<<"Synchronized sensors, fix time offset: " << TD<<endl;

    ROLLING_SHUTTER = fsSettings["rolling_shutter"];
    if (ROLLING_SHUTTER)
    {
        TR = fsSettings["rolling_shutter_tr"];
        cout<<"rolling shutter camera, read out time per line: " << TR<<endl;
    }
    else
    {
        TR = 0;
    }


    fsSettings.release();


}