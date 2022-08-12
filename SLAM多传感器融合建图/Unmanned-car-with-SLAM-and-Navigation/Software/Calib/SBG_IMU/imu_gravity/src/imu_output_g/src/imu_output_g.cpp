#include <ros/ros.h>
#include <queue>
#include <sensor_msgs/Imu.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <fstream>
using namespace std;
std::deque<sensor_msgs::Imu> imuQueue;
uint64_t num=0;
uint64_t t_num=0;
double sum=0;
double t_sum=0;
double ave=0;
#define  onenum  1000
#define  twonum  1650

sensor_msgs::Imu imuConverter(const sensor_msgs::Imu& imu_in)
{
    sensor_msgs::Imu imu_out = imu_in;
    // rotate acceleration
//    Eigen::Vector3d acc(imu_in.linear_acceleration.x, imu_in.linear_acceleration.y, imu_in.linear_acceleration.z);
//    acc = extRot * acc;
//    imu_out.linear_acceleration.x = acc.x();
//    imu_out.linear_acceleration.y = acc.y();
//    imu_out.linear_acceleration.z = acc.z();
//    // rotate gyroscope
//    Eigen::Vector3d gyr(imu_in.angular_velocity.x, imu_in.angular_velocity.y, imu_in.angular_velocity.z);
//    gyr = extRot * gyr;
//    imu_out.angular_velocity.x = gyr.x();
//    imu_out.angular_velocity.y = gyr.y();
//    imu_out.angular_velocity.z = gyr.z();
//    // rotate roll pitch yaw
//    Eigen::Quaterniond q_from(imu_in.orientation.w, imu_in.orientation.x, imu_in.orientation.y, imu_in.orientation.z);
//    Eigen::Quaterniond q_final = q_from * extQRPY;
//    imu_out.orientation.x = q_final.x();
//    imu_out.orientation.y = q_final.y();
//    imu_out.orientation.z = q_final.z();
//    imu_out.orientation.w = q_final.w();
//
//    if (sqrt(q_final.x()*q_final.x() + q_final.y()*q_final.y() + q_final.z()*q_final.z() + q_final.w()*q_final.w()) < 0.1)
//    {
//        ROS_ERROR("Invalid quaternion, please use a 9-axis IMU!");
//        ros::shutdown();
//    }

    return imu_out;
}


void imuHandler(const sensor_msgs::Imu::ConstPtr& imuMsg)
{
    double g;
    sensor_msgs::Imu thisImu = imuConverter(*imuMsg);
    imuQueue.push_back(thisImu);
    cout << std::setprecision(6);
    g = thisImu.linear_acceleration.z;
    sum = sum + g;
    num++;
    FILE* file = fopen("/home/qjs/data/imu_calib/imug_txt","a++");
    if(num == onenum)
    {
        cout << "IMU acc z:" << sum/onenum<< endl;
        t_sum = t_sum + sum/onenum;
        num = 0;
        sum = 0;
        t_num++;
        if(t_num == twonum)
        {
            ave = t_sum/twonum;
            fprintf(file,"%f \r\n",ave);
            cout << "IMU acc ave z:" << ave<< endl;
            t_num = 0;
            t_sum = 0;
        }
    }
    fclose(file);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "imu_output_g"); //初始化节点名称
    ros::NodeHandle nh; //声明一个节点句柄
    ros::Rate loop_rate(200);

    ros::Subscriber subimu = nh.subscribe<sensor_msgs::Imu>  ("/imu_raw", 2000, imuHandler);

    ros::spin();
}

