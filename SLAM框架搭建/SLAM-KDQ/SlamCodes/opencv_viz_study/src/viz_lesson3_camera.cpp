
#include <opencv2/viz.hpp>
#include <iostream>
#include <fstream>
using namespace cv;
using namespace std;
const string bunnyfile = "/home/zx/workspace/SLAM-KDQ/SlamCodes/opencv_viz_study/sample/bunny.ply";

static void help()
{
    cout
    << "--------------------------------------------------------------------------"   << endl
    << "This program shows how to use makeTransformToGlobal() to compute required pose,"
    << "how to use makeCameraPose and Viz3d::setViewerPose. You can observe the scene "
    << "from camera point of view (C) or global point of view (G)"                    << endl
    << "Usage:"                                                                       << endl
    << "./transformations [ G | C ]"                                                 << endl
    << endl;
}
static Mat cvcloud_load()
{
    Mat cloud(1, 1889, CV_32FC3);
    ifstream ifs(bunnyfile);
    string str;
    for(size_t i = 0; i < 12; ++i)
        getline(ifs, str);
    Point3f* data = cloud.ptr<cv::Point3f>();
    float dummy1, dummy2;
    for(size_t i = 0; i < 1889; ++i)
        ifs >> data[i].x >> data[i].y >> data[i].z >> dummy1 >> dummy2;
    cloud *= 5.0f;
    return cloud;
}
int main(int argn, char **argv)
{
    help();
    if (argn < 2)
    {
        cout << "Missing arguments." << endl;
        return 1;
    }
    bool camera_pov = (argv[1][0] == 'C');
    viz::Viz3d myWindow("Coordinate Frame");
    myWindow.showWidget("Coordinate Widget", viz::WCoordinateSystem());
    //KDQ:指定camera的位置和焦点位置，这样其实等于确定了camera的z轴（camera位置指向焦点即为z轴方向）,如果y轴不合z轴垂直，程序会强制垂直，甚至会出现无法显示camera框架的问题
    Vec3f cam_pos(3.0f,0.0f,0.0f), cam_focal_point(0.0f,0.0f,0.0f), cam_y_dir(0.0f,1.0f,0.0f);
    Affine3f cam_pose = viz::makeCameraPose(cam_pos, cam_focal_point, cam_y_dir);
    //KDQ:求camera坐标系相当于全局坐标系的变换关系Twc，其实和cam_pose值一样
    Affine3f transform = viz::makeTransformToGlobal(Vec3f(0.0f,0.0f,1.0f), Vec3f(0.0f,1.0f,0.0f), Vec3f(-1.0f,0.0f,0.0f), cam_pos);
    Mat bunny_cloud = cvcloud_load();
    viz::WCloud cloud_widget(bunny_cloud, viz::Color::green());
    //KDQ:这个制定cloud点云在相机坐标系下的位置tcp
    Affine3f cloud_pose = Affine3f().translate(Vec3f(0.0f,0.0f,5.0f));
    //KDQ:求cloud到全局坐标系的变换关系Twp
    Affine3f cloud_pose_global = transform * cloud_pose;
    if (!camera_pov)
    {
        //KDQ:可以直接查这个类，参数是一个的设置camera坐标系的尺度
        viz::WCameraPosition cpw(1); // Coordinate axes
        //KDQ:这个可以调节显示的camera框，锥截面的长宽，以及显示尺度
        viz::WCameraPosition cpw_frustum(Vec2f(0.8, 0.5),1.0); // Camera frustum
        myWindow.showWidget("CPW", cpw, cam_pose);
        myWindow.showWidget("CPW_FRUSTUM", cpw_frustum, cam_pose);
    }
    myWindow.showWidget("bunny", cloud_widget, cloud_pose_global);
    if (camera_pov)
        myWindow.setViewerPose(cam_pose);
    myWindow.spin();
    return 0;
}