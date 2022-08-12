#include <iostream>
#include <opencv2/viz.hpp>
using namespace std;
using namespace cv;
int main() {
    viz::Viz3d window("show widget");
    window.showWidget("widget coordinate",viz::WCoordinateSystem());
    viz::WLine line(Point3d(-1,-1,-1),Point3d(1,1,1),viz::Color::bluberry());
    line.setRenderingProperty(viz::LINE_WIDTH,3);
    window.showWidget("widget line",line);
    viz::WCube cube(Point3f(-0.2,-0.2,-0.2),Point3f(0.2,0.2,0.2),false,viz::Color::green());
    window.showWidget("widget cube",cube);
    Vec3f rot_vec(0,0,1);
    Vec3f trans_vec(1,0,0);
    float theta = 0;
    float r = 2;
  
    while (!window.wasStopped())
    {   
        theta += CV_PI * 0.01;
        rot_vec[2] = theta;
        trans_vec[0] = r * sin(theta);
        trans_vec[1] = r * cos(theta);
        Affine3f pose(rot_vec,trans_vec);
        cube.setPose(pose);
        window.spinOnce(1,true);
    }
    return 0;
}