#include <iostream>
#include <opencv2/viz.hpp>
using namespace std;
using namespace cv;

int main(int argc,char** argv) {
    viz::Viz3d window1("window1");
    window1.spin();
    cout << "First Window over!" << endl;
    viz::Viz3d window2 = viz::getWindowByName("window1");
    window2.spin();
    cout << "Second window over!" << endl;
    viz::Viz3d window3("window3");
    //下面这种操作适合固定频率刷屏
    window3.spinOnce(1,true);
    while(!window3.wasStopped()) {
        window3.spinOnce(1,true);   
    }
    cout << "spin once window over!" << endl;

    return 0;
}