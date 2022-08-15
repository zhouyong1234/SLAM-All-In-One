//
// Created by JinTian on 18/03/2018.
//

#include <iostream>
#include <opencv2/imgcodecs.hpp>
#include <opencv/cv.hpp>
#include <dirent.h>
#include "visual_odometry.h"
// #include "thor/os.h"
#include "thor/os.h"
#include "vector"
#include <time.h>
#include <ctime>
#include <unistd.h>

using namespace std;

void getAllImages(const string &imgPath, vector<string> &allImages) {
    // simply range from 000000 ~ 004540, for continue of sequences
    if (thor::os::exists(imgPath)) {
        for (int i = 1; i < 445; ++i) {
            char img[256];
            sprintf(img, "%d.png", i);
            allImages.push_back(thor::os::join(imgPath, img));
        }
    } else {
        cout << "path: " << imgPath << " not exist.\n";
    }
}

int main(int argc, char *argv[]) {

    string sequenceDir = argv[1];

    vector<string> allImages;
    getAllImages(sequenceDir, allImages);

    for (int j=0; j<allImages.size();j++) {
        if (thor::os::suffix(allImages[j]) != "png") {
            allImages.erase(allImages.begin() + j);
        }
    }

    // for (int k = 0; k < 30; ++k) {
    //     cout << allImages[k] << endl;
    // }



    // auto *camera = new MonoCamera(1241, 376,
    //                                    718.8560, 718.8560,
    //                                     607.1928, 185.2157);
     auto *camera = new MonoCamera(640, 480,
                                  484.0910811928012, 484.01576020265475,
                                  321.6130540995404, 238.71540961023783,
                                  0.03966100417888477, -0.05079535254969738,
                                  -0.0008809115033325634, 0.0014337994776532444, 0);
    VisualOdometry vo = VisualOdometry(camera);

    // Setup for show
    char text[100];
    int font_face = cv::FONT_HERSHEY_PLAIN;
    double font_scale = 1;
    int thickness = 1;
    cv::Point text_org(80, 60);
    // cv::namedWindow("Road facing camera", cv::WINDOW_AUTOSIZE);
    cv::namedWindow("Trajectory", cv::WINDOW_AUTOSIZE);
    cv::Mat trajectory = cv::Mat::zeros(600, 600, CV_8UC3);

    double x=0.0, y=0.0,z=0.0;
    // iterate all images and get frameID
    for(int i=0; i<allImages.size(); i++) {

        cv::Mat img(cv::imread(allImages[i].c_str(), 0));
        assert(!img.empty());

        vo.addImage(img, i);
        cv::Mat curT = vo.getCurrentT();
        if (curT.rows != 0) {
            x = curT.at<double>(0);
            y = curT.at<double>(1);
            z = curT.at<double>(2);
        }


        // std::cout << "x: " << x << " y: " << y << " z: " << z << std::endl;
        std::cout << "x: " << x << std::endl;

        int drawX = int(x) + 300;
        int drawY = int(z) + 100;

        cv::circle(trajectory, cv::Point(drawX, 100), 1, CV_RGB(255, 0, 0), 5);
        cv::rectangle(trajectory, cv::Point(10, 30), cv::Point(580, 60), CV_RGB(0, 0, 0), CV_FILLED);
        // sprintf(text, "Coordinates: x = %02fm y = %02fm z = %02fm", x, y, z);
        sprintf(text,  "Coordinates: x = %02fm", x);

        cv::putText(trajectory, text, text_org, font_face, 2, cv::Scalar::all(255), 2, 8);
        // cv::imshow("Road facing camera", img);
        cv::imshow("Trajectory", trajectory);
        cv::waitKey(10);
        // usleep(50000);

    }

    delete camera;
    return 0;
}