#include <iostream>
#include <algorithm>
#include <dirent.h>
#include <thread>
#include "OpticalSLAM.h"
#include "params.h"

using namespace std;

const string path = "/home/touchair/下载/dataset/data_odometry_gray/00/";

bool getImages(vector<string>& files)
{
	
	string imgPath = path+"image_0/";
    DIR *dp;
    struct dirent *dirp;
    if ((dp = opendir(imgPath.c_str())) == NULL) {
        cout<<"failed to get the images!"<<endl;
        return false;;
    }

    while ((dirp = readdir(dp)) != NULL) {
        std::string name = std::string(dirp->d_name);
        //cout<<name<<endl;
        if (name.substr(0,1) != "." && name != ".." && name.substr(name.size() - 3, name.size()) == "png")
            files.push_back(name);
    }
    closedir(dp);
    std::sort(files.begin(), files.end());
    cout<<"Got the images."<<endl;
    return true;
}
int main()
{
	bool is_first_img = true;
	vector<string> sImgs;
	if(!getImages(sImgs))
	{
		cout<<"Failed to get the images!"<<endl;
		return -1;
	}

	setParams();
	OpticalSLAM slam_system;
	slam_system.initialize();
	// Mat img_to_display;
	std::thread slam_thread([&](){
		for(int i = 0; i < sImgs.size(); ++i)
		{
			string strLeftImg = path + "image_0/" + sImgs[i];
			string strRightImg = path + "image_1/" + sImgs[i];

			Mat leftImg = imread(strLeftImg, 0);
			Mat rightImg = imread(strRightImg, 0);
			// img_to_display = leftImg.clone();
			// imshow("display", img_to_display);
			if(is_first_img)
			{
				slam_system.setFirstFrame(leftImg, rightImg);
				is_first_img = false;
			}
			else
				slam_system.trackStereoFrames(leftImg, rightImg);
			if(i >= WINDOW_SIZE)
				std::this_thread::sleep_for(chrono::milliseconds(200)); 
		}

	});

	slam_system.display();

	slam_thread.join();
	return 0;
}