/**
* This file is part of DSO.
* 
* Copyright 2016 Technical University of Munich and Intel.
* Developed by Jakob Engel <engelj at in dot tum dot de>,
* for more information see <http://vision.in.tum.de/dso>.
* If you use this code, please cite the respective publications as
* listed on the above website.
*
* DSO is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* DSO is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with DSO. If not, see <http://www.gnu.org/licenses/>.
*/


#pragma once
#include <cstring>
#include <iostream>


namespace dso
{

//* 辐照度值B和曝光时间t
class ImageAndExposure
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
	float* image;			// irradiance. between 0 and 256
	int w,h;				// width and height;
	double timestamp;
	float exposure_time;	// exposure time in ms.
	//! 论文中曝光时间t * 辐照度B
	inline ImageAndExposure(int w_, int h_, double timestamp_=0) : w(w_), h(h_), timestamp(timestamp_)
	{
		image = new float[w*h]; //这表示图像,666, 标定后的辐照度
		exposure_time=1;
	}
	inline ~ImageAndExposure()
	{
		delete[] image;
	}
	// 曝光时间赋值给other
	inline void copyMetaTo(ImageAndExposure &other)
	{
		other.exposure_time = exposure_time;
	}
	// 深度拷贝
	inline ImageAndExposure* getDeepCopy()
	{
		ImageAndExposure* img = new ImageAndExposure(w,h,timestamp);
		img->exposure_time = exposure_time;
		memcpy(img->image, image, w*h*sizeof(float));
		return img;
	}
};


}
