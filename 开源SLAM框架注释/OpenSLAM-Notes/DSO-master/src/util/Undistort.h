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

#include "util/ImageAndExposure.h"
#include "util/MinimalImage.h"
#include "util/NumType.h"
#include "Eigen/Core"





namespace dso
{


class PhotometricUndistorter
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
	PhotometricUndistorter(std::string file, std::string noiseImage, std::string vignetteImage, int w_, int h_);
	~PhotometricUndistorter();

	// removes readout noise, and converts to irradiance.
	// affine normalizes values to 0 <= I < 256.
	// raw irradiance = a*I + b.
	// output will be written in [output].
	template<typename T> void processFrame(T* image_in, float exposure_time, float factor=1);
	void unMapFloatImage(float* image);

	ImageAndExposure* output; //!< 光度矫正后的图像

	float* getG() {if(!valid) return 0; else return G;};
private:
    float G[256*256]; 	//!< 响应函数值逆变换
    int GDepth;			//!< 响应函数值的个数 
	float* vignetteMap;
	float* vignetteMapInv;
	int w,h;
	bool valid;
};


class Undistort
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
	virtual ~Undistort();

	virtual void distortCoordinates(float* in_x, float* in_y, float* out_x, float* out_y, int n) const = 0;

	
	inline const Mat33 getK() const {return K;};
	inline const Eigen::Vector2i getSize() const {return Eigen::Vector2i(w,h);};
	inline const VecX getOriginalParameter() const {return parsOrg;};
	inline const Eigen::Vector2i getOriginalSize() {return Eigen::Vector2i(wOrg,hOrg);};
	inline bool isValid() {return valid;};

	template<typename T>
	ImageAndExposure* undistort(const MinimalImage<T>* image_raw, float exposure=0, double timestamp=0, float factor=1) const;
	static Undistort* getUndistorterForFile(std::string configFilename, std::string gammaFilename, std::string vignetteFilename);

	void loadPhotometricCalibration(std::string file, std::string noiseImage, std::string vignetteImage);

	PhotometricUndistorter* photometricUndist; // 光度矫正类

protected:
    int w, h, wOrg, hOrg, wUp, hUp;	//!< 输入图像大小, 相机原像素大小, 
    int upsampleUndistFactor;
	Mat33 K;						//!< 矫正后的相机参数(也可能是更改了的标定输出)
	VecX parsOrg;					//!< 原来相机参数
	bool valid;						//!< 参数有效
	bool passthrough;				//!< 通过??? 不知道这个是干嘛的

	float* remapX;					//!< 矫正所用的remap, 无畸变与畸变的映射
	float* remapY;

	void applyBlurNoise(float* img) const;

	void makeOptimalK_crop();
	void makeOptimalK_full();

	void readFromFile(const char* configFileName, int nPars, std::string prefix = "");
};

class UndistortFOV : public Undistort
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    UndistortFOV(const char* configFileName, bool noprefix);
	~UndistortFOV();
	void distortCoordinates(float* in_x, float* in_y, float* out_x, float* out_y, int n) const;

};

class UndistortRadTan : public Undistort
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    UndistortRadTan(const char* configFileName, bool noprefix);
    ~UndistortRadTan();
    void distortCoordinates(float* in_x, float* in_y, float* out_x, float* out_y, int n) const;

};

class UndistortEquidistant : public Undistort
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    UndistortEquidistant(const char* configFileName, bool noprefix);
    ~UndistortEquidistant();
    void distortCoordinates(float* in_x, float* in_y, float* out_x, float* out_y, int n) const;

};

class UndistortPinhole : public Undistort
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    UndistortPinhole(const char* configFileName, bool noprefix);
	~UndistortPinhole();
	void distortCoordinates(float* in_x, float* in_y, float* out_x, float* out_y, int n) const;

private:
	float inputCalibration[8];
};

class UndistortKB : public Undistort
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    UndistortKB(const char* configFileName, bool noprefix);
	~UndistortKB();
	void distortCoordinates(float* in_x, float* in_y, float* out_x, float* out_y, int n) const;

};

}

