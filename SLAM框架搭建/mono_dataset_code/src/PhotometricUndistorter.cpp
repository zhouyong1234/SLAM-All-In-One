/* Copyright (c) 2016, Jakob Engel
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without 
 * modification, are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, 
 * this list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form must reproduce the above copyright notice, 
 * this list of conditions and the following disclaimer in the documentation and/or 
 * other materials provided with the distribution.
 * 
 * 3. Neither the name of the copyright holder nor the names of its contributors 
 * may be used to endorse or promote products derived from this software without 
 * specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. 
 * IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, 
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, 
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
 * POSSIBILITY OF SUCH DAMAGE.
 */


#include <sstream>
#include <fstream>
#include <iterator>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include "PhotometricUndistorter.h"


PhotometricUndistorter::PhotometricUndistorter(
		std::string file,
		std::string vignetteImage,
		int w_, int h_)
{

	validVignette=false;
	validGamma=false;

	vignetteMap=0;
	vignetteMapInv=0;
	w = w_;
	h = h_;

	if(file=="" || vignetteImage=="") return;


	// read G.
	std::ifstream f(file.c_str());
	printf("Reading Photometric Calibration from file %s\n",file.c_str());
	if (!f.good())
	{
		printf("PhotometricUndistorter: Could not open file!\n");
		return;
	}



	std::string line;
	std::getline( f, line );
	std::istringstream l1i( line );
	std::vector<float> GInvvec = std::vector<float>( std::istream_iterator<float>(l1i), std::istream_iterator<float>() );
	if(GInvvec.size() != 256)
	{
		printf("PhotometricUndistorter: invalid format! got %d entries in first line, expected 256!\n",(int)GInvvec.size());
		return;
	}
	for(int i=0;i<256;i++) GInv[i] = GInvvec[i];

	for(int i=0;i<255;i++)
	{
		if(GInv[i+1] <= GInv[i])
		{
			printf("PhotometricUndistorter: G invalid! it has to be strictly increasing, but it isnt!\n");
			return;
		}
	}
	float min=GInv[0];
	float max=GInv[255];
	for(int i=0;i<256;i++) GInv[i] = 255.0 * (GInv[i] - min) / (max-min);			// make it to 0..255 => 0..255.

	// invert Gamma (maybe someone needs this).
	for(int i=1;i<255;i++)
	{
		// find val, such that Binv[val] = i.
		// I dont care about speed for this, so do it the stupid way.
		for(int s=1;s<255;s++)
		{
			if(GInv[s] <= i && GInv[s+1] >= i)
			{
				G[i] = s+(i - GInv[s]) / (GInv[s+1]-GInv[s]);
				break;
			}
		}
	}
	G[0] = 0;
	G[255] = 255;
	f.close();
	validGamma=true;








	printf("Reading Vignette Image from %s\n",vignetteImage.c_str());
	cv::Mat vignetteMat = cv::imread(vignetteImage.c_str(), CV_LOAD_IMAGE_UNCHANGED);
	vignetteMap = new float[w*h];
	vignetteMapInv = new float[w*h];
	if(vignetteMat.rows != h || vignetteMat.cols != w)
	{
		printf("PhotometricUndistorter: Invalid vignette image size! got %d x %d, expected %d x %d. Set vignette to 1.\n",
				vignetteMat.cols, vignetteMat.rows, w, h);
		return;
	}

	if(vignetteMat.type() == CV_8U)
	{
		float maxV=0;
		for(int i=0;i<w*h;i++)
			if(vignetteMat.at<unsigned char>(i) > maxV) maxV = vignetteMat.at<unsigned char>(i);

		for(int i=0;i<w*h;i++)
			vignetteMap[i] = vignetteMat.at<unsigned char>(i) / maxV;
	}
	else if(vignetteMat.type() == CV_16U)
	{
		float maxV=0;
		for(int i=0;i<w*h;i++)
			if(vignetteMat.at<ushort>(i) > maxV) maxV = vignetteMat.at<ushort>(i);

		for(int i=0;i<w*h;i++)
			vignetteMap[i] = vignetteMat.at<ushort>(i) / maxV;
	}
	else
		assert(false);

	for(int i=0;i<w*h;i++)
		vignetteMapInv[i] = 1.0f / vignetteMap[i];


	printf("Successfully read photometric calibration!\n");
	validVignette = true;
}
PhotometricUndistorter::~PhotometricUndistorter()
{
	if(vignetteMap != 0) delete[] vignetteMap;
	if(vignetteMapInv != 0) delete[] vignetteMapInv;
}


void PhotometricUndistorter::unMapImage(
		unsigned char* image_in,
		float* image_out,
		int n,
		bool undoGamma,
		bool undoVignette,
		bool killOverexposed)
{
	if(!validGamma && undoGamma)
	{
		printf("Photometric Undistorter did not load Gamma correctly. correctly. Not undoing gamma!\n");
		undoGamma = false;
	}
	if(!validVignette && undoVignette)
	{
		printf("Photometric Undistorter did not load Vignette correctly. correctly. Not undoing Vignette!\n");
		undoVignette = false;
	}

	if(!undoGamma && undoVignette)
	{
		printf("it doesn't make sense to undo vignette without undoing gamma! not doing neither.\n");
		undoVignette=false;
		undoGamma=false;
	}

	assert(n == w*h);

	if(!undoGamma && !undoVignette)
	{
		for(int i=0;i<n;i++) image_out[i] = image_in[i];
	}

	if(undoGamma && !undoVignette)
	{
		for(int i=0;i<n;i++) image_out[i] = GInv[image_in[i]];
	}

	if(undoGamma && undoVignette)
	{
		for(int i=0;i<n;i++) image_out[i] = GInv[image_in[i]] * vignetteMapInv[i];
	}

	if(killOverexposed)
	{
		for(int i=0;i<n;i++) if(image_in[i]==255) image_out[i]=NAN;
	}
}



