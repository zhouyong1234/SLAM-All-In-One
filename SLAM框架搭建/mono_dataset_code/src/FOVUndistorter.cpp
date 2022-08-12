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


#include "FOVUndistorter.h"

#include <sstream>
#include <fstream>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/opencv.hpp>
#include <Eigen/Core>

UndistorterFOV::UndistorterFOV()
{
	remapX = nullptr;
	remapY = nullptr;
	valid=false;
}

UndistorterFOV::UndistorterFOV(const char* configFileName)
{
	remapX = nullptr;
	remapY = nullptr;
	valid=false;

	// read parameters
	std::ifstream infile(configFileName);
	if(!infile.good())
	{
		printf("Failed to read camera calibration (invalid format?)\nCalibration file: %s\n", configFileName);
		return;
	}


	std::string l1,l2,l3,l4;
	std::getline(infile,l1);
	std::getline(infile,l2);
	std::getline(infile,l3);
	std::getline(infile,l4);


	// l1 & l2
	if(std::sscanf(l1.c_str(), "%f %f %f %f %f", &inputCalibration[0], &inputCalibration[1], &inputCalibration[2], &inputCalibration[3], &inputCalibration[4]) == 5 &&
			std::sscanf(l2.c_str(), "%d %d", &in_width, &in_height) == 2)
	{
		printf("Input resolution: %d %d\n",in_width, in_height);
		printf("Input Calibration (fx fy cx cy): %f %f %f %f %f\n",
				in_width*inputCalibration[0], in_height*inputCalibration[1], in_width*inputCalibration[2], in_height*inputCalibration[3], inputCalibration[4]);
	}
	else
	{
		printf("Failed to read camera calibration (invalid format?)\nCalibration file: %s\n", configFileName);
		return;
	}


	// l3
	if(l3 == "crop")
	{
		outputCalibration[0] = -1;
		printf("Out: Crop\n");
	}
	else if(l3 == "full")
	{
		outputCalibration[0] = -2;
		printf("Out: Full\n");
	}
	else if(l3 == "none")
	{
		printf("NO RECTIFICATION\n");
		return;
	}
	else if(std::sscanf(l3.c_str(), "%f %f %f %f %f", &outputCalibration[0], &outputCalibration[1], &outputCalibration[2], &outputCalibration[3], &outputCalibration[4]) == 5)
	{
		printf("Out: %f %f %f %f %f\n",
				outputCalibration[0], outputCalibration[1], outputCalibration[2], outputCalibration[3], outputCalibration[4]);
	}
	else
	{
		printf("Out: Failed to Read Output pars... not rectifying.\n");
		return;
	}



	// l4
	if(std::sscanf(l4.c_str(), "%d %d", &out_width, &out_height) == 2)
	{
		printf("Output resolution: %d %d\n",out_width, out_height);
	}
	else
	{
		printf("Out: Failed to Read Output resolution... not rectifying.\n");
		return;
	}


	valid=true;


	// =============================== find optimal new camera matrix ===============================
	// prep warp matrices
	float dist = inputCalibration[4];
	float d2t = 2.0f * tan(dist / 2.0f);

	// current camera parameters
	float fx = inputCalibration[0] * in_width;
	float fy = inputCalibration[1] * in_height;
	float cx = inputCalibration[2] * in_width - 0.5;
	float cy = inputCalibration[3] * in_height - 0.5;

	// output camera parameters
	float ofx, ofy, ocx, ocy;

	// find new camera matrix for "crop" and "full"
	if (inputCalibration[4] == 0)
	{
		ofx = inputCalibration[0] * out_width;
		ofy = inputCalibration[1] * out_height;
		ocx = (inputCalibration[2] * out_width) - 0.5;
		ocy = (inputCalibration[3] * out_height) - 0.5;
	}
	else if(outputCalibration[0] == -1)	// "crop"
	{
		// find left-most and right-most radius
		float left_radius = (cx)/fx;
		float right_radius = (in_width-1 - cx)/fx;
		float top_radius = (cy)/fy;
		float bottom_radius = (in_height-1 - cy)/fy;

		float trans_left_radius = tan(left_radius * dist)/d2t;
		float trans_right_radius = tan(right_radius * dist)/d2t;
		float trans_top_radius = tan(top_radius * dist)/d2t;
		float trans_bottom_radius = tan(bottom_radius * dist)/d2t;

		ofy = fy * ((top_radius + bottom_radius) / (trans_top_radius + trans_bottom_radius)) * ((float)out_height / (float)in_height);
		ocy = (trans_top_radius/top_radius) * ofy*cy/fy;

		ofx = fx * ((left_radius + right_radius) / (trans_left_radius + trans_right_radius)) * ((float)out_width / (float)in_width);
		ocx = (trans_left_radius/left_radius) * ofx*cx/fx;

		printf("new K: %f %f %f %f\n",ofx,ofy,ocx,ocy);
		printf("old K: %f %f %f %f\n",fx,fy,cx,cy);
	}
	else if(outputCalibration[0] == -2)	 // "full"
	{
		float left_radius = cx/fx;
		float right_radius = (in_width-1 - cx)/fx;
		float top_radius = cy/fy;
		float bottom_radius = (in_height-1 - cy)/fy;

		// find left-most and right-most radius
		float tl_radius = sqrt(left_radius*left_radius + top_radius*top_radius);
		float tr_radius = sqrt(right_radius*right_radius + top_radius*top_radius);
		float bl_radius = sqrt(left_radius*left_radius + bottom_radius*bottom_radius);
		float br_radius = sqrt(right_radius*right_radius + bottom_radius*bottom_radius);

		float trans_tl_radius = tan(tl_radius * dist)/d2t;
		float trans_tr_radius = tan(tr_radius * dist)/d2t;
		float trans_bl_radius = tan(bl_radius * dist)/d2t;
		float trans_br_radius = tan(br_radius * dist)/d2t;

		float hor = std::max(br_radius,tr_radius) + std::max(bl_radius,tl_radius);
		float vert = std::max(tr_radius,tl_radius) + std::max(bl_radius,br_radius);

		float trans_hor = std::max(trans_br_radius,trans_tr_radius) + std::max(trans_bl_radius,trans_tl_radius);
		float trans_vert = std::max(trans_tr_radius,trans_tl_radius) + std::max(trans_bl_radius,trans_br_radius);

		ofy = fy * ((vert) / (trans_vert)) * ((float)out_height / (float)in_height);
		ocy = std::max(trans_tl_radius/tl_radius,trans_tr_radius/tr_radius) * ofy*cy/fy;

		ofx = fx * ((hor) / (trans_hor)) * ((float)out_width / (float)in_width);
		ocx = std::max(trans_bl_radius/bl_radius,trans_tl_radius/tl_radius) * ofx*cx/fx;

		printf("new K: %f %f %f %f\n",ofx,ofy,ocx,ocy);
		printf("old K: %f %f %f %f\n",fx,fy,cx,cy);
	}
	else
	{
		ofx = outputCalibration[0] * out_width;
		ofy = outputCalibration[1] * out_height;
		ocx = outputCalibration[2] * out_width-0.5;
		ocy = outputCalibration[3] * out_height-0.5;
	}

	outputCalibration[0] = ofx / out_width;
	outputCalibration[1] = ofy / out_height;
	outputCalibration[2] = (ocx+0.5) / out_width;
	outputCalibration[3] = (ocy+0.5) / out_height;
	outputCalibration[4] = 0;




	// =============================== build rectification map ===============================
	remapX = new float[out_width * out_height];
	remapY = new float[out_width * out_height];
	for(int y=0;y<out_height;y++)
		for(int x=0;x<out_width;x++)
		{
			remapX[x+y*out_width] = x;
			remapY[x+y*out_width] = y;
		}
	distortCoordinates(remapX, remapY, out_height*out_width);

	bool hasBlackPoints = false;
	for(int i=0;i<out_width * out_height;i++)
	{
		if(remapX[i] == 0) remapX[i] = 0.01;
		if(remapY[i] == 0) remapY[i] = 0.01;
		if(remapX[i] == in_width-1) remapX[i] = in_width-1.01;
		if(remapY[i] == in_height-1) remapY[i] = in_height-1.01;


		if(!(remapX[i] > 0 && remapY[i] > 0 && remapX[i] < in_width-1 &&  remapY[i] < in_height-1))
		{
			//printf("black pixel at %d %d %f %f!\n", i, i, remapX[i], remapY[i]);
			hasBlackPoints=true;
			remapX[i]=-1;
			remapY[i]=-1;

		}
	}

	if(hasBlackPoints)
		printf("\n\nFOV Undistorter: Warning! Image has black pixels.\n\n\n");

	// =============================== set Krect ===============================
	Krect.setIdentity();
	Krect(0,0) = outputCalibration[0] * out_width;
	Krect(1,1) = outputCalibration[1] * out_height;
	Krect(0,2) = outputCalibration[2] * out_width-0.5;
	Krect(1,2) = outputCalibration[3] * out_height-0.5;


	Korg.setIdentity();
	Korg(0,0) = inputCalibration[0] * in_width;
	Korg(1,1) = inputCalibration[1] * in_height;
	Korg(0,2) = inputCalibration[2] * in_width-0.5;
	Korg(1,2) = inputCalibration[3] * in_height-0.5;
}



UndistorterFOV::~UndistorterFOV()
{
	if(remapX != 0) delete[] remapX;
	if(remapY != 0) delete[] remapY;
}


void UndistorterFOV::distortCoordinates(float* in_x, float* in_y, int n)
{
	if(!valid)
	{
		printf("ERROR: invalid UndistorterFOV!\n");
		return;
	}


	float dist = inputCalibration[4];
	float d2t = 2.0f * tan(dist / 2.0f);

	// current camera parameters
	float fx = inputCalibration[0] * in_width;
	float fy = inputCalibration[1] * in_height;
	float cx = inputCalibration[2] * in_width - 0.5;
	float cy = inputCalibration[3] * in_height - 0.5;

	float ofx = outputCalibration[0]*out_width;
	float ofy = outputCalibration[1]*out_height;
	float ocx = outputCalibration[2]*out_width-0.5f;
	float ocy = outputCalibration[3]*out_height-0.5f;

	for(int i=0;i<n;i++)
	{
		float x = in_x[i];
		float y = in_y[i];
		float ix = (x - ocx) / ofx;
		float iy = (y - ocy) / ofy;

		float r = sqrtf(ix*ix + iy*iy);
		float fac = (r==0 || dist==0) ? 1 : atanf(r * d2t)/(dist*r);

		ix = fx*fac*ix+cx;
		iy = fy*fac*iy+cy;

		in_x[i] = ix;
		in_y[i] = iy;
	}
}


template<typename T>
void UndistorterFOV::undistort(const T* input, float* output, int nPixIn, int nPixOut) const
{
	if(!valid) return;

	if(nPixIn != in_width*in_height)
	{
		printf("ERROR: undistort called with wrong input image dismesions (expected %d pixel, got %d pixel)\n",
				in_width*in_height, nPixIn);
		return;
	}
	if(nPixOut != out_width*out_height)
	{
		printf("ERROR: undistort called with wrong output image dismesions (expected %d pixel, got %d pixel)\n",
				out_width*out_height, nPixOut);
		return;
	}


	for(int idx = 0; idx < out_width*out_height;idx++)
	{
		// get interp. values
		float xx = remapX[idx];
		float yy = remapY[idx];

		if(xx<0)
			output[idx] = 0;
		else
		{
			// get integer and rational parts
			int xxi = xx;
			int yyi = yy;
			xx -= xxi;
			yy -= yyi;
			float xxyy = xx*yy;

			// get array base pointer
			const T* src = input + xxi + yyi * in_width;

			// interpolate (bilinear)
			output[idx] =  xxyy * src[1+in_width]
								+ (yy-xxyy) * src[in_width]
								+ (xx-xxyy) * src[1]
								+ (1-xx-yy+xxyy) * src[0];
		}
	}
}
template void UndistorterFOV::undistort<float>(const float* input, float* output, int nPixIn, int nPixOut) const;
template void UndistorterFOV::undistort<unsigned char>(const unsigned char* input, float* output, int nPixIn, int nPixOut) const;
