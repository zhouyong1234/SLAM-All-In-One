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


#include <opencv2/core/core.hpp>
#include "ExposureImage.h"
#include "Eigen/Core"


class UndistorterFOV
{
public:

	UndistorterFOV(const char* configFileName);
	UndistorterFOV();
	~UndistorterFOV();


	template<typename T>
	void undistort(const T* input, float* output, int nPixIn, int nPixOut) const;
	void distortCoordinates(float* in_x, float* in_y, int n);

	inline Eigen::Matrix3f getK_rect() const
	{
		return Krect;
	}
	inline Eigen::Matrix3f getK_org() const
	{
		return Korg;
	}
	inline float getOmega() const
	{
		return inputCalibration[4];
	}
	const Eigen::VectorXf getOriginalCalibration() const
	{
		Eigen::VectorXf vec(5);
		vec[0] = inputCalibration[0] * in_width;
		vec[1] = inputCalibration[1] * in_height;
		vec[2] = inputCalibration[2] * in_width - 0.5;
		vec[3] = inputCalibration[3] * in_height - 0.5;
		vec[4] = inputCalibration[4];
		return vec;
	}
	const Eigen::Vector2i getInputDims() const
	{
		return Eigen::Vector2i(in_width, in_height);
	}
	const Eigen::Vector2i getOutputDims() const
	{
		return Eigen::Vector2i(out_width, out_height);
	}
	
	bool isValid() const
	{
		return valid;
	}
private:
	Eigen::Matrix3f Krect;
	Eigen::Matrix3f Korg;
	float inputCalibration[5];
	float outputCalibration[5];
	int out_width, out_height;
	int in_width, in_height;

	float* remapX;
	float* remapY;

	bool valid;
};
