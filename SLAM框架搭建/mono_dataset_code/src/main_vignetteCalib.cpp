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




#include "opencv2/opencv.hpp"
#include "opencv2/video/tracking.hpp"

#include <aruco/aruco.h>
#include <aruco/cvdrawingutils.h>
#include <opencv2/highgui/highgui.hpp>

#include "BenchmarkDatasetReader.h"
#include "Eigen/Core"
#include "Eigen/LU"

#include <sstream>
#include <fstream>
#include <dirent.h>
#include <algorithm>


// reads interpolated element from a uchar* array
// SSE2 optimization possible
EIGEN_ALWAYS_INLINE float getInterpolatedElement(const float* const mat, const float x, const float y, const int width)
{
	//stats.num_pixelInterpolations++;

	int ix = (int)x;
	int iy = (int)y;
	float dx = x - ix;
	float dy = y - iy;
	float dxdy = dx*dy;
	const float* bp = mat +ix+iy*width;


	float res =   dxdy * bp[1+width]
				+ (dy-dxdy) * bp[width]
				+ (dx-dxdy) * bp[1]
				+ (1-dx-dy+dxdy) * bp[0];

	return res;
}

void displayImage(float* I, int w, int h, std::string name)
{
	float vmin=1e10;
	float vmax=-1e10;

	for(int i=0;i<w*h;i++)
	{
		if(vmin > I[i]) vmin = I[i];
		if(vmax < I[i]) vmax = I[i];
	}

	cv::Mat img = cv::Mat(h,w,CV_8UC3);

	for(int i=0;i<w*h;i++)
	{
		if(isnanf(I[i])) img.at<cv::Vec3b>(i) = cv::Vec3b(0,0,255);
		else img.at<cv::Vec3b>(i) = cv::Vec3b(255*(I[i]-vmin) / (vmax-vmin),255*(I[i]-vmin) / (vmax-vmin),255*(I[i]-vmin) / (vmax-vmin));
	}

	printf("plane image values %f - %f!\n", vmin, vmax);
	cv::imshow(name, img);
	cv::imwrite("vignetteCalibResult/plane.png", img);
}
void displayImageV(float* I, int w, int h, std::string name)
{
	cv::Mat img = cv::Mat(h,w,CV_8UC3);
	for(int i=0;i<w*h;i++)
	{
		if(isnanf(I[i]))
			img.at<cv::Vec3b>(i) = cv::Vec3b(0,0,255);
		else
		{
			float c = 254*I[i];
			img.at<cv::Vec3b>(i) = cv::Vec3b(c,c,c);
		}

	}
	cv::imshow(name, img);
}




int imageSkip=1;


int maxIterations=20;
int outlierTh = 15;

// grid width for template image.
int gw = 1000;
int gh = 1000;

// width of grid relative to marker (fac times marker size)
float facw = 5;
float fach = 5;

// remove pixel with absolute gradient larger than this from the optimization.
int maxAbsGrad = 255;

void parseArgument(char* arg)
{
	int option;
	float optionf;

	if(1==sscanf(arg,"iterations=%d",&option))
	{
		maxIterations = option;
		printf("nits set to %d!\n", maxIterations);
		return;
	}

	if(1==sscanf(arg,"skip=%d",&option))
	{
		imageSkip = option;
		printf("skipFrames set to %d!\n", imageSkip);
		return;
	}

	if(1==sscanf(arg,"patternX=%d",&option))
	{
		gw = option;
		printf("patternX set to %d!\n", gw);
		return;
	}

	if(1==sscanf(arg,"patternY=%d",&option))
	{
		gh = option;
		printf("patternY set to %d!\n", gh);
		return;
	}

	if(1==sscanf(arg,"facW=%f",&optionf))
	{
		facw = optionf;
		printf("facw set to %f!\n", facw);
		return;
	}

	if(1==sscanf(arg,"facH=%f",&optionf))
	{
		fach = optionf;
		printf("fach set to %f!\n", fach);
		return;
	}

	printf("could not parse argument \"%s\"!!\n", arg);
}





int main( int argc, char** argv )
{
	for(int i=2; i<argc;i++)
		parseArgument(argv[i]);

	if(-1 == system("rm -rf vignetteCalibResult")) printf("could not delete old vignetteCalibResult folder!\n");
	if(-1 == system("mkdir vignetteCalibResult")) printf("could not delete old vignetteCalibResult folder!\n");

	// affine map from plane cordinates to grid coordinates.
	Eigen::Matrix3f K_p2idx = Eigen::Matrix3f::Identity();
	K_p2idx(0,0) = gw / facw;
	K_p2idx(1,1) = gh / fach;
	K_p2idx(0,2) = gw / 2;
	K_p2idx(1,2) = gh / 2;
	Eigen::Matrix3f K_p2idx_inverse = K_p2idx.inverse();


	// load images, rectify and estimate the camera pose wrt. the plane.
	DatasetReader* reader = new DatasetReader(argv[1]);
	printf("SEQUENCE NAME: %s!\n", argv[1]);

	int w_out, h_out;
	//Eigen::Matrix3f K = reader->getUndistorter()->getK_rect();
	w_out = reader->getUndistorter()->getOutputDims()[0];
	h_out = reader->getUndistorter()->getOutputDims()[1];

	aruco::MarkerDetector MDetector;

	std::vector<float*> images;
	std::vector<float*> p2imgX;
	std::vector<float*> p2imgY;

	int wI = reader->getUndistorter()->getInputDims()[0];
	int hI = reader->getUndistorter()->getInputDims()[1];


	float meanExposure = 0;
	for(int i=0;i<reader->getNumImages();i+=imageSkip)
		meanExposure+=reader->getExposure(i);
	meanExposure = meanExposure/reader->getNumImages();

	if(meanExposure==0) meanExposure = 1;


	for(int i=0;i<reader->getNumImages();i+=imageSkip)
	{
        std::vector<aruco::Marker> Markers;
		ExposureImage* img = reader->getImage(i,true, false, false, false);

		cv::Mat InImage;
		cv::Mat(h_out, w_out, CV_32F, img->image).convertTo(InImage, CV_8U, 1, 0);
		delete img;

		MDetector.detect(InImage,Markers);
		if(Markers.size() != 1) continue;

        std::vector<cv::Point2f> ptsP;
        std::vector<cv::Point2f> ptsI;
		ptsI.push_back(cv::Point2f(Markers[0][0].x, Markers[0][0].y));
		ptsI.push_back(cv::Point2f(Markers[0][1].x, Markers[0][1].y));
		ptsI.push_back(cv::Point2f(Markers[0][2].x, Markers[0][2].y));
		ptsI.push_back(cv::Point2f(Markers[0][3].x, Markers[0][3].y));
		ptsP.push_back(cv::Point2f(-0.5,0.5));
		ptsP.push_back(cv::Point2f(0.5,0.5));
		ptsP.push_back(cv::Point2f(0.5,-0.5));
		ptsP.push_back(cv::Point2f(-0.5,-0.5));

		cv::Mat Hcv = cv::findHomography(ptsP, ptsI);
		Eigen::Matrix3f H;
		H(0,0) = Hcv.at<double>(0,0);
		H(0,1) = Hcv.at<double>(0,1);
		H(0,2) = Hcv.at<double>(0,2);
		H(1,0) = Hcv.at<double>(1,0);
		H(1,1) = Hcv.at<double>(1,1);
		H(1,2) = Hcv.at<double>(1,2);
		H(2,0) = Hcv.at<double>(2,0);
		H(2,1) = Hcv.at<double>(2,1);
		H(2,2) = Hcv.at<double>(2,2);

		ExposureImage* imgRaw = reader->getImage(i,false, true, false, false);


		float* plane2imgX = new float[gw*gh];
		float* plane2imgY = new float[gw*gh];

		Eigen::Matrix3f HK = H*K_p2idx_inverse;


		int idx=0;
		for(int y=0;y<gh;y++)
			for(int x=0;x<gw;x++)
			{
				Eigen::Vector3f pp = HK*Eigen::Vector3f(x,y,1);
				plane2imgX[idx] = pp[0] / pp[2];
				plane2imgY[idx] = pp[1] / pp[2];
				idx++;
			}

		reader->getUndistorter()->distortCoordinates(plane2imgX, plane2imgY, gw*gh);

		if(imgRaw->exposure_time == 0) imgRaw->exposure_time = 1;

		float* image = new float[wI*hI];
		for(int y=0; y<hI;y++)
			for(int x=0; x<wI;x++)
				image[x+y*wI] = meanExposure*imgRaw->image[x+y*wI] / imgRaw->exposure_time;

		for(int y=2; y<hI-2;y++)
			for(int x=2; x<wI-2;x++)
			{
				for(int deltax=-2; deltax<3;deltax++)
					for(int deltay=-2; deltay<3;deltay++)
					{
						if(fabsf(image[x+y*wI] - image[x+deltax+(y+deltay)*wI]) > maxAbsGrad) { image[x+y*wI] = NAN; image[x+deltax+(y+deltay)*wI]=NAN; }
					}
			}

		images.push_back(image);


		// debug-plot.
		cv::Mat dbgImg(imgRaw->h, imgRaw->w, CV_8UC3);
		for(int i=0;i<imgRaw->w*imgRaw->h;i++)
			dbgImg.at<cv::Vec3b>(i) = cv::Vec3b(imgRaw->image[i], imgRaw->image[i], imgRaw->image[i]);

		for(int x=0; x<=gw;x+=200)
			for(int y=0; y<=gh;y+=10)
			{
				int idxS = (x<gw ? x : gw-1)+(y<gh ? y : gh-1)*gw;
				int idxT = (x<gw ? x : gw-1)+((y+10)<gh ? (y+10) : gh-1)*gw;

				int u_dS = plane2imgX[idxS]+0.5;
				int v_dS = plane2imgY[idxS]+0.5;

				int u_dT = plane2imgX[idxT]+0.5;
				int v_dT = plane2imgY[idxT]+0.5;

				if(u_dS>=0 && v_dS >=0 && u_dS<wI && v_dS<hI && u_dT>=0 && v_dT >=0 && u_dT<wI && v_dT<hI)
					cv::line(dbgImg, cv::Point(u_dS, v_dS), cv::Point(u_dT, v_dT), cv::Scalar(0,0,255), 10, CV_AA);
			}


		for(int x=0; x<=gw;x+=10)
			for(int y=0; y<=gh;y+=200)
			{
				int idxS = (x<gw ? x : gw-1)+(y<gh ? y : gh-1)*gw;
				int idxT = ((x+10)<gw ? (x+10) : gw-1)+(y<gh ? y : gh-1)*gw;

				int u_dS = plane2imgX[idxS]+0.5;
				int v_dS = plane2imgY[idxS]+0.5;

				int u_dT = plane2imgX[idxT]+0.5;
				int v_dT = plane2imgY[idxT]+0.5;

				if(u_dS>=0 && v_dS >=0 && u_dS<wI && v_dS<hI && u_dT>=0 && v_dT >=0 && u_dT<wI && v_dT<hI)
					cv::line(dbgImg, cv::Point(u_dS, v_dS), cv::Point(u_dT, v_dT), cv::Scalar(0,0,255), 10, CV_AA);
			}



		for(int x=0; x<gw;x++)
			for(int y=0; y<gh;y++)
			{
				int u_d = plane2imgX[x+y*gw]+0.5;
				int v_d = plane2imgY[x+y*gw]+0.5;

				if(!(u_d>1 && v_d >1 && u_d<wI-2 && v_d<hI-2))
				{
					plane2imgX[x+y*gw] = NAN;
					plane2imgY[x+y*gw] = NAN;
				}
			}

		cv::imshow("inRaw",dbgImg);

		if(rand()%40==0)
		{
			char buf[1000];
			snprintf(buf,1000,"vignetteCalibResult/img%d.png",i);
			cv::imwrite(buf, dbgImg);
		}

		cv::waitKey(1);

		p2imgX.push_back(plane2imgX);
		p2imgY.push_back(plane2imgY);
	}


	std::ofstream logFile;
	logFile.open("vignetteCalibResult/log.txt", std::ios::trunc | std::ios::out);
	logFile.precision(15);



	int n = images.size();
	float* planeColor = new float[gw*gh];
	float* planeColorFF = new float[gw*gh];
	float* planeColorFC = new float[gw*gh];
	float* vignetteFactor = new float[hI*wI];
	float* vignetteFactorTT = new float[hI*wI];
	float* vignetteFactorCT = new float[hI*wI];


	// initialize vignette factors to 1.
	for(int i=0;i<hI*wI;i++) vignetteFactor[i] = 1;

	double E=0;
	double R=0;
	for(int it=0;it<maxIterations;it++)
	{
		int oth2 = outlierTh*outlierTh;
		if(it < maxIterations/2) oth2=10000*10000;

		// ============================ optimize planeColor ================================
		memset(planeColorFF,0,gw*gh*sizeof(float));
		memset(planeColorFC,0,gw*gh*sizeof(float));
		E=0;R=0;

		// for each plane pixel, it's optimum is at sum(CF)/sum(FF)
		for(int img=0;img<n;img++)	// for all images
		{
			float* plane2imgX = p2imgX[img];
			float* plane2imgY = p2imgY[img];
			float* image = images[img];

			for(int pi=0;pi<gw*gh;pi++)		// for all plane points
			{
				if(isnanf(plane2imgX[pi])) continue;

				// get vignetted color at that point, and add to build average.
				float color = getInterpolatedElement(image, plane2imgX[pi], plane2imgY[pi], wI);
				float fac = getInterpolatedElement(vignetteFactor, plane2imgX[pi], plane2imgY[pi], wI);

				if(isnanf(fac)) continue;
				if(isnanf(color)) continue;

				double residual = (double)((color - planeColor[pi]*fac)*(color - planeColor[pi]*fac));
				if(abs(residual) > oth2)
				{
					E += oth2;
					R ++;
					continue;
				}


				planeColorFF[pi] += fac*fac;
				planeColorFC[pi] += color*fac;

				if(isnanf(planeColor[pi])) continue;
				E += residual;
				R ++;
			}
		}

		for(int pi=0;pi<gw*gh;pi++)		// for all plane points
		{
			if(planeColorFF[pi] < 1)
				planeColor[pi]=NAN;
			else
				planeColor[pi] = planeColorFC[pi] / planeColorFF[pi];
		}
		displayImage(planeColor, gw, gh, "Plane");

		printf("%f residual terms => %f\n", R, sqrtf(E/R));





		// ================================ optimize vignette =======================================
		memset(vignetteFactorTT,0,hI*wI*sizeof(float));
		memset(vignetteFactorCT,0,hI*wI*sizeof(float));
		E=0;R=0;

		for(int img=0;img<n;img++)	// for all images
		{
			float* plane2imgX = p2imgX[img];
			float* plane2imgY = p2imgY[img];
			float* image = images[img];

			for(int pi=0;pi<gw*gh;pi++)		// for all plane points
			{
				if(isnanf(plane2imgX[pi])) continue;
				float x = plane2imgX[pi];
				float y = plane2imgY[pi];

				float colorImage = getInterpolatedElement(image, x, y, wI);
				float fac = getInterpolatedElement(vignetteFactor, x, y, wI);
				float colorPlane = planeColor[pi];

				if(isnanf(colorPlane)) continue;
				if(isnanf(colorImage)) continue;

				double residual = (double)((colorImage - colorPlane*fac)*(colorImage - colorPlane*fac));
				if(abs(residual) > oth2)
				{
					E += oth2;
					R ++;
					continue;
				}


				int ix = (int)x;
				int iy = (int)y;
				float dx = x - ix;
				float dy = y - iy;
				float dxdy = dx*dy;

				vignetteFactorTT[ix+iy*wI + 0] += (1-dx-dy+dxdy) * 	colorPlane*colorPlane;
				vignetteFactorTT[ix+iy*wI + 1] += (dx-dxdy) * 		colorPlane*colorPlane;
				vignetteFactorTT[ix+iy*wI + wI] += (dy-dxdy) * 		colorPlane*colorPlane;
				vignetteFactorTT[ix+iy*wI + 1+wI] += dxdy * 		colorPlane*colorPlane;

				vignetteFactorCT[ix+iy*wI + 0] += (1-dx-dy+dxdy) * 	colorImage*colorPlane;
				vignetteFactorCT[ix+iy*wI + 1] += (dx-dxdy) * 		colorImage*colorPlane;
				vignetteFactorCT[ix+iy*wI + wI] += (dy-dxdy) * 		colorImage*colorPlane;
				vignetteFactorCT[ix+iy*wI + 1+wI] += dxdy * 		colorImage*colorPlane;

				if(isnanf(fac)) continue;
				E += residual;
				R ++;
			}
		}

		float maxFac=0;
		for(int pi=0;pi<hI*wI;pi++)		// for all plane points
		{
			if(vignetteFactorTT[pi] < 1)
				vignetteFactor[pi]=NAN;
			else
			{
				vignetteFactor[pi] = vignetteFactorCT[pi] / vignetteFactorTT[pi];
				if(vignetteFactor[pi]>maxFac) maxFac=vignetteFactor[pi];
			}
		}

		printf("%f residual terms => %f\n", R, sqrtf(E/R));

		// normalize to vignette max. factor 1.
		for(int pi=0;pi<hI*wI;pi++)
			vignetteFactor[pi] /= maxFac;



		logFile << it << " " << n << " " << R << " " << sqrtf(E/R) << "\n";






		// dilate & smoothe vignette by 4 pixel for output.
		// does not change anything in the optimization; uses vignetteFactorTT and vignetteFactorCT for temporary storing
		{
			memcpy(vignetteFactorTT, vignetteFactor, sizeof(float)*hI*wI);
			for(int dilit=0; dilit<4;dilit++)
			{
				memcpy(vignetteFactorCT, vignetteFactorTT, sizeof(float)*hI*wI);
				for(int y=0; y<hI;y++)
					for(int x=0; x<wI;x++)
					{
						int idx = x+y*wI;
						{
							float sum=0, num=0;
							if(x<wI-1 && y<hI-1 && !isnanf(vignetteFactorCT[idx+1+wI])) {sum += vignetteFactorCT[idx+1+wI]; num++;}
							if(x<wI-1 &&           !isnanf(vignetteFactorCT[idx+1])) {sum += vignetteFactorCT[idx+1]; num++;}
							if(x<wI-1 && y>0 &&    !isnanf(vignetteFactorCT[idx+1-wI])) {sum += vignetteFactorCT[idx+1-wI]; num++;}

							if(y<hI-1 &&           !isnanf(vignetteFactorCT[idx+wI])) {sum += vignetteFactorCT[idx+wI]; num++;}
							if(			           !isnanf(vignetteFactorCT[idx])) {sum += vignetteFactorCT[idx]; num++;}
							if(y>0 &&              !isnanf(vignetteFactorCT[idx-wI])) {sum += vignetteFactorCT[idx-wI]; num++;}

							if(y<hI-1 && x>0 &&    !isnanf(vignetteFactorCT[idx-1+wI])) {sum += vignetteFactorCT[idx-1+wI]; num++;}
							if(x>0 &&              !isnanf(vignetteFactorCT[idx-1])) {sum += vignetteFactorCT[idx-1]; num++;}
							if(y>0 && x>0 &&       !isnanf(vignetteFactorCT[idx-1-wI])) {sum += vignetteFactorCT[idx-1-wI]; num++;}

							if(num>0) vignetteFactorTT[idx] = sum/num;
						}
					}
			}

			{
				displayImageV(vignetteFactorTT, wI, hI, "VignetteSmoothed");
				cv::Mat wrap = cv::Mat(hI, wI, CV_32F, vignetteFactorTT)*254.9*254.9;
				cv::Mat wrap16;
				wrap.convertTo(wrap16, CV_16U,1,0);
				cv::imwrite("vignetteCalibResult/vignetteSmoothed.png", wrap16);
				cv::waitKey(50);
			}
			{
				displayImageV(vignetteFactor, wI, hI, "VignetteOrg");
				cv::Mat wrap = cv::Mat(hI, wI, CV_32F, vignetteFactor)*254.9*254.9;
				cv::Mat wrap16;
				wrap.convertTo(wrap16, CV_16U,1,0);
				cv::imwrite("vignetteCalibResult/vignette.png", wrap16);
				cv::waitKey(50);
			}
		}
	}



	logFile.flush();
	logFile.close();

	delete[] planeColor;
	delete[] planeColorFF;
	delete[] planeColorFC;
	delete[] vignetteFactor;
	delete[] vignetteFactorTT;
	delete[] vignetteFactorCT;


	for(int i=0;i<n;i++)
	{
		delete[] images[i];
		delete[] p2imgX[i];
		delete[] p2imgY[i];
	}


	delete reader;
}
