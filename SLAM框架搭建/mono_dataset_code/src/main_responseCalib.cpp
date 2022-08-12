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

#include <sstream>
#include <fstream>
#include <dirent.h>
#include <algorithm>

#include "BenchmarkDatasetReader.h"


int leakPadding=2;
int nits = 10;
int skipFrames = 1;



Eigen::Vector2d rmse(double* G, double* E, std::vector<double> &exposureVec, std::vector<unsigned char*> &dataVec,  int wh)
{
	long double e=0;		// yeah - these will be sums of a LOT of values, so we need super high precision.
	long double num=0;

	int n = dataVec.size();
	for(int i=0;i<n;i++)
	{
		for(int k=0;k<wh;k++)
		{
			if(dataVec[i][k] == 255) continue;
			double r = G[dataVec[i][k]] - exposureVec[i]*E[k];
			if(!std::isfinite(r)) continue;
			e += r*r*1e-10;
			num++;
		}
	}

	return Eigen::Vector2d(1e5*sqrtl((e/num)), (double)num);
}


void plotE(double* E, int w, int h, std::string saveTo="")
{

	// try to find some good color scaling for plotting.
	double offset = 20;
	double min=1e10, max=-1e10;

	double Emin=1e10, Emax=-1e10;

	for(int i=0;i<w*h;i++)
	{
		double le = log(E[i]+offset);
		if(le < min) min = le;
		if(le > max) max = le;

		if(E[i] < Emin) Emin = E[i];
		if(E[i] > Emax) Emax = E[i];
	}

	cv::Mat EImg = cv::Mat(h,w,CV_8UC3);
	cv::Mat EImg16 = cv::Mat(h,w,CV_16U);

	for(int i=0;i<w*h;i++)
	{
		float val = 3 * (exp((log(E[i]+offset)-min) / (max-min))-1) / 1.7183;

		int icP = val;
		float ifP = val-icP;
		icP = icP%3;

		cv::Vec3b color;
		if(icP == 0) color= cv::Vec3b(0 ,	   	0,		     	255*ifP);
		if(icP == 1) color= cv::Vec3b(0, 		255*ifP,     	255);
		if(icP == 2) color= cv::Vec3b(255*ifP, 	255, 			255);

		EImg.at<cv::Vec3b>(i) = color;
		EImg16.at<ushort>(i) = 255* 255* (E[i]-Emin) / (Emax-Emin);
	}

	printf("Irradiance %f - %f\n", Emin, Emax);
	cv::imshow("lnE", EImg);

	if(saveTo != "")
	{
		cv::imwrite(saveTo+".png", EImg);
		cv::imwrite(saveTo+"16.png", EImg16);
	}
}
void plotG(double* G, std::string saveTo="")
{
	cv::Mat GImg = cv::Mat(256,256,CV_32FC1);
	GImg.setTo(0);

	double min=1e10, max=-1e10;

	for(int i=0;i<256;i++)
	{
		if(G[i] < min) min = G[i];
		if(G[i] > max) max = G[i];
	}

	for(int i=0;i<256;i++)
	{
		double val = 256*(G[i]-min) / (max-min);
		for(int k=0;k<256;k++)
		{
			if(val < k)
				GImg.at<float>(k,i) = k-val;
		}
	}

	printf("Inv. Response %f - %f\n", min, max);
	cv::imshow("G", GImg);
	if(saveTo != "") cv::imwrite(saveTo, GImg*255);
}


void parseArgument(char* arg)
{
	int option;

	if(1==sscanf(arg,"leakPadding=%d",&option))
	{
		leakPadding = option;
		printf("leakPadding set to %d!\n", leakPadding);
		return;
	}
	if(1==sscanf(arg,"iterations=%d",&option))
	{
		nits = option;
		printf("nits set to %d!\n", nits);
		return;
	}
	if(1==sscanf(arg,"skip=%d",&option))
	{
		skipFrames = option;
		printf("skipFrames set to %d!\n", skipFrames);
		return;
	}

	printf("could not parse argument \"%s\"!!\n", arg);
}



int main( int argc, char** argv )
{
	// parse arguments
	for(int i=2; i<argc;i++)
		parseArgument(argv[i]);


	// load exposure times & images.
	// first parameter is dataset location.
	int w=0,h=0,n=0;


	DatasetReader* reader = new DatasetReader(argv[1]);
	std::vector<double> exposureVec;
	std::vector<unsigned char*> dataVec;
	for(int i=0;i<reader->getNumImages();i+=skipFrames)
	{
		cv::Mat img = reader->getImageRaw_internal(i);
		if(img.rows==0 || img.cols==0) continue;
		assert(img.type() == CV_8U);

		if((w!=0 && w != img.cols) || img.cols==0)
		{ printf("width mismatch!\n"); exit(1); };
		if((h!=0 && h != img.rows) || img.rows==0)
		{ printf("height mismatch!\n"); exit(1); };
		w = img.cols;
		h = img.rows;


		unsigned char* data = new unsigned char[img.rows*img.cols];
		memcpy(data, img.data, img.rows*img.cols);
		dataVec.push_back(data);
		exposureVec.push_back((double)(reader->getExposure(i)));


		unsigned char* data2 = new unsigned char[img.rows*img.cols];
		for(int it=0;it<leakPadding;it++)
		{
			memcpy(data2, data, img.rows*img.cols);
			for(int y=1;y<h-1;y++)
				for(int x=1;x<w-1;x++)
				{
					if(data[x+y*w]==255)
					{
						data2[x+1 + w*(y+1)] = 255;
						data2[x+1 + w*(y  )] = 255;
						data2[x+1 + w*(y-1)] = 255;

						data2[x   + w*(y+1)] = 255;
						data2[x   + w*(y  )] = 255;
						data2[x   + w*(y-1)] = 255;

						data2[x-1 + w*(y+1)] = 255;
						data2[x-1 + w*(y  )] = 255;
						data2[x-1 + w*(y-1)] = 255;
					}
				}
			memcpy(data, data2, img.rows*img.cols);
		}
		delete[] data2;
	}
	n = dataVec.size();


	printf("loaded %d images\n", n);


	double* E = new double[w*h];		// scene irradiance
	double* En = new double[w*h];		// scene irradiance
	double* G = new double[256];		// inverse response function

	// set starting scene irradiance to mean of all images.
	memset(E,0,sizeof(double)*w*h);
	memset(En,0,sizeof(double)*w*h);
	memset(G,0,sizeof(double)*256);
	for(int i=0;i<n;i++)
		for(int k=0;k<w*h;k++)
		{
			//if(dataVec[i][k]==255) continue;
			E[k] += dataVec[i][k];
			En[k] ++;
		}
	for(int k=0;k<w*h;k++) E[k] = E[k]/En[k];



	if(-1 == system("rm -rf photoCalibResult")) printf("could not delete old photoCalibResult folder!\n");
	if(-1 == system("mkdir photoCalibResult")) printf("could not create photoCalibResult folder!\n");


	std::ofstream logFile;
	logFile.open("photoCalibResult/log.txt", std::ios::trunc | std::ios::out);
	logFile.precision(15);


	printf("init RMSE = %f! \t", rmse(G, E, exposureVec, dataVec, w*h )[0]);
	plotE(E,w,h, "photoCalibResult/E-0");
	cv::waitKey(100);


	bool optE = true;
	bool optG = true;


	for(int it=0;it<nits;it++)
	{
		if(optG)
		{
			// optimize log inverse response function.
			double* GSum = new double[256];
			double* GNum = new double[256];
			memset(GSum,0,256*sizeof(double));
			memset(GNum,0,256*sizeof(double));
			for(int i=0;i<n;i++)
			{
				for(int k=0;k<w*h;k++)
				{
					int b = dataVec[i][k];
					if(b == 255) continue;
					GNum[b]++;
					GSum[b]+= E[k] * exposureVec[i];
				}
			}
			for(int i=0;i<256;i++)
			{
				G[i] = GSum[i] / GNum[i];
				if(!std::isfinite(G[i]) && i > 1) G[i] = G[i-1] + (G[i-1]-G[i-2]);
			}
			delete[] GSum;
			delete[] GNum;
			printf("optG RMSE = %f! \t", rmse(G, E, exposureVec, dataVec, w*h )[0]);

			char buf[1000]; snprintf(buf, 1000, "photoCalibResult/G-%d.png", it+1);
			plotG(G, buf);
		}





		if(optE)
		{
			// optimize scene irradiance function.
			double* ESum = new double[w*h];
			double* ENum = new double[w*h];
			memset(ESum,0,w*h*sizeof(double));
			memset(ENum,0,w*h*sizeof(double));
			for(int i=0;i<n;i++)
			{
				for(int k=0;k<w*h;k++)
				{
					int b = dataVec[i][k];
					if(b == 255) continue;
					ENum[k] += exposureVec[i]*exposureVec[i];
					ESum[k] += (G[b]) * exposureVec[i];
				}
			}
			for(int i=0;i<w*h;i++)
			{
				E[i] = ESum[i] / ENum[i];
				if(E[i] < 0) E[i] = 0;
			}

			delete[] ENum;
			delete[] ESum;
			printf("OptE RMSE = %f!  \t", rmse(G, E, exposureVec, dataVec, w*h )[0]);

			char buf[1000]; snprintf(buf, 1000, "photoCalibResult/E-%d", it+1);
			plotE(E,w,h, buf);
		}


		// rescale such that maximum response is 255 (fairly arbitrary choice).
		double rescaleFactor=255.0 / G[255];
		for(int i=0;i<w*h;i++)
		{
			E[i] *= rescaleFactor;
			if(i<256) G[i] *= rescaleFactor;
		}
		Eigen::Vector2d err = rmse(G, E, exposureVec, dataVec, w*h );
		printf("resc RMSE = %f!  \trescale with %f!\n",  err[0], rescaleFactor);

		logFile << it << " " << n << " " << err[1] << " " << err[0] << "\n";

		cv::waitKey(100);
	}

	logFile.flush();
	logFile.close();

	std::ofstream lg;
	lg.open("photoCalibResult/pcalib.txt", std::ios::trunc | std::ios::out);
	lg.precision(15);
	for(int i=0;i<256;i++)
		lg << G[i] << " ";
	lg << "\n";

	lg.flush();
	lg.close();

	delete[] E;
	delete[] En;
	delete[] G;
	for(int i=0;i<n;i++) delete[] dataVec[i];
	return 0;
}
