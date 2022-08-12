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


#pragma once
#include <sstream>
#include <fstream>
#include <dirent.h>
#include <algorithm>

#include "opencv2/opencv.hpp"
#include "FOVUndistorter.h"
#include "PhotometricUndistorter.h"

#include "zip.h"


inline int getdir (std::string dir, std::vector<std::string> &files)
{
    DIR *dp;
    struct dirent *dirp;
    if((dp  = opendir(dir.c_str())) == NULL)
    {
        return -1;
    }

    while ((dirp = readdir(dp)) != NULL) {
    	std::string name = std::string(dirp->d_name);

    	if(name != "." && name != "..")
    		files.push_back(name);
    }
    closedir(dp);


    std::sort(files.begin(), files.end());

    if(dir.at( dir.length() - 1 ) != '/') dir = dir+"/";
	for(unsigned int i=0;i<files.size();i++)
	{
		if(files[i].at(0) != '/')
			files[i] = dir + files[i];
	}

    return files.size();
}



/*
 * provides read functionality for one of the dataset sequences.
 * * path is the folder for the sequence, with trailing slash (e.g. /home/Peter/datasets/sequenceX/
 * 		=> if it contains a filder "images", images will be read from there.
 * 		=> otherwise we assume the images will be zipped in "images.zip".
 * * MT = true: multi-threaded image loading, to allow for real-time playback (reading, decoding & rectifying images takes a while).
 */
class DatasetReader
{
public:
	DatasetReader(std::string folder)
	{
		this->path = folder;
		for(int i=0;i<3;i++)
		{
			ziparchive=0;
			undistorter=0;
			databuffer=0;
		}

		getdir (path+"images/", files);
		if(files.size() > 0)
		{
			printf("Load Dataset %s: found %d files in folder /images; assuming that all images are there.\n",
					path.c_str(), (int)files.size());
			isZipped=false;
		}
		else
		{
			printf("Load Dataset %s: found no in folder /images; assuming that images are zipped.\n",
					path.c_str());
			isZipped=true;

			int ziperror=0;
			ziparchive = zip_open((path+"images.zip").c_str(),  ZIP_RDONLY, &ziperror);
			if(ziperror!=0)
			{
				printf("ERROR %d reading archive %s!\n", ziperror, (path+"images.zip").c_str());
				exit(1);
			}

			files.clear();
			int numEntries = zip_get_num_entries(ziparchive, 0);
			for(int k=0;k<numEntries;k++)
			{
				const char* name = zip_get_name(ziparchive, k,  ZIP_FL_ENC_STRICT);
				std::string nstr = std::string(name);
				if(nstr == "." || nstr == "..") continue;
				files.push_back(name);
			}

			printf("got %d entries and %d files from zipfile!\n", numEntries, (int)files.size());
			std::sort(files.begin(), files.end());

		}
		loadTimestamps(path+"times.txt");


		// create undistorter.
		undistorter = new UndistorterFOV((path+"camera.txt").c_str());
		photoUndistorter = new PhotometricUndistorter(path+"pcalib.txt", path+"vignette.png",undistorter->getInputDims()[0],undistorter->getInputDims()[1]);


		// get image widths.
		widthOrg = undistorter->getInputDims()[0];
		heightOrg = undistorter->getInputDims()[1];
		width= undistorter->getOutputDims()[0];
		height= undistorter->getOutputDims()[1];

		internalTempBuffer=new float[widthOrg*heightOrg];

		printf("Dataset %s: Got %d files!\n", path.c_str(), (int)getNumImages());
	}
	~DatasetReader()
	{
		if(undistorter!=0) delete undistorter;
		if(photoUndistorter!=0) delete photoUndistorter;
		if(ziparchive!=0) zip_close(ziparchive);
		if(databuffer!=0) delete[] databuffer;
		delete[] internalTempBuffer;

	}

	UndistorterFOV* getUndistorter()
	{
		return undistorter;
	}

	PhotometricUndistorter* getPhotoUndistorter()
	{
		return photoUndistorter;
	}

	int getNumImages()
	{
		return files.size();
	}

	double getTimestamp(int id)
	{
		if(id >= (int)timestamps.size()) return 0;
		if(id < 0) return 0;
		return timestamps[id];
	}

	float getExposure(int id)
	{
		if(id >= (int)exposures.size()) return 0;
		if(id < 0) return 0;
		return exposures[id];
	}

	ExposureImage* getImage(int id, bool rectify, bool removeGamma, bool removeVignette, bool nanOverexposed)
	{
		assert(id >= 0 && id < (int)files.size());

		cv::Mat imageRaw = getImageRaw_internal(id);

		if(imageRaw.rows != heightOrg || imageRaw.cols != widthOrg)
		{
			printf("ERROR: expected cv-mat to have dimensions %d x %d; found %d x %d (image %s)!\n",
					widthOrg, heightOrg, imageRaw.cols, imageRaw.rows, files[id].c_str());
			return 0;
		}

		if(imageRaw.type() != CV_8U)
		{
			printf("ERROR: expected cv-mat to have type 8U!\n");
			return 0;
		}

		ExposureImage* ret=0;


		if(removeGamma || removeVignette || nanOverexposed)
		{
			if(!rectify)
			{
				// photo undist only.
				ret = new ExposureImage(widthOrg, heightOrg, timestamps[id], exposures[id], id);
				photoUndistorter->unMapImage(imageRaw.data, ret->image, widthOrg*heightOrg, removeGamma, removeVignette, nanOverexposed );
			}
			else
			{
				// photo undist to buffer, then rect
				ret = new ExposureImage(width, height, timestamps[id], exposures[id], id);
				photoUndistorter->unMapImage(imageRaw.data, internalTempBuffer, widthOrg*heightOrg, removeGamma, removeVignette, nanOverexposed );
				undistorter->undistort<float>(internalTempBuffer, ret->image, widthOrg*heightOrg, width*height);
			}
		}
		else
		{
			if(rectify)
			{
				// rect only.
				ret = new ExposureImage(width, height, timestamps[id], exposures[id], id);
				undistorter->undistort<unsigned char>(imageRaw.data, ret->image, widthOrg*heightOrg, width*height);
			}
			else
			{
				// do nothing.
				ret = new ExposureImage(widthOrg, heightOrg, timestamps[id], exposures[id], id);
				for(int i=0;i<widthOrg*heightOrg;i++)
					ret->image[i] = imageRaw.at<uchar>(i);
			}
		}
		return ret;
	}



	cv::Mat getImageRaw_internal(int id)
	{
		if(!isZipped)
		{
			// CHANGE FOR ZIP FILE
			return cv::imread(files[id],CV_LOAD_IMAGE_GRAYSCALE);
		}
		else
		{
			if(databuffer==0) databuffer = new char[widthOrg*heightOrg*6+10000];
			zip_file_t* fle = zip_fopen(ziparchive, files[id].c_str(), 0);
			long readbytes = zip_fread(fle, databuffer, (long)widthOrg*heightOrg*6+10000);

			if(readbytes > (long)widthOrg*heightOrg*6)
			{
				printf("read %ld/%ld bytes for file %s. increase buffer!!\n", readbytes,(long)widthOrg*heightOrg*6+10000, files[id].c_str());
				delete[] databuffer;
				databuffer = new char[(long)widthOrg*heightOrg*60+1000000];
				fle = zip_fopen(ziparchive, files[id].c_str(), 0);
				readbytes = zip_fread(fle, databuffer, (long)widthOrg*heightOrg*60+100000);

				if(readbytes > (long)widthOrg*heightOrg*60+10000)
				{
					printf("buffer still to small (read %ld/%ld). abort.\n", readbytes,(long)widthOrg*heightOrg*60+100000);
					exit(1);
				}
			}
			return cv::imdecode(cv::Mat(readbytes,1,CV_8U, databuffer), CV_LOAD_IMAGE_GRAYSCALE);
		}
	}


private:


	inline void loadTimestamps(std::string timesFile)
	{
		std::ifstream tr;
		tr.open(timesFile.c_str());
		timestamps.clear();
		exposures.clear();
		while(!tr.eof() && tr.good())
		{
			std::string line;
			char buf[1000];
			tr.getline(buf, 1000);

			int id;
			double stamp;
			float exposure = 0;

			if(3 == sscanf(buf, "%d %lf %f", &id, &stamp, &exposure))
			{
				timestamps.push_back(stamp);
				exposures.push_back(exposure);
			}

			else if(2 == sscanf(buf, "%d %lf", &id, &stamp))
			{
				timestamps.push_back(stamp);
				exposures.push_back(0);
			}
		}
		tr.close();

		if((int)exposures.size()!=(int)getNumImages())
		{
			printf("DatasetReader: Mismatch between number of images and number of timestamps / exposure times. Set all to zero.");
			timestamps.clear();
			exposures.clear();
			for(int i=0;i<(int)getNumImages();i++)
			{
				timestamps.push_back(0.0);
				exposures.push_back(0);
			}
		}
	}

	// data is here.
	std::vector<std::string> files;
	std::vector<double> timestamps;
	std::vector<float> exposures;

	int width, height;
	int widthOrg, heightOrg;

	std::string path;
	bool isZipped;



	// internal structures.
	UndistorterFOV* undistorter;
	PhotometricUndistorter* photoUndistorter;
	zip_t* ziparchive;
	char* databuffer;

	float* internalTempBuffer;
};

