/*
 * <one line to give the program's name and a brief idea of what it does.>
 * Copyright (C) 2016  <copyright holder> <email>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */


#ifndef COMMON_INCLUDE_H
#define COMMON_INCLUDE_H

// define the commonly included file to avoid a long include list

// for Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>

using Eigen::Vector2d;
using Eigen::Vector3d;

// for cv
#include <opencv2/opencv.hpp>

using cv::Mat;

// std 
#include <vector>
#include <list>
#include <memory>
#include <string>
#include <iostream>
#include <set>
#include <unordered_map>
#include <map>
#include <stdint.h>

//#include "converter.h"


using namespace std; 

using namespace cv;

#define RESET "\033[0m"
#define BLACK "\033[30m"               /* Black */
#define RED "\033[31m"                 /* Red */
#define GREEN "\033[32m"               /* Green */
#define YELLOW "\033[33m"              /* Yellow */
#define BLUE "\033[34m"                /* Blue */
#define MAGENTA "\033[35m"             /* Magenta */
#define CYAN "\033[36m"                /* Cyan */
#define WHITE "\033[37m"               /* White */
#define BOLDBLACK "\033[1m\033[30m"    /* Bold Black */
#define BOLDRED "\033[1m\033[31m"      /* Bold Red */
#define BOLDGREEN "\033[1m\033[32m"    /* Bold Green */
#define BOLDYELLOW "\033[1m\033[33m"   /* Bold Yellow */
#define BOLDBLUE "\033[1m\033[34m"     /* Bold Blue */
#define BOLDMAGENTA "\033[1m\033[35m"  /* Bold Magenta */
#define BOLDCYAN "\033[1m\033[36m"     /* Bold Cyan */
#define BOLDWHITE "\033[1m\033[37m"    /* Bold White */


#endif
