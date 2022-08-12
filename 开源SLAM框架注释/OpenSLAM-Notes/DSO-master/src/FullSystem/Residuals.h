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

 
#include "util/globalCalib.h"
#include "vector"
 
#include "util/NumType.h"
#include <iostream>
#include <fstream>
#include "util/globalFuncs.h"
#include "OptimizationBackend/RawResidualJacobian.h"

namespace dso
{
class PointHessian;
class FrameHessian;
class CalibHessian;

class EFResidual;

enum ResLocation {ACTIVE=0, LINEARIZED, MARGINALIZED, NONE};
enum ResState {IN=0, OOB, OUTLIER};  // IN在内部, OOB 点超出图像, OUTLIER有外点 

struct FullJacRowT
{
	Eigen::Vector2f projectedTo[MAX_RES_PER_POINT];
};

class PointFrameResidual
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	EFResidual* efResidual;

	static int instanceCounter;


	ResState state_state;				//!< 上一次的残差状态
	double state_energy;				//!< 上一次的能量值
	ResState state_NewState;			//!< 新的一次计算的状态
	double state_NewEnergy;				//!< 新的能量, 如果大于阈值则把等于阈值
	double state_NewEnergyWithOutlier;	//!< 可能具有外点的能量, 可能大于阈值


	void setState(ResState s) {state_state = s;}


	PointHessian* point;		//!< 点
	FrameHessian* host;			//!< 主帧
	FrameHessian* target;		//!< 目标帧
	RawResidualJacobian* J;		//!< 残差对变量的各种雅克比


	bool isNew;


	Eigen::Vector2f projectedTo[MAX_RES_PER_POINT]; //!< 各个patch的投影坐标
	Vec3f centerProjectedTo;	//!< patch的中心点投影 [像素x, 像素y, 新帧逆深度]

	~PointFrameResidual();
	PointFrameResidual();
	PointFrameResidual(PointHessian* point_, FrameHessian* host_, FrameHessian* target_);
	double linearize(CalibHessian* HCalib);


	void resetOOB()
	{
		state_NewEnergy = state_energy = 0;
		state_NewState = ResState::OUTLIER;

		setState(ResState::IN);
	};
	void applyRes( bool copyJacobians);

	void debugPlot();

	void printRows(std::vector<VecX> &v, VecX &r, int nFrames, int nPoints, int M, int res);
};
}

