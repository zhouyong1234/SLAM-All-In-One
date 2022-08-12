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


#include "OptimizationBackend/EnergyFunctionalStructs.h"
#include "OptimizationBackend/EnergyFunctional.h"
#include "FullSystem/FullSystem.h"
#include "FullSystem/HessianBlocks.h"
#include "FullSystem/Residuals.h"

#if !defined(__SSE3__) && !defined(__SSE2__) && !defined(__SSE1__)
#include "SSE2NEON.h"
#endif

namespace dso
{


void EFResidual::takeDataF()
{
	std::swap<RawResidualJacobian*>(J, data->J);

	//! 图像导数 * 图像导数 * 逆深度导数
	Vec2f JI_JI_Jd = J->JIdx2 * J->Jpdd;
	//! 位姿导数 * 图像导数 * 图像导数 * 逆深度导数
	for(int i=0;i<6;i++)
		JpJdF[i] = J->Jpdxi[0][i]*JI_JI_Jd[0] + J->Jpdxi[1][i] * JI_JI_Jd[1];
	//! 图像导数 * 逆深度导数 * 光度导数
	JpJdF.segment<2>(6) = J->JabJIdx*J->Jpdd;
}

//@ 从 FrameHessian 中提取数据
void EFFrame::takeData()
{
	prior = data->getPrior().head<8>(); 	// 得到先验状态, 主要是光度仿射变换
	delta = data->get_state_minus_stateZero().head<8>(); // 状态与FEJ零状态之间差
	delta_prior =  (data->get_state() - data->getPriorZero()).head<8>(); // 状态与先验之间的差 //? 可先验是0啊?



	//	Vec10 state_zero =  data->get_state_zero();
	//	state_zero.segment<3>(0) = SCALE_XI_TRANS * state_zero.segment<3>(0);
	//	state_zero.segment<3>(3) = SCALE_XI_ROT * state_zero.segment<3>(3);
	//	state_zero[6] = SCALE_A * state_zero[6];
	//	state_zero[7] = SCALE_B * state_zero[7];
	//	state_zero[8] = SCALE_A * state_zero[8];
	//	state_zero[9] = SCALE_B * state_zero[9];
	//
	//	std::cout << "state_zero: " << state_zero.transpose() << "\n";


	assert(data->frameID != -1);

	frameID = data->frameID;  // 所有帧的ID序号
}



//@ 从PointHessian读取先验和当前状态信息
void EFPoint::takeData()
{
	priorF = data->hasDepthPrior ? setting_idepthFixPrior*SCALE_IDEPTH*SCALE_IDEPTH : 0;
	if(setting_solverMode & SOLVER_REMOVE_POSEPRIOR) priorF=0;

	//TODO 每次都更新线性化点，这不一直是零？？
	deltaF = data->idepth - data->idepth_zero; // 当前状态逆深度减去线性化处
}

//@ 计算线性化更新后的残差,
//! 没平方叫残差, 平方叫能量
void EFResidual::fixLinearizationF(EnergyFunctional* ef)
{
	Vec8f dp = ef->adHTdeltaF[hostIDX+ef->nFrames*targetIDX]; // 得到hostIDX --> targetIDX的状态增量

	// compute Jp*delta
	__m128 Jp_delta_x = _mm_set1_ps(J->Jpdxi[0].dot(dp.head<6>())
								   +J->Jpdc[0].dot(ef->cDeltaF)
								   +J->Jpdd[0]*point->deltaF);
	__m128 Jp_delta_y = _mm_set1_ps(J->Jpdxi[1].dot(dp.head<6>())
								   +J->Jpdc[1].dot(ef->cDeltaF)
								   +J->Jpdd[1]*point->deltaF);
	__m128 delta_a = _mm_set1_ps((float)(dp[6]));
	__m128 delta_b = _mm_set1_ps((float)(dp[7]));

	for(int i=0;i<patternNum;i+=4)
	{
		// PATTERN: rtz = resF - [JI*Jp Ja]*delta.
		__m128 rtz = _mm_load_ps(((float*)&J->resF)+i); // 光度残差
		//! res - J * delta_x
		rtz = _mm_sub_ps(rtz,_mm_mul_ps(_mm_load_ps(((float*)(J->JIdx))+i),Jp_delta_x));
		rtz = _mm_sub_ps(rtz,_mm_mul_ps(_mm_load_ps(((float*)(J->JIdx+1))+i),Jp_delta_y));
		rtz = _mm_sub_ps(rtz,_mm_mul_ps(_mm_load_ps(((float*)(J->JabF))+i),delta_a));
		rtz = _mm_sub_ps(rtz,_mm_mul_ps(_mm_load_ps(((float*)(J->JabF+1))+i),delta_b));
		_mm_store_ps(((float*)&res_toZeroF)+i, rtz); // 存储在res_toZeroF
		// if(res_toZeroF[i] > J->resF[i])
			// printf("true");
	}

	// std::cout<<"resF: "<<J->resF<<" ||   res_toZeroF "<<res_toZeroF<<std::endl;
	isLinearized = true;
}

}
