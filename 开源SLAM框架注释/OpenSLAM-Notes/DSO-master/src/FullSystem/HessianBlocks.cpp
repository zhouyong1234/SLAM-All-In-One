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


 
#include "FullSystem/HessianBlocks.h"
#include "util/FrameShell.h"
#include "FullSystem/ImmaturePoint.h"
#include "OptimizationBackend/EnergyFunctionalStructs.h"

namespace dso
{

//@ 从ImmaturePoint构造函数, 不成熟点变地图点
PointHessian::PointHessian(const ImmaturePoint* const rawPoint, CalibHessian* Hcalib)
{
	instanceCounter++;
	host = rawPoint->host; // 主帧
	hasDepthPrior=false;

	idepth_hessian=0;
	maxRelBaseline=0;
	numGoodResiduals=0;

	// set static values & initialization.
	u = rawPoint->u;
	v = rawPoint->v;
	assert(std::isfinite(rawPoint->idepth_max));
	//idepth_init = rawPoint->idepth_GT;

	my_type = rawPoint->my_type;  //似乎是显示用的

	setIdepthScaled((rawPoint->idepth_max + rawPoint->idepth_min)*0.5); //深度均值
	setPointStatus(PointHessian::INACTIVE);

	int n = patternNum;
	memcpy(color, rawPoint->color, sizeof(float)*n);// 一个点对应8个像素
	memcpy(weights, rawPoint->weights, sizeof(float)*n);
	energyTH = rawPoint->energyTH;

	efPoint=0; // 指针=0


}

//@ 释放residual
void PointHessian::release()
{
	for(unsigned int i=0;i<residuals.size();i++) delete residuals[i];
	residuals.clear();
}

//@ 设置固定线性化点位置的状态
//TODO 后面求nullspaces地方没看懂, 回头再看<2019.09.18> 数学原理是啥?
void FrameHessian::setStateZero(const Vec10 &state_zero)
{
	//! 前六维位姿必须是0
	assert(state_zero.head<6>().squaredNorm() < 1e-20);

	this->state_zero = state_zero;

	//! 感觉这个nullspaces_pose就是 Adj_T
	//! Exp(Adj_T*zeta)=T*Exp(zeta)*T^{-1}
	// 全局转为局部的，左乘边右乘
	//! T_c_w * delta_T_g * T_c_w_inv = delta_T_l
	//TODO 这个是数值求导的方法么???
	for(int i=0;i<6;i++)
	{
		Vec6 eps; eps.setZero(); eps[i] = 1e-3;
		SE3 EepsP = Sophus::SE3::exp(eps);
		SE3 EepsM = Sophus::SE3::exp(-eps);
		SE3 w2c_leftEps_P_x0 = (get_worldToCam_evalPT() * EepsP) * get_worldToCam_evalPT().inverse();
		SE3 w2c_leftEps_M_x0 = (get_worldToCam_evalPT() * EepsM) * get_worldToCam_evalPT().inverse();
		nullspaces_pose.col(i) = (w2c_leftEps_P_x0.log() - w2c_leftEps_M_x0.log())/(2e-3);
	}
	//nullspaces_pose.topRows<3>() *= SCALE_XI_TRANS_INVERSE;
	//nullspaces_pose.bottomRows<3>() *= SCALE_XI_ROT_INVERSE;

	//? rethink
	// scale change
	SE3 w2c_leftEps_P_x0 = (get_worldToCam_evalPT());
	w2c_leftEps_P_x0.translation() *= 1.00001;
	w2c_leftEps_P_x0 = w2c_leftEps_P_x0 * get_worldToCam_evalPT().inverse();
	SE3 w2c_leftEps_M_x0 = (get_worldToCam_evalPT());
	w2c_leftEps_M_x0.translation() /= 1.00001;
	w2c_leftEps_M_x0 = w2c_leftEps_M_x0 * get_worldToCam_evalPT().inverse();
	nullspaces_scale = (w2c_leftEps_P_x0.log() - w2c_leftEps_M_x0.log())/(2e-3);


	nullspaces_affine.setZero();
	nullspaces_affine.topLeftCorner<2,1>()  = Vec2(1,0);
	assert(ab_exposure > 0);
	nullspaces_affine.topRightCorner<2,1>() = Vec2(0, expf(aff_g2l_0().a)*ab_exposure);
};



void FrameHessian::release()
{
	// DELETE POINT
	// DELETE RESIDUAL
	for(unsigned int i=0;i<pointHessians.size();i++) delete pointHessians[i];
	for(unsigned int i=0;i<pointHessiansMarginalized.size();i++) delete pointHessiansMarginalized[i];
	for(unsigned int i=0;i<pointHessiansOut.size();i++) delete pointHessiansOut[i];
	for(unsigned int i=0;i<immaturePoints.size();i++) delete immaturePoints[i];


	pointHessians.clear();
	pointHessiansMarginalized.clear();
	pointHessiansOut.clear();
	immaturePoints.clear();
}

//* 计算各层金字塔图像的像素值（辐射强度）和梯度
void FrameHessian::makeImages(float* color, CalibHessian* HCalib)
{
	// 每一层创建图像值, 和图像梯度的存储空间
	for(int i=0;i<pyrLevelsUsed;i++)
	{
		dIp[i] = new Eigen::Vector3f[wG[i]*hG[i]];
		absSquaredGrad[i] = new float[wG[i]*hG[i]];
	}
	dI = dIp[0]; // 原来他们指向同一个地方


	// make d0
	int w=wG[0]; // 零层weight
	int h=hG[0]; // 零层height
	for(int i=0;i<w*h;i++)
		dI[i][0] = color[i];

	for(int lvl=0; lvl<pyrLevelsUsed; lvl++)
	{
		int wl = wG[lvl], hl = hG[lvl]; // 该层图像大小
		Eigen::Vector3f* dI_l = dIp[lvl];

		float* dabs_l = absSquaredGrad[lvl];
		if(lvl>0)
		{
			int lvlm1 = lvl-1;
			int wlm1 = wG[lvlm1]; // 列数
			Eigen::Vector3f* dI_lm = dIp[lvlm1];


			// 像素4合1, 生成金字塔
			for(int y=0;y<hl;y++)
				for(int x=0;x<wl;x++)
				{
					dI_l[x + y*wl][0] = 0.25f * (dI_lm[2*x   + 2*y*wlm1][0] +
												dI_lm[2*x+1 + 2*y*wlm1][0] +
												dI_lm[2*x   + 2*y*wlm1+wlm1][0] +
												dI_lm[2*x+1 + 2*y*wlm1+wlm1][0]);
				}
		}

		for(int idx=wl;idx < wl*(hl-1);idx++) // 第二行开始
		{
			float dx = 0.5f*(dI_l[idx+1][0] - dI_l[idx-1][0]);
			float dy = 0.5f*(dI_l[idx+wl][0] - dI_l[idx-wl][0]);


			if(!std::isfinite(dx)) dx=0;
			if(!std::isfinite(dy)) dy=0;

			dI_l[idx][1] = dx; // 梯度
			dI_l[idx][2] = dy;


			dabs_l[idx] = dx*dx+dy*dy; // 梯度平方

			if(setting_gammaWeightsPixelSelect==1 && HCalib!=0)
			{
				//! 乘上响应函数, 变换回正常的颜色, 因为光度矫正时 I = G^-1(I) / V(x)
				float gw = HCalib->getBGradOnly((float)(dI_l[idx][0])); 
				dabs_l[idx] *= gw*gw;	// convert to gradient of original color space (before removing response).
			}
		}
	}
}

//@ 计算优化前和优化后的相对位姿, 相对光度变化, 及中间变量
void FrameFramePrecalc::set(FrameHessian* host, FrameHessian* target, CalibHessian* HCalib )
{
	this->host = host;    // 这个是赋值, 计数会增加, 不是拷贝
	this->target = target;
	
	//? 实在不懂leftToleft_0这个名字怎么个含义
	// 优化前host target间位姿变换
	SE3 leftToLeft_0 = target->get_worldToCam_evalPT() * host->get_worldToCam_evalPT().inverse();
	PRE_RTll_0 = (leftToLeft_0.rotationMatrix()).cast<float>();
	PRE_tTll_0 = (leftToLeft_0.translation()).cast<float>();


	// 优化后host到target间位姿变换
	SE3 leftToLeft = target->PRE_worldToCam * host->PRE_camToWorld;
	PRE_RTll = (leftToLeft.rotationMatrix()).cast<float>();
	PRE_tTll = (leftToLeft.translation()).cast<float>();
	distanceLL = leftToLeft.translation().norm();

	// 乘上内参, 中间量?
	Mat33f K = Mat33f::Zero();
	K(0,0) = HCalib->fxl();
	K(1,1) = HCalib->fyl();
	K(0,2) = HCalib->cxl();
	K(1,2) = HCalib->cyl();
	K(2,2) = 1;
	PRE_KRKiTll = K * PRE_RTll * K.inverse();
	PRE_RKiTll = PRE_RTll * K.inverse();
	PRE_KtTll = K * PRE_tTll;

	// 光度仿射值
	PRE_aff_mode = AffLight::fromToVecExposure(host->ab_exposure, target->ab_exposure, host->aff_g2l(), target->aff_g2l()).cast<float>();
	PRE_b0_mode = host->aff_g2l_0().b;
}

}

