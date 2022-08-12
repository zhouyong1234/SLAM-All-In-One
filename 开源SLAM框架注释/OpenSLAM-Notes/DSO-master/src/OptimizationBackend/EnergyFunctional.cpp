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


#include "OptimizationBackend/EnergyFunctional.h"
#include "OptimizationBackend/EnergyFunctionalStructs.h"
#include "FullSystem/FullSystem.h"
#include "FullSystem/HessianBlocks.h"
#include "FullSystem/Residuals.h"
#include "OptimizationBackend/AccumulatedSCHessian.h"
#include "OptimizationBackend/AccumulatedTopHessian.h"

#if !defined(__SSE3__) && !defined(__SSE2__) && !defined(__SSE1__)
#include "SSE2NEON.h"
#endif

namespace dso
{


bool EFAdjointsValid = false;		//!< 是否设置状态伴随矩阵
bool EFIndicesValid = false;		//!< 是否设置frame, point, res的ID
bool EFDeltaValid = false;			//!< 是否设置状态增量值

//@ 计算adHost(F), adTarget(F)
// 传的参数也没用啊
void EnergyFunctional::setAdjointsF(CalibHessian* Hcalib)
{

	if(adHost != 0) delete[] adHost;
	if(adTarget != 0) delete[] adTarget;
	adHost = new Mat88[nFrames*nFrames];
	adTarget = new Mat88[nFrames*nFrames];

	for(int h=0;h<nFrames;h++) // 主帧
		for(int t=0;t<nFrames;t++) // 目标帧
		{
			FrameHessian* host = frames[h]->data;
			FrameHessian* target = frames[t]->data;

			SE3 hostToTarget = target->get_worldToCam_evalPT() * host->get_worldToCam_evalPT().inverse();

			Mat88 AH = Mat88::Identity();
			Mat88 AT = Mat88::Identity();
			
			// 见笔记推导吧, 或者https://www.cnblogs.com/JingeTU/p/9077372.html
			AH.topLeftCorner<6,6>() = -hostToTarget.Adj().transpose(); //* 转置是因为后面stitchDoubleInternal计算hessian时候就不转了
			AT.topLeftCorner<6,6>() = Mat66::Identity();

			// 光度参数, 合并项对参数求导
			//! E = Ij - tj*exp(aj) / ti*exp(ai) * Ii - (bj - tj*exp(aj) / ti*exp(ai) * bi)
			//! a = - tj*exp(aj) / ti*exp(ai),  b = - (bj - tj*exp(aj) / ti*exp(ai) * bi)
			Vec2f affLL = AffLight::fromToVecExposure(host->ab_exposure, target->ab_exposure, host->aff_g2l_0(), target->aff_g2l_0()).cast<float>();
			AT(6,6) = -affLL[0]; //! a'(aj)
			AH(6,6) = affLL[0];	 //! a'(ai)
			AT(7,7) = -1;		 //! b'(bj)
			AH(7,7) = affLL[0];	 //! b'(bi)

			AH.block<3,8>(0,0) *= SCALE_XI_TRANS;
			AH.block<3,8>(3,0) *= SCALE_XI_ROT;
			AH.block<1,8>(6,0) *= SCALE_A;
			AH.block<1,8>(7,0) *= SCALE_B;
			AT.block<3,8>(0,0) *= SCALE_XI_TRANS;
			AT.block<3,8>(3,0) *= SCALE_XI_ROT;
			AT.block<1,8>(6,0) *= SCALE_A;    //? 已经是乘过的, 怎么又乘一遍
			AT.block<1,8>(7,0) *= SCALE_B;

			adHost[h+t*nFrames] = AH;
			adTarget[h+t*nFrames] = AT;
		}
	cPrior = VecC::Constant(setting_initialCalibHessian); // 常数矩阵

	// float型
	if(adHostF != 0) delete[] adHostF;
	if(adTargetF != 0) delete[] adTargetF;
	adHostF = new Mat88f[nFrames*nFrames];
	adTargetF = new Mat88f[nFrames*nFrames];

	for(int h=0;h<nFrames;h++)
		for(int t=0;t<nFrames;t++)
		{
			adHostF[h+t*nFrames] = adHost[h+t*nFrames].cast<float>();
			adTargetF[h+t*nFrames] = adTarget[h+t*nFrames].cast<float>();
		}

	cPriorF = cPrior.cast<float>();


	EFAdjointsValid = true;
}



EnergyFunctional::EnergyFunctional()
{
	adHost=0;
	adTarget=0;


	red=0;

	adHostF=0;
	adTargetF=0;
	adHTdeltaF=0;

	nFrames = nResiduals = nPoints = 0;

	HM = MatXX::Zero(CPARS,CPARS); // 初始的, 后面增加frame改变
	bM = VecX::Zero(CPARS);


	accSSE_top_L = new AccumulatedTopHessianSSE();
	accSSE_top_A = new AccumulatedTopHessianSSE();
	accSSE_bot = new AccumulatedSCHessianSSE();

	resInA = resInL = resInM = 0;
	currentLambda=0;
}
EnergyFunctional::~EnergyFunctional()
{
	for(EFFrame* f : frames)
	{
		for(EFPoint* p : f->points)
		{
			for(EFResidual* r : p->residualsAll)
			{
				r->data->efResidual=0;
				delete r;
			}
			p->data->efPoint=0;
			delete p;
		}
		f->data->efFrame=0;
		delete f;
	}

	if(adHost != 0) delete[] adHost;
	if(adTarget != 0) delete[] adTarget;


	if(adHostF != 0) delete[] adHostF;
	if(adTargetF != 0) delete[] adTargetF;
	if(adHTdeltaF != 0) delete[] adHTdeltaF;



	delete accSSE_top_L;
	delete accSSE_top_A;
	delete accSSE_bot;
}



//@ 计算各种状态的相对量的增量
void EnergyFunctional::setDeltaF(CalibHessian* HCalib)
{
	if(adHTdeltaF != 0) delete[] adHTdeltaF;
	adHTdeltaF = new Mat18f[nFrames*nFrames];
	for(int h=0;h<nFrames;h++)
		for(int t=0;t<nFrames;t++)
		{
			int idx = h+t*nFrames;
			//! delta_th = Adj * delta_t or delta_th = Adj * delta_h
			// 加一起应该是, 两帧之间位姿变换的增量, 因为h变一点, t变一点
			adHTdeltaF[idx] = frames[h]->data->get_state_minus_stateZero().head<8>().cast<float>().transpose() * adHostF[idx]
					        +frames[t]->data->get_state_minus_stateZero().head<8>().cast<float>().transpose() * adTargetF[idx];
		}

	cDeltaF = HCalib->value_minus_value_zero.cast<float>(); // 相机内参增量

	for(EFFrame* f : frames)
	{
		f->delta = f->data->get_state_minus_stateZero().head<8>();  // 帧位姿增量
		f->delta_prior = (f->data->get_state() - f->data->getPriorZero()).head<8>(); // 先验增量

		for(EFPoint* p : f->points)
			p->deltaF = p->data->idepth - p->data->idepth_zero; // 逆深度的增量
	}

	EFDeltaValid = true;
}


// accumulates & shifts L.
//@ 计算能量方程内帧点构成的 正规方程
void EnergyFunctional::accumulateAF_MT(MatXX &H, VecX &b, bool MT)
{
	if(MT)
	{
		red->reduce(boost::bind(&AccumulatedTopHessianSSE::setZero, accSSE_top_A, nFrames,  _1, _2, _3, _4), 0, 0, 0);
		red->reduce(boost::bind(&AccumulatedTopHessianSSE::addPointsInternal<0>,
				accSSE_top_A, &allPoints, this,  _1, _2, _3, _4), 0, allPoints.size(), 50);
		// accSSE_top_A->stitchDoubleMT(red,H,b,this,false,true);
		accSSE_top_A->stitchDoubleMT(red,H,b,this,true,true);
		resInA = accSSE_top_A->nres[0];
	}
	else
	{
		accSSE_top_A->setZero(nFrames);
		for(EFFrame* f : frames)
			for(EFPoint* p : f->points)
				accSSE_top_A->addPoint<0>(p,this); //! mode 0 增加EF点
		// accSSE_top_A->stitchDoubleMT(red,H,b,this,false,false); // 不加先验, 得到H, b
		accSSE_top_A->stitchDoubleMT(red,H,b,this,true,false); // 加先验, 得到H, b
		resInA = accSSE_top_A->nres[0];  // 所有残差计数
	}
}

//@ 计算 H 和 b , 加先验, res是减去线性化残差
// accumulates & shifts L.
void EnergyFunctional::accumulateLF_MT(MatXX &H, VecX &b, bool MT)
{
	if(MT)
	{
		red->reduce(boost::bind(&AccumulatedTopHessianSSE::setZero, accSSE_top_L, nFrames,  _1, _2, _3, _4), 0, 0, 0);
		red->reduce(boost::bind(&AccumulatedTopHessianSSE::addPointsInternal<1>,
				accSSE_top_L, &allPoints, this,  _1, _2, _3, _4), 0, allPoints.size(), 50);
		accSSE_top_L->stitchDoubleMT(red,H,b,this,true,true);
		resInL = accSSE_top_L->nres[0];
	}
	else
	{
		accSSE_top_L->setZero(nFrames);
		for(EFFrame* f : frames)
			for(EFPoint* p : f->points)
				accSSE_top_L->addPoint<1>(p,this); //! mode 1
		accSSE_top_L->stitchDoubleMT(red,H,b,this,true,false);
		resInL = accSSE_top_L->nres[0];
	}
}




//@ 计算边缘化掉逆深度的Schur complement部分
void EnergyFunctional::accumulateSCF_MT(MatXX &H, VecX &b, bool MT)
{
	if(MT)
	{
		red->reduce(boost::bind(&AccumulatedSCHessianSSE::setZero, accSSE_bot, nFrames,  _1, _2, _3, _4), 0, 0, 0);
		red->reduce(boost::bind(&AccumulatedSCHessianSSE::addPointsInternal,
				accSSE_bot, &allPoints, true,  _1, _2, _3, _4), 0, allPoints.size(), 50);
		accSSE_bot->stitchDoubleMT(red,H,b,this,true);
	}
	else
	{
		accSSE_bot->setZero(nFrames);
		for(EFFrame* f : frames)
			for(EFPoint* p : f->points)
				accSSE_bot->addPoint(p, true);
		accSSE_bot->stitchDoubleMT(red, H, b,this,false);
	}
}

//@ 计算相机内参和位姿, 光度的增量
void EnergyFunctional::resubstituteF_MT(VecX x, CalibHessian* HCalib, bool MT)
{
	assert(x.size() == CPARS+nFrames*8);

	VecXf xF = x.cast<float>();
	HCalib->step = - x.head<CPARS>(); // 相机内参, 这次的增量

	Mat18f* xAd = new Mat18f[nFrames*nFrames];
	VecCf cstep = xF.head<CPARS>();

	for(EFFrame* h : frames)
	{
		h->data->step.head<8>() = - x.segment<8>(CPARS+8*h->idx); // 帧位姿和光度求解的增量
		h->data->step.tail<2>().setZero();

		//* 绝对位姿增量变相对的
		for(EFFrame* t : frames)
			xAd[nFrames*h->idx + t->idx] = xF.segment<8>(CPARS+8*h->idx).transpose() *   adHostF[h->idx+nFrames*t->idx]
			            + xF.segment<8>(CPARS+8*t->idx).transpose() * adTargetF[h->idx+nFrames*t->idx];
	}

	//* 计算点的逆深度增量
	if(MT)
		red->reduce(boost::bind(&EnergyFunctional::resubstituteFPt,
						this, cstep, xAd,  _1, _2, _3, _4), 0, allPoints.size(), 50);
	else
		resubstituteFPt(cstep, xAd, 0, allPoints.size(), 0,0);

	delete[] xAd;
}

//@ 计算点逆深度的增量
void EnergyFunctional::resubstituteFPt(
        const VecCf &xc, Mat18f* xAd, int min, int max, Vec10* stats, int tid)
{
	for(int k=min;k<max;k++)
	{
		EFPoint* p = allPoints[k];

		int ngoodres = 0;
		for(EFResidual* r : p->residualsAll) if(r->isActive()) ngoodres++;
		if(ngoodres==0)
		{
			p->data->step = 0;
			continue;
		}
		float b = p->bdSumF;
		// b -= xc.dot(p->Hcd_accAF + p->Hcd_accLF); //* 减去逆深度和内参
		b -= xc.dot(p->Hcd_accAF); //* 减去逆深度和内参

		for(EFResidual* r : p->residualsAll)
		{
			if(!r->isActive()) continue;
			//* 减去逆深度和位姿 光度参数
			b -= xAd[r->hostIDX*nFrames + r->targetIDX] * r->JpJdF; //! 绝对变相对的, xAd是转置了的
		}

		p->data->step = - b*p->HdiF; // 逆深度的增量
		assert(std::isfinite(p->data->step));
	}
}

//@ 也是求能量, 使用HM和bM求的, delta是绝对的

double EnergyFunctional::calcMEnergyF()
{

	assert(EFDeltaValid);
	assert(EFAdjointsValid);
	assert(EFIndicesValid);

	VecX delta = getStitchedDeltaF();
	return delta.dot(2*bM + HM*delta);
}

//@ 计算所有点的能量E之和, delta是相对的
void EnergyFunctional::calcLEnergyPt(int min, int max, Vec10* stats, int tid)
{

	Accumulator11 E;
	E.initialize();
	VecCf dc = cDeltaF;

	for(int i=min;i<max;i++)
	{
		EFPoint* p = allPoints[i];
		float dd = p->deltaF;

		for(EFResidual* r : p->residualsAll)
		{
			if(!r->isLinearized || !r->isActive()) continue; // 同时满足

			Mat18f dp = adHTdeltaF[r->hostIDX+nFrames*r->targetIDX];
			RawResidualJacobian* rJ = r->J;



			// compute Jp*delta
			float Jp_delta_x_1 =  rJ->Jpdxi[0].dot(dp.head<6>())
						   +rJ->Jpdc[0].dot(dc)
						   +rJ->Jpdd[0]*dd;

			float Jp_delta_y_1 =  rJ->Jpdxi[1].dot(dp.head<6>())
						   +rJ->Jpdc[1].dot(dc)
						   +rJ->Jpdd[1]*dd;

			__m128 Jp_delta_x = _mm_set1_ps(Jp_delta_x_1);
			__m128 Jp_delta_y = _mm_set1_ps(Jp_delta_y_1);
			__m128 delta_a = _mm_set1_ps((float)(dp[6]));
			__m128 delta_b = _mm_set1_ps((float)(dp[7]));

			for(int i=0;i+3<patternNum;i+=4)
			{
				
				//! PATTERN: E = (2*resb_toZeroF + J*delta) * J*delta.
				//! E = (f(x0)+J*dx)^2 = dx*H*dx + 2*J*dx*f(x0) + f(x0)^2 丢掉常数 f(x0)^2
				__m128 Jdelta =            _mm_mul_ps(_mm_load_ps(((float*)(rJ->JIdx))+i),Jp_delta_x);
				Jdelta = _mm_add_ps(Jdelta,_mm_mul_ps(_mm_load_ps(((float*)(rJ->JIdx+1))+i),Jp_delta_y));
				Jdelta = _mm_add_ps(Jdelta,_mm_mul_ps(_mm_load_ps(((float*)(rJ->JabF))+i),delta_a));
				Jdelta = _mm_add_ps(Jdelta,_mm_mul_ps(_mm_load_ps(((float*)(rJ->JabF+1))+i),delta_b));

				__m128 r0 = _mm_load_ps(((float*)&r->res_toZeroF)+i);
				r0 = _mm_add_ps(r0,r0);
				r0 = _mm_add_ps(r0,Jdelta);
				Jdelta = _mm_mul_ps(Jdelta,r0);
				E.updateSSENoShift(Jdelta); // 累加
			}
			// 128位对齐, 多出来部分
			for(int i=((patternNum>>2)<<2); i < patternNum; i++)  //* %4 的余数
			{
				float Jdelta = rJ->JIdx[0][i]*Jp_delta_x_1 + rJ->JIdx[1][i]*Jp_delta_y_1 +
								rJ->JabF[0][i]*dp[6] + rJ->JabF[1][i]*dp[7];
				E.updateSingleNoShift((float)(Jdelta * (Jdelta + 2*r->res_toZeroF[i])));
			}
		}
		E.updateSingle(p->deltaF*p->deltaF*p->priorF); // 逆深度先验
	}
	E.finish();
	(*stats)[0] += E.A;
}



//@ MT是多线程, 计算能量, 包括 先验 + 点残差平方
double EnergyFunctional::calcLEnergyF_MT()
{
	assert(EFDeltaValid);
	assert(EFAdjointsValid);
	assert(EFIndicesValid);

	double E = 0;
	//! 先验的能量 (x-x_prior)^T * ∑ * (x-x_prior)
	//* 因为 f->prior 是hessian的对角线, 使用向量表示, 所以使用cwiseProduct进行逐个相乘
	for(EFFrame* f : frames)
        E += f->delta_prior.cwiseProduct(f->prior).dot(f->delta_prior); // 位姿先验

	E += cDeltaF.cwiseProduct(cPriorF).dot(cDeltaF); // 相机内参先验

	red->reduce(boost::bind(&EnergyFunctional::calcLEnergyPt,
			this, _1, _2, _3, _4), 0, allPoints.size(), 50);

	return E+red->stats[0];
}


//@ 向能量函数中插入一残差, 更新连接图关系
EFResidual* EnergyFunctional::insertResidual(PointFrameResidual* r)
{
	EFResidual* efr = new EFResidual(r, r->point->efPoint, r->host->efFrame, r->target->efFrame);
	efr->idxInAll = r->point->efPoint->residualsAll.size(); // 在这个点的所有残差的id
	r->point->efPoint->residualsAll.push_back(efr); // 这个点的所有残差

	// 两帧之间的res计数加一
    connectivityMap[(((uint64_t)efr->host->frameID) << 32) + ((uint64_t)efr->target->frameID)][0]++;

	nResiduals++;
	r->efResidual = efr;
	return efr;
}

//@ 向能量函数中增加一帧, 进行的操作: 改变正规方程, 重新排ID, 共视关系
EFFrame* EnergyFunctional::insertFrame(FrameHessian* fh, CalibHessian* Hcalib)
{
	// 建立优化用的能量函数帧. 并加进能量函数frames中
	EFFrame* eff = new EFFrame(fh);
	eff->idx = frames.size(); 
	frames.push_back(eff);

	nFrames++;
	fh->efFrame = eff; // FrameHessian 指向能量函数帧

	assert(HM.cols() == 8*nFrames+CPARS-8);  // 边缘化掉一帧, 缺8个
	// 一个帧8个参数 + 相机内参
	bM.conservativeResize(8*nFrames+CPARS);
	HM.conservativeResize(8*nFrames+CPARS,8*nFrames+CPARS);
	// 新帧的块为0
	bM.tail<8>().setZero();
	HM.rightCols<8>().setZero();
	HM.bottomRows<8>().setZero();

	EFIndicesValid = false;
	EFAdjointsValid=false;
	EFDeltaValid=false;

	setAdjointsF(Hcalib); 	// 设置伴随矩阵
	makeIDX();				// 设置ID


	for(EFFrame* fh2 : frames)
	{
		// 前32位是host帧的历史ID, 后32位是Target的历史ID
        connectivityMap[(((uint64_t)eff->frameID) << 32) + ((uint64_t)fh2->frameID)] = Eigen::Vector2i(0,0);
		if(fh2 != eff)
            connectivityMap[(((uint64_t)fh2->frameID) << 32) + ((uint64_t)eff->frameID)] = Eigen::Vector2i(0,0);
	}

	return eff;
}

//@ 向能量函数中插入一个点, 放入对应的EFframe
EFPoint* EnergyFunctional::insertPoint(PointHessian* ph)
{
	EFPoint* efp = new EFPoint(ph, ph->host->efFrame);
	efp->idxInPoints = ph->host->efFrame->points.size();
	ph->host->efFrame->points.push_back(efp);

	nPoints++;
	ph->efPoint = efp;

	EFIndicesValid = false; // 有插入需要重新梳理残差的ID

	return efp;
}

//@ 丢掉一个residual, 并更新关系
void EnergyFunctional::dropResidual(EFResidual* r)
{
	EFPoint* p = r->point;
	assert(r == p->residualsAll[r->idxInAll]);

	p->residualsAll[r->idxInAll] = p->residualsAll.back(); 	// 最后一个给当前的
	p->residualsAll[r->idxInAll]->idxInAll = r->idxInAll;	// 当前的id变成现在位置的
	p->residualsAll.pop_back();		// 弹出最有一个

	// 计数
	if(r->isActive())
		r->host->data->shell->statistics_goodResOnThis++;
	else
		r->host->data->shell->statistics_outlierResOnThis++;

	// residual关键减一
    connectivityMap[(((uint64_t)r->host->frameID) << 32) + ((uint64_t)r->target->frameID)][0]--;
	nResiduals--;
	r->data->efResidual=0; // pointframehessian指向该残差的指针
	delete r;
}

//@ 边缘化掉一帧 fh
void EnergyFunctional::marginalizeFrame(EFFrame* fh)
{

	assert(EFDeltaValid);
	assert(EFAdjointsValid);
	assert(EFIndicesValid);

	assert((int)fh->points.size()==0);
	int ndim = nFrames*8+CPARS-8;// new dimension
	int odim = nFrames*8+CPARS;// old dimension


	//	VecX eigenvaluesPre = HM.eigenvalues().real();
	//	std::sort(eigenvaluesPre.data(), eigenvaluesPre.data()+eigenvaluesPre.size());
	//


//[ ***step 1*** ] 把边缘化的帧挪到最右边, 最下边
	//* HM bM就是边缘化点得到的
	if((int)fh->idx != (int)frames.size()-1)
	{
		int io = fh->idx*8+CPARS;	// index of frame to move to end
		int ntail = 8*(nFrames-fh->idx-1); // 边缘化帧后面的变量数
		assert((io+8+ntail) == nFrames*8+CPARS);

		Vec8 bTmp = bM.segment<8>(io); // 被边缘化的8个变量
		VecX tailTMP = bM.tail(ntail); // 后面的挪到前面
		bM.segment(io,ntail) = tailTMP;
		bM.tail<8>() = bTmp;

		//* 边缘化帧右侧挪前面
		MatXX HtmpCol = HM.block(0,io,odim,8);
		MatXX rightColsTmp = HM.rightCols(ntail);
		HM.block(0,io,odim,ntail) = rightColsTmp;
		HM.rightCols(8) = HtmpCol;
		//* 边缘化帧下边挪上面
		MatXX HtmpRow = HM.block(io,0,8,odim);
		MatXX botRowsTmp = HM.bottomRows(ntail);
		HM.block(io,0,ntail,odim) = botRowsTmp;
		HM.bottomRows(8) = HtmpRow;
	}

//[ ***step 2*** ] 加上先验
	//* 如果是初始化得到的帧有先验, 边缘化时需要加上. 光度也有先验
	// marginalize. First add prior here, instead of to active.
    HM.bottomRightCorner<8,8>().diagonal() += fh->prior;
    bM.tail<8>() += fh->prior.cwiseProduct(fh->delta_prior);



	//	std::cout << std::setprecision(16) << "HMPre:\n" << HM << "\n\n";

//[ ***step 3*** ] 先scaled 然后计算Schur complement
	VecX SVec = (HM.diagonal().cwiseAbs()+VecX::Constant(HM.cols(), 10)).cwiseSqrt();
	VecX SVecI = SVec.cwiseInverse();


	//	std::cout << std::setprecision(16) << "SVec: " << SVec.transpose() << "\n\n";
	//	std::cout << std::setprecision(16) << "SVecI: " << SVecI.transpose() << "\n\n";

	// scale!
	MatXX HMScaled = SVecI.asDiagonal() * HM * SVecI.asDiagonal();
	VecX bMScaled =  SVecI.asDiagonal() * bM;

	// invert bottom part!
	Mat88 hpi = HMScaled.bottomRightCorner<8,8>();
	hpi = 0.5f*(hpi+hpi);
	hpi = hpi.inverse();
	hpi = 0.5f*(hpi+hpi);

	// schur-complement!
	MatXX bli = HMScaled.bottomLeftCorner(8,ndim).transpose() * hpi;
	HMScaled.topLeftCorner(ndim,ndim).noalias() -= bli * HMScaled.bottomLeftCorner(8,ndim);
	bMScaled.head(ndim).noalias() -= bli*bMScaled.tail<8>();

	//unscale!
	HMScaled = SVec.asDiagonal() * HMScaled * SVec.asDiagonal();
	bMScaled = SVec.asDiagonal() * bMScaled;

	// set.
	HM = 0.5*(HMScaled.topLeftCorner(ndim,ndim) + HMScaled.topLeftCorner(ndim,ndim).transpose());
	bM = bMScaled.head(ndim);

//[ ***step 4*** ] 改变EFFrame的ID编号, 并删除
	// remove from vector, without changing the order!
	for(unsigned int i=fh->idx; i+1<frames.size();i++)
	{
		frames[i] = frames[i+1];
		frames[i]->idx = i;
	}
	frames.pop_back();
	nFrames--;
	fh->data->efFrame=0;

	assert((int)frames.size()*8+CPARS == (int)HM.rows());
	assert((int)frames.size()*8+CPARS == (int)HM.cols());
	assert((int)frames.size()*8+CPARS == (int)bM.size());
	assert((int)frames.size() == (int)nFrames);




	//	VecX eigenvaluesPost = HM.eigenvalues().real();
	//	std::sort(eigenvaluesPost.data(), eigenvaluesPost.data()+eigenvaluesPost.size());

	//	std::cout << std::setprecision(16) << "HMPost:\n" << HM << "\n\n";

	//	std::cout << "EigPre:: " << eigenvaluesPre.transpose() << "\n";
	//	std::cout << "EigPost: " << eigenvaluesPost.transpose() << "\n";
	
	EFIndicesValid = false;
	EFAdjointsValid=false;
	EFDeltaValid=false;

	makeIDX();
	delete fh;
}



//@ 边缘化掉一个点
void EnergyFunctional::marginalizePointsF()
{
	assert(EFDeltaValid);
	assert(EFAdjointsValid);
	assert(EFIndicesValid);

//[ ***step 1*** ] 记录被边缘化的点
	allPointsToMarg.clear();
	for(EFFrame* f : frames)
	{
		for(int i=0;i<(int)f->points.size();i++)
		{
			EFPoint* p = f->points[i];
			if(p->stateFlag == EFPointStatus::PS_MARGINALIZE)
			{
				p->priorF *= setting_idepthFixPriorMargFac; //? 这是干啥 ???
				for(EFResidual* r : p->residualsAll)
					if(r->isActive()) // 边缘化残差计数
                        connectivityMap[(((uint64_t)r->host->frameID) << 32) + ((uint64_t)r->target->frameID)][1]++;
				allPointsToMarg.push_back(p);
			}
		}
	}

//[ ***step 2*** ] 计算该点相连的残差构成的H, b, HSC, bSC
	accSSE_bot->setZero(nFrames);
	accSSE_top_A->setZero(nFrames);
	for(EFPoint* p : allPointsToMarg)
	{
		accSSE_top_A->addPoint<2>(p,this); // 这个点的残差, 计算 H b
		accSSE_bot->addPoint(p,false);	// 边缘化部分
		removePoint(p);
	}
	MatXX M, Msc;
	VecX Mb, Mbsc;
	accSSE_top_A->stitchDouble(M,Mb,this,false,false); // 不加先验, 在后面加了
	accSSE_bot->stitchDouble(Msc,Mbsc,this);

	resInM+= accSSE_top_A->nres[0];

	MatXX H =  M-Msc;
    VecX b =  Mb-Mbsc;

//[ ***step 3*** ] 处理零空间
	// 减去零空间部分
	if(setting_solverMode & SOLVER_ORTHOGONALIZE_POINTMARG)
	{
		// have a look if prior is there.
		bool haveFirstFrame = false;
		for(EFFrame* f : frames) if(f->frameID==0) haveFirstFrame=true;

		if(!haveFirstFrame)
			orthogonalize(&b, &H);

	}
	
	//! 给边缘化的量加了个权重，不准确的线性化
	HM += setting_margWeightFac*H;  //* 所以边缘化的部分直接加在HM bM了
	bM += setting_margWeightFac*b;

	if(setting_solverMode & SOLVER_ORTHOGONALIZE_FULL)
		orthogonalize(&bM, &HM);

	EFIndicesValid = false;
	makeIDX(); // 梳理ID
}

//@ 直接丢掉点, 不边缘化
void EnergyFunctional::dropPointsF()
{


	for(EFFrame* f : frames)
	{
		for(int i=0;i<(int)f->points.size();i++)
		{
			EFPoint* p = f->points[i];
			if(p->stateFlag == EFPointStatus::PS_DROP)
			{
				removePoint(p);
				i--;
			}
		}
	}

	EFIndicesValid = false;
	makeIDX();
}

//@ 从EFFrame中移除一个点p
void EnergyFunctional::removePoint(EFPoint* p)
{
	for(EFResidual* r : p->residualsAll)
		dropResidual(r); // 丢掉改点的所有残差

	EFFrame* h = p->host;
	h->points[p->idxInPoints] = h->points.back();
	h->points[p->idxInPoints]->idxInPoints = p->idxInPoints;
	h->points.pop_back();

	nPoints--;
	p->data->efPoint = 0;

	EFIndicesValid = false;

	delete p;
}

//@ 计算零空间矩阵伪逆, 从 H 和 b 中减去零空间, 相当于设相应的Jacob为0
void EnergyFunctional::orthogonalize(VecX* b, MatXX* H)
{
	//	VecX eigenvaluesPre = H.eigenvalues().real();
	//	std::sort(eigenvaluesPre.data(), eigenvaluesPre.data()+eigenvaluesPre.size());
	//	std::cout << "EigPre:: " << eigenvaluesPre.transpose() << "\n";
	

	// decide to which nullspaces to orthogonalize.
	std::vector<VecX> ns;
	ns.insert(ns.end(), lastNullspaces_pose.begin(), lastNullspaces_pose.end());
	ns.insert(ns.end(), lastNullspaces_scale.begin(), lastNullspaces_scale.end());
	//	if(setting_affineOptModeA <= 0)
	//		ns.insert(ns.end(), lastNullspaces_affA.begin(), lastNullspaces_affA.end());
	//	if(setting_affineOptModeB <= 0)
	//		ns.insert(ns.end(), lastNullspaces_affB.begin(), lastNullspaces_affB.end());


	// make Nullspaces matrix
	//! 7自由度不可观
	MatXX N(ns[0].rows(), ns.size());  //! size (4+8*n)×7
	for(unsigned int i=0;i<ns.size();i++)
		N.col(i) = ns[i].normalized();


	//* 求伪逆
	// compute Npi := N * (N' * N)^-1 = pseudo inverse of N.
	Eigen::JacobiSVD<MatXX> svdNN(N, Eigen::ComputeThinU | Eigen::ComputeThinV);

	VecX SNN = svdNN.singularValues();
	double minSv = 1e10, maxSv = 0;
	for(int i=0;i<SNN.size();i++)
	{
		if(SNN[i] < minSv) minSv = SNN[i];
		if(SNN[i] > maxSv) maxSv = SNN[i];
	}
	// 比最大奇异值小setting_solverModeDelta(e-5)倍, 则认为是0
	for(int i=0;i<SNN.size();i++)
		{ if(SNN[i] > setting_solverModeDelta*maxSv) SNN[i] = 1.0 / SNN[i]; else SNN[i] = 0; } // 求逆

	MatXX Npi = svdNN.matrixU() * SNN.asDiagonal() * svdNN.matrixV().transpose(); 	// [dim] x 7.
	//! Npi.transpose()是N的伪逆
	MatXX NNpiT = N*Npi.transpose(); 	// [dim] x [dim].
	MatXX NNpiTS = 0.5*(NNpiT + NNpiT.transpose());	// = N * (N' * N)^-1 * N'.
	
//*****************add by gong********************
	// std::vector<VecX> ns;
	// ns.insert(ns.end(), lastNullspaces_pose.begin(), lastNullspaces_pose.end());
	// ns.insert(ns.end(), lastNullspaces_scale.begin(), lastNullspaces_scale.end());
	std::cout<<"//=====================Test null space start=====================/ "<<std::endl;
	// make Nullspaces matrix
	//! 7自由度不可观
	// MatXX N(ns[0].rows(), ns.size());  //! size (4+8*n)×7
	// for(unsigned int i=0;i<ns.size();i++)
	// 	N.col(i) = ns[i].normalized();

	VecX zero_x = *b;

	// MatXX zero = (lastHS) * zero_x;
	for(int i=0; i<zero_x.cols(); i++)
	{
		VecX xHx = 0.5 * zero_x.col(i).transpose() *  lastHS *  zero_x.col(i);
		VecX xb =  zero_x.col(i).transpose() * lastbS;

		std::cout<<"Before nullspace process "<< i<< " : " << xHx << " + "<< xb << std::endl;
	}
	
	// std::cout<<"//=====================Test null space start=====================/ "<<std::endl;
	// std::cout<<"HA_top * nullspace matrix = " << zero << std::endl;
	// std::cout<<"//=====================Test null space end=====================/ "<<std::endl;

	

	//TODO 为什么这么做?
	//* 把零空间从H和b中减去??? 以免乱飘?
	if(b!=0) *b -= NNpiTS * *b;
	if(H!=0) *H -= NNpiTS * *H * NNpiTS;

	zero_x = *b;
	for(int i=0; i<zero_x.cols(); i++)
	{
		VecX xHx = 0.5 * zero_x.col(i).transpose() *  lastHS *  zero_x.col(i);
		VecX xb =  zero_x.col(i).transpose() * lastbS;

		std::cout<<"After nullspace process "<< i<< " : " << xHx << " + "<< xb << std::endl;
	}
	std::cout<<"//=====================Test null space end=====================/ "<<std::endl;

	//	std::cout << std::setprecision(16) << "Orth SV: " << SNN.reverse().transpose() << "\n";

	//	VecX eigenvaluesPost = H.eigenvalues().real();
	//	std::sort(eigenvaluesPost.data(), eigenvaluesPost.data()+eigenvaluesPost.size());
	//	std::cout << "EigPost:: " << eigenvaluesPost.transpose() << "\n";

}

//@ 计算正规方程, 并求解
void EnergyFunctional::solveSystemF(int iteration, double lambda, CalibHessian* HCalib)
{
	if(setting_solverMode & SOLVER_USE_GN) lambda=0;			// 不同的位控制不同的模式
	if(setting_solverMode & SOLVER_FIX_LAMBDA) lambda = 1e-5;   //! 还真他娘的用的GN, 只是一个小阻尼

	assert(EFDeltaValid);
	assert(EFAdjointsValid);
	assert(EFIndicesValid);
//[ ***step 1*** ] 先计算正规方程, 涉及边缘化, 先验, 舒尔补等
	MatXX HL_top, HA_top, H_sc;
	VecX  bL_top, bA_top, bM_top, b_sc;
	
	//* 针对新的残差, 使用的当前残差, 没有逆深度的部分
	accumulateAF_MT(HA_top, bA_top,multiThreading);

	//* 边缘化fix的残差, 有边缘化对的, 使用的res_toZeroF减去线性化部分, 加上先验, 没有逆深度的部分
	//bug: 这里根本就没有点参与了, 只有先验信息, 因为边缘化的和删除的点都不在了
	//! 这里唯一的作用就是 把 p相关的置零
	accumulateLF_MT(HL_top, bL_top,multiThreading);  // 计算的是之前计算过得
		// p->Hdd_accLF = 0;
		// p->bd_accLF = 0;
		// p->Hcd_accLF =0 ;



	//* 关于逆深度的Schur部分
	accumulateSCF_MT(H_sc, b_sc,multiThreading);

	//TODO HM 和 bM是啥啊
	//* 由于固定线性化点, 每次迭代更新残差
	bM_top = (bM+ HM * getStitchedDeltaF());

	// printf("HA_top: \n");
	// for(int i = 0; i < HA_top.rows(); i++)
	// {	
	// 	// for(int j = 0; j < HA_top.cols(); j++)
	// 	{
	// 		printf("  %f", HA_top(i,i));
	// 	}
	// 	// printf("; \n");
	// }

	// printf("HL_top: \n");
	// for(int i = 0; i < HL_top.rows(); i++)
	// {
	// 	// for(int j = 0; j < HL_top.cols(); j++)
	// 	{
	// 		printf("  %f", HL_top(i,i));
	// 	}
	// 	// printf("; \n");
	// }	


	MatXX HFinal_top;
	VecX bFinal_top;
//[ ***step 2*** ] 如果是设置求解正交系统, 则把相对应的零空间部分Jacobian设置为0, 否则正常计算schur
	if(setting_solverMode & SOLVER_ORTHOGONALIZE_SYSTEM)
	{
		// have a look if prior is there.
		bool haveFirstFrame = false;
		for(EFFrame* f : frames) if(f->frameID==0) haveFirstFrame=true;



		// 计算Schur之后的
		// MatXX HT_act =  HL_top + HA_top - H_sc;
		MatXX HT_act =  HA_top - H_sc;
		// VecX bT_act =   bL_top + bA_top - b_sc;
		VecX bT_act =   bA_top - b_sc;

		//! 包含第一帧则不减去零空间
		//! 不包含第一帧, 因为要固定第一帧, 和第一帧统一, 减去零空间, 防止在零空间乱飘
		if(!haveFirstFrame)
			orthogonalize(&bT_act, &HT_act);

		HFinal_top = HT_act + HM;
		bFinal_top = bT_act + bM_top;





		lastHS = HFinal_top;
		lastbS = bFinal_top;
		// LM
		//* 这个阻尼也是加在Schur complement计算之后的
		for(int i=0;i<8*nFrames+CPARS;i++) HFinal_top(i,i) *= (1+lambda);

	}
	else
	{


		// HFinal_top = HL_top + HM + HA_top;
		HFinal_top = HM + HA_top;
		// bFinal_top = bL_top + bM_top + bA_top - b_sc;
		bFinal_top = bM_top + bA_top - b_sc;

		lastHS = HFinal_top - H_sc;
		lastbS = bFinal_top;

		
		//* 而这个就是阻尼加在了整个Hessian上
		//? 为什么呢, 是因为减去了零空间么  ??
		for(int i=0;i<8*nFrames+CPARS;i++) HFinal_top(i,i) *= (1+lambda);
		HFinal_top -= H_sc * (1.0f/(1+lambda)); // 因为Schur里面有个对角线的逆, 所以是倒数
	}




//[ ***step 3*** ] 使用SVD求解, 或者ldlt直接求解
	VecX x;
	if(setting_solverMode & SOLVER_SVD)
	{
		//* 为数值稳定进行缩放
		VecX SVecI = HFinal_top.diagonal().cwiseSqrt().cwiseInverse();
		MatXX HFinalScaled = SVecI.asDiagonal() * HFinal_top * SVecI.asDiagonal();
		VecX bFinalScaled  = SVecI.asDiagonal() * bFinal_top;
		//! Hx=b --->  U∑V^T*x = b
		Eigen::JacobiSVD<MatXX> svd(HFinalScaled, Eigen::ComputeThinU | Eigen::ComputeThinV);

		VecX S = svd.singularValues(); // 奇异值
		double minSv = 1e10, maxSv = 0;
		for(int i=0;i<S.size();i++)
		{
			if(S[i] < minSv) minSv = S[i];
			if(S[i] > maxSv) maxSv = S[i];
		}

		//! Hx=b --->  U∑V^T*x = b  --->  ∑V^T*x = U^T*b
		VecX Ub = svd.matrixU().transpose()*bFinalScaled;
		int setZero=0;
		for(int i=0;i<Ub.size();i++)
		{
			if(S[i] < setting_solverModeDelta*maxSv) //* 奇异值小的设置为0
			{ Ub[i] = 0; setZero++; }

			if((setting_solverMode & SOLVER_SVD_CUT7) && (i >= Ub.size()-7))  //* 留出7个不可观的, 零空间
			{ Ub[i] = 0; setZero++; }
			//! V^T*x = ∑^-1*U^T*b
			else Ub[i] /= S[i];
		}
		//! x = V*∑^-1*U^T*b   把scaled的乘回来
		x = SVecI.asDiagonal() * svd.matrixV() * Ub;

	}
	else
	{
		VecX SVecI = (HFinal_top.diagonal()+VecX::Constant(HFinal_top.cols(), 10)).cwiseSqrt().cwiseInverse();
		MatXX HFinalScaled = SVecI.asDiagonal() * HFinal_top * SVecI.asDiagonal();
		x = SVecI.asDiagonal() * HFinalScaled.ldlt().solve(SVecI.asDiagonal() * bFinal_top);//  SVec.asDiagonal() * svd.matrixV() * Ub;
	}

//[ ***step 4*** ] 如果设置的是直接对解进行处理, 直接去掉解x中的零空间
	if((setting_solverMode & SOLVER_ORTHOGONALIZE_X) || (iteration >= 2 && (setting_solverMode & SOLVER_ORTHOGONALIZE_X_LATER)))
	{
		VecX xOld = x;
		orthogonalize(&x, 0);
		// //********************* check nullspace added by gong ***********************
		// VecX new_b = HA_top * x;
		// VecX old_b = HA_top * xOld;
		// std::cout<<"//=====================Test null space start=====================/ "<<std::endl;
		// std::cout<<"new_b - old_b: "<< (new_b - old_b).transpose() << std::endl;
		// // xHx
		// std::cout<<"//=====================Test null space end=====================/ "<<std::endl;
	}


	lastX = x;

//[ ***step 5*** ] 分别求出各个待求量的增量值
	//resubstituteF(x, HCalib);
	currentLambda= lambda;
	resubstituteF_MT(x, HCalib,multiThreading);
	currentLambda=0;


}

//@ 设置EFFrame, EFPoint, EFResidual对应的 ID 号
void EnergyFunctional::makeIDX()
{
	// 重新赋值ID
	for(unsigned int idx=0;idx<frames.size();idx++)
		frames[idx]->idx = idx;

	allPoints.clear();

	for(EFFrame* f : frames)
		for(EFPoint* p : f->points) 
		{
			allPoints.push_back(p);
			// 残差的ID号
			for(EFResidual* r : p->residualsAll)
			{
				r->hostIDX = r->host->idx;  // EFFrame的idx
				r->targetIDX = r->target->idx;
			}
		}


	EFIndicesValid=true;
}

//@ 返回状态增量, 这里帧位姿和光度参数, 使用的是每一帧绝对的
VecX EnergyFunctional::getStitchedDeltaF() const
{
	VecX d = VecX(CPARS+nFrames*8); d.head<CPARS>() = cDeltaF.cast<double>(); // 相机内参增量
	for(int h=0;h<nFrames;h++) d.segment<8>(CPARS+8*h) = frames[h]->delta;  
	return d;
}



}
