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


#include "OptimizationBackend/AccumulatedTopHessian.h"
#include "OptimizationBackend/EnergyFunctional.h"
#include "OptimizationBackend/EnergyFunctionalStructs.h"
#include <iostream>

#if !defined(__SSE3__) && !defined(__SSE2__) && !defined(__SSE1__)
#include "SSE2NEON.h"
#endif

namespace dso
{


//@ 计算残差对应的 Hessian和Jres
template<int mode>
void AccumulatedTopHessianSSE::addPoint(EFPoint* p, EnergyFunctional const * const ef, int tid)	// 0 = active, 1 = linearized, 2=marginalize
{


	assert(mode==0 || mode==1 || mode==2);

	VecCf dc = ef->cDeltaF; // 内参
	float dd = p->deltaF;	// 逆深度

	float bd_acc=0;
	float Hdd_acc=0;
	VecCf  Hcd_acc = VecCf::Zero();

	for(EFResidual* r : p->residualsAll) // 对该点所有残差遍历一遍
	{
		//* 这个和运行的mode不一样, 又混了.....
		if(mode==0) // 只计算新加入的残差
		{
			if(r->isLinearized || !r->isActive()) continue;
		}
		if(mode==1)	//bug: 这个条件就一直满足 计算旧的残差, 之前计算过得
		{
			if(!r->isLinearized || !r->isActive()) continue;
		}
		if(mode==2) // 边缘化计算的情况
		{
			if(!r->isActive()) continue;
			assert(r->isLinearized);
		}
		// if(mode == 1)
		// {
		// 	printf("yeah I'm IN !");
		// }

		RawResidualJacobian* rJ = r->J; // 导数
		//* ID 来控制不同帧之间的变量, 区分出相同两帧 但是host target角色互换的
		int htIDX = r->hostIDX + r->targetIDX*nframes[tid]; 
		Mat18f dp = ef->adHTdeltaF[htIDX]; // 位姿+光度a b


		
		VecNRf resApprox;
		if(mode==0)
			resApprox = rJ->resF;
		if(mode==2) // 边缘化时使用的
			resApprox = r->res_toZeroF;
		if(mode==1)
		{
			//* 因为计算的是旧的, 由于更新需要重新计算
			// compute Jp*delta
			__m128 Jp_delta_x = _mm_set1_ps(rJ->Jpdxi[0].dot(dp.head<6>())+rJ->Jpdc[0].dot(dc)+rJ->Jpdd[0]*dd);
			__m128 Jp_delta_y = _mm_set1_ps(rJ->Jpdxi[1].dot(dp.head<6>())+rJ->Jpdc[1].dot(dc)+rJ->Jpdd[1]*dd);
			__m128 delta_a = _mm_set1_ps((float)(dp[6]));
			__m128 delta_b = _mm_set1_ps((float)(dp[7]));

			for(int i=0;i<patternNum;i+=4)
			{
				//! PATTERN: rtz = res_toZeroF - [JI*Jp Ja]*delta.
				//* 线性更新b值, 边缘化量, 每次在res_toZeroF上减
				__m128 rtz = _mm_load_ps(((float*)&r->res_toZeroF)+i);
				rtz = _mm_add_ps(rtz,_mm_mul_ps(_mm_load_ps(((float*)(rJ->JIdx))+i),Jp_delta_x));
				rtz = _mm_add_ps(rtz,_mm_mul_ps(_mm_load_ps(((float*)(rJ->JIdx+1))+i),Jp_delta_y));
				rtz = _mm_add_ps(rtz,_mm_mul_ps(_mm_load_ps(((float*)(rJ->JabF))+i),delta_a));
				rtz = _mm_add_ps(rtz,_mm_mul_ps(_mm_load_ps(((float*)(rJ->JabF+1))+i),delta_b));
				_mm_store_ps(((float*)&resApprox)+i, rtz);
			}
		}

		// need to compute JI^T * r, and Jab^T * r. (both are 2-vectors).
		Vec2f JI_r(0,0);
		Vec2f Jab_r(0,0);
		float rr=0;
		for(int i=0;i<patternNum;i++)
		{
			JI_r[0] += resApprox[i] *rJ->JIdx[0][i];
			JI_r[1] += resApprox[i] *rJ->JIdx[1][i];
			Jab_r[0] += resApprox[i] *rJ->JabF[0][i];
			Jab_r[1] += resApprox[i] *rJ->JabF[1][i];
			rr += resApprox[i]*resApprox[i];
		}

		//* 计算hessian 10*10矩阵, [位姿+相机参数]
		acc[tid][htIDX].update(
				rJ->Jpdc[0].data(), rJ->Jpdxi[0].data(),
				rJ->Jpdc[1].data(), rJ->Jpdxi[1].data(),
				rJ->JIdx2(0,0),rJ->JIdx2(0,1),rJ->JIdx2(1,1));
		//* 计算 3*3 矩阵, [光度a, 光度b, 残差r]
		acc[tid][htIDX].updateBotRight(
				rJ->Jab2(0,0), rJ->Jab2(0,1), Jab_r[0],
				rJ->Jab2(1,1), Jab_r[1],rr);
		//* 计算 10*3 矩阵, [位姿+相机参数]*[光度a, 光度b, 残差r]
		acc[tid][htIDX].updateTopRight(
				rJ->Jpdc[0].data(), rJ->Jpdxi[0].data(),
				rJ->Jpdc[1].data(), rJ->Jpdxi[1].data(),
				rJ->JabJIdx(0,0), rJ->JabJIdx(0,1),
				rJ->JabJIdx(1,0), rJ->JabJIdx(1,1),
				JI_r[0], JI_r[1]);


		Vec2f Ji2_Jpdd = rJ->JIdx2 * rJ->Jpdd;
		bd_acc +=  JI_r[0]*rJ->Jpdd[0] + JI_r[1]*rJ->Jpdd[1]; //* 残差*逆深度J
		Hdd_acc += Ji2_Jpdd.dot(rJ->Jpdd);	//* 光度对逆深度hessian
		Hcd_acc += rJ->Jpdc[0]*Ji2_Jpdd[0] + rJ->Jpdc[1]*Ji2_Jpdd[1]; //* 光度对内参J*光度对逆深度J

		nres[tid]++;
	}

	if(mode==0)
	{
		p->Hdd_accAF = Hdd_acc;
		p->bd_accAF = bd_acc;
		p->Hcd_accAF = Hcd_acc;
	}
	if(mode==1 || mode==2)
	{
		p->Hdd_accLF = Hdd_acc;
		p->bd_accLF = bd_acc;
		p->Hcd_accLF = Hcd_acc;
	}
	if(mode==2) // 边缘化掉, 设为0
	{
		p->Hcd_accAF.setZero();
		p->Hdd_accAF = 0;
		p->bd_accAF = 0;
	}

}
// 实例化
template void AccumulatedTopHessianSSE::addPoint<0>(EFPoint* p, EnergyFunctional const * const ef, int tid);
template void AccumulatedTopHessianSSE::addPoint<1>(EFPoint* p, EnergyFunctional const * const ef, int tid);
template void AccumulatedTopHessianSSE::addPoint<2>(EFPoint* p, EnergyFunctional const * const ef, int tid);







//@ 对某一个线程进行的 H 和 b 计算, 或者是没有使用多线程
void AccumulatedTopHessianSSE::stitchDouble(MatXX &H, VecX &b, EnergyFunctional const * const EF, bool usePrior, bool useDelta, int tid)
{
	H = MatXX::Zero(nframes[tid]*8+CPARS, nframes[tid]*8+CPARS);
	b = VecX::Zero(nframes[tid]*8+CPARS);


	for(int h=0;h<nframes[tid];h++)
		for(int t=0;t<nframes[tid];t++)
		{
			int hIdx = CPARS+h*8;
			int tIdx = CPARS+t*8;
			int aidx = h+nframes[tid]*t;



			acc[tid][aidx].finish();
			if(acc[tid][aidx].num==0) continue;

			MatPCPC accH = acc[tid][aidx].H.cast<double>();


			H.block<8,8>(hIdx, hIdx).noalias() += EF->adHost[aidx] * accH.block<8,8>(CPARS,CPARS) * EF->adHost[aidx].transpose();

			H.block<8,8>(tIdx, tIdx).noalias() += EF->adTarget[aidx] * accH.block<8,8>(CPARS,CPARS) * EF->adTarget[aidx].transpose();

			H.block<8,8>(hIdx, tIdx).noalias() += EF->adHost[aidx] * accH.block<8,8>(CPARS,CPARS) * EF->adTarget[aidx].transpose();

			H.block<8,CPARS>(hIdx,0).noalias() += EF->adHost[aidx] * accH.block<8,CPARS>(CPARS,0);

			H.block<8,CPARS>(tIdx,0).noalias() += EF->adTarget[aidx] * accH.block<8,CPARS>(CPARS,0);

			H.topLeftCorner<CPARS,CPARS>().noalias() += accH.block<CPARS,CPARS>(0,0);

			b.segment<8>(hIdx).noalias() += EF->adHost[aidx] * accH.block<8,1>(CPARS,8+CPARS);

			b.segment<8>(tIdx).noalias() += EF->adTarget[aidx] * accH.block<8,1>(CPARS,8+CPARS);

			b.head<CPARS>().noalias() += accH.block<CPARS,1>(0,8+CPARS);
		}


	// ----- new: copy transposed parts.
	for(int h=0;h<nframes[tid];h++)
	{
		int hIdx = CPARS+h*8;
		H.block<CPARS,8>(0,hIdx).noalias() = H.block<8,CPARS>(hIdx,0).transpose();

		for(int t=h+1;t<nframes[tid];t++)
		{
			int tIdx = CPARS+t*8;
			H.block<8,8>(hIdx, tIdx).noalias() += H.block<8,8>(tIdx, hIdx).transpose();
			H.block<8,8>(tIdx, hIdx).noalias() = H.block<8,8>(hIdx, tIdx).transpose();
		}
	}


	if(usePrior)
	{
		assert(useDelta);
		H.diagonal().head<CPARS>() += EF->cPrior;
		b.head<CPARS>() += EF->cPrior.cwiseProduct(EF->cDeltaF.cast<double>());
		for(int h=0;h<nframes[tid];h++)
		{
            H.diagonal().segment<8>(CPARS+h*8) += EF->frames[h]->prior;
            b.segment<8>(CPARS+h*8) += EF->frames[h]->prior.cwiseProduct(EF->frames[h]->delta_prior);
		}
	}
}

//@ 构造Hessian矩阵, b=Jres矩阵
void AccumulatedTopHessianSSE::stitchDoubleInternal(
		MatXX* H, VecX* b, EnergyFunctional const * const EF, bool usePrior,
		int min, int max, Vec10* stats, int tid)
{
	int toAggregate = NUM_THREADS;
	// 不用多线程, 为啥不能统一一下
	if(tid == -1) { toAggregate = 1; tid = 0; }	// special case: if we dont do multithreading, dont aggregate.
	if(min==max) return;


	for(int k=min;k<max;k++) // 帧的范围 最大nframes[0]*nframes[0]
	{
		int h = k%nframes[0]; // 和两个循环一样
		int t = k/nframes[0];

		int hIdx = CPARS+h*8; // 起始元素id
		int tIdx = CPARS+t*8;
		int aidx = h+nframes[0]*t; // 总的id

		assert(aidx == k);

		MatPCPC accH = MatPCPC::Zero(); // (8+4+1)*(8+4+1)矩阵

		for(int tid2=0;tid2 < toAggregate;tid2++)
		{
			acc[tid2][aidx].finish();
			if(acc[tid2][aidx].num==0) continue;
			accH += acc[tid2][aidx].H.cast<double>(); // 不同线程之间的加起来
		}
		//* 相对的量通过adj变成绝对的量, 并累加到 H, b 中
		H[tid].block<8,8>(hIdx, hIdx).noalias() += EF->adHost[aidx] * accH.block<8,8>(CPARS,CPARS) * EF->adHost[aidx].transpose();

		H[tid].block<8,8>(tIdx, tIdx).noalias() += EF->adTarget[aidx] * accH.block<8,8>(CPARS,CPARS) * EF->adTarget[aidx].transpose();

		H[tid].block<8,8>(hIdx, tIdx).noalias() += EF->adHost[aidx] * accH.block<8,8>(CPARS,CPARS) * EF->adTarget[aidx].transpose();

		H[tid].block<8,CPARS>(hIdx,0).noalias() += EF->adHost[aidx] * accH.block<8,CPARS>(CPARS,0);

		H[tid].block<8,CPARS>(tIdx,0).noalias() += EF->adTarget[aidx] * accH.block<8,CPARS>(CPARS,0);

		H[tid].topLeftCorner<CPARS,CPARS>().noalias() += accH.block<CPARS,CPARS>(0,0);

		b[tid].segment<8>(hIdx).noalias() += EF->adHost[aidx] * accH.block<8,1>(CPARS,CPARS+8);

		b[tid].segment<8>(tIdx).noalias() += EF->adTarget[aidx] * accH.block<8,1>(CPARS,CPARS+8);

		b[tid].head<CPARS>().noalias() += accH.block<CPARS,1>(0,CPARS+8);  //! 残差 * 内参

	}

	// only do this on one thread.
	if(min==0 && usePrior)
	{
		H[tid].diagonal().head<CPARS>() += EF->cPrior;	//! hessian先验
		b[tid].head<CPARS>() += EF->cPrior.cwiseProduct(EF->cDeltaF.cast<double>()); //! H*delta 更新残差
		for(int h=0;h<nframes[tid];h++)
		{
            H[tid].diagonal().segment<8>(CPARS+h*8) += EF->frames[h]->prior;	//! hessian先验
            b[tid].segment<8>(CPARS+h*8) += EF->frames[h]->prior.cwiseProduct(EF->frames[h]->delta_prior); 
		}
	}
}



}


