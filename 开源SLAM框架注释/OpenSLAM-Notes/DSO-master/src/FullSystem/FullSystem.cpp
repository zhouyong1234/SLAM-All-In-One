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


/*
 * KFBuffer.cpp
 *
 *  Created on: Jan 7, 2014
 *      Author: engelj
 */

#include "FullSystem/FullSystem.h"
 
#include "stdio.h"
#include "util/globalFuncs.h"
#include <Eigen/LU>
#include <algorithm>
#include "IOWrapper/ImageDisplay.h"
#include "util/globalCalib.h"
#include <Eigen/SVD>
#include <Eigen/Eigenvalues>
#include "FullSystem/PixelSelector.h"
#include "FullSystem/PixelSelector2.h"
#include "FullSystem/ResidualProjections.h"
#include "FullSystem/ImmaturePoint.h"

#include "FullSystem/CoarseTracker.h"
#include "FullSystem/CoarseInitializer.h"

#include "OptimizationBackend/EnergyFunctional.h"
#include "OptimizationBackend/EnergyFunctionalStructs.h"

#include "IOWrapper/Output3DWrapper.h"

#include "util/ImageAndExposure.h"

#include <cmath>

namespace dso
{
// Hessian矩阵计数, 有点像 shared_ptr
int FrameHessian::instanceCounter=0;
int PointHessian::instanceCounter=0;
int CalibHessian::instanceCounter=0;


/********************************
 * @ function: 构造函数
 * 
 * @ param: 
 * 
 * @ note:
 *******************************/
FullSystem::FullSystem()
{

	int retstat =0;
	if(setting_logStuff)
	{
		//shell命令删除旧的文件夹, 创建新的
		retstat += system("rm -rf logs");
		retstat += system("mkdir logs");

		retstat += system("rm -rf mats");
		retstat += system("mkdir mats");

		// 打开读写log文件
		calibLog = new std::ofstream();
		calibLog->open("logs/calibLog.txt", std::ios::trunc | std::ios::out);
		calibLog->precision(12);

		numsLog = new std::ofstream();
		numsLog->open("logs/numsLog.txt", std::ios::trunc | std::ios::out);
		numsLog->precision(10);

		coarseTrackingLog = new std::ofstream();
		coarseTrackingLog->open("logs/coarseTrackingLog.txt", std::ios::trunc | std::ios::out);
		coarseTrackingLog->precision(10);

		eigenAllLog = new std::ofstream();
		eigenAllLog->open("logs/eigenAllLog.txt", std::ios::trunc | std::ios::out);
		eigenAllLog->precision(10);

		eigenPLog = new std::ofstream();
		eigenPLog->open("logs/eigenPLog.txt", std::ios::trunc | std::ios::out);
		eigenPLog->precision(10);

		eigenALog = new std::ofstream();
		eigenALog->open("logs/eigenALog.txt", std::ios::trunc | std::ios::out);
		eigenALog->precision(10);

		DiagonalLog = new std::ofstream();
		DiagonalLog->open("logs/diagonal.txt", std::ios::trunc | std::ios::out);
		DiagonalLog->precision(10);

		variancesLog = new std::ofstream();
		variancesLog->open("logs/variancesLog.txt", std::ios::trunc | std::ios::out);
		variancesLog->precision(10);


		nullspacesLog = new std::ofstream();
		nullspacesLog->open("logs/nullspacesLog.txt", std::ios::trunc | std::ios::out);
		nullspacesLog->precision(10);
	}
	else
	{
		nullspacesLog=0;
		variancesLog=0;
		DiagonalLog=0;
		eigenALog=0;
		eigenPLog=0;
		eigenAllLog=0;
		numsLog=0;
		calibLog=0;
	}

	assert(retstat!=293847); // shell正常执行结束返回这么个值,填充8~15位bit, 有趣



	selectionMap = new float[wG[0]*hG[0]];

	coarseDistanceMap = new CoarseDistanceMap(wG[0], hG[0]);
	coarseTracker = new CoarseTracker(wG[0], hG[0]);
	coarseTracker_forNewKF = new CoarseTracker(wG[0], hG[0]);
	coarseInitializer = new CoarseInitializer(wG[0], hG[0]);
	pixelSelector = new PixelSelector(wG[0], hG[0]);

	statistics_lastNumOptIts=0;
	statistics_numDroppedPoints=0;
	statistics_numActivatedPoints=0;
	statistics_numCreatedPoints=0;
	statistics_numForceDroppedResBwd = 0;
	statistics_numForceDroppedResFwd = 0;
	statistics_numMargResFwd = 0;
	statistics_numMargResBwd = 0;

	lastCoarseRMSE.setConstant(100); //5维向量都=100

	currentMinActDist=2;
	initialized=false;


	ef = new EnergyFunctional();
	ef->red = &this->treadReduce;

	isLost=false;
	initFailed=false;


	needNewKFAfter = -1;

	linearizeOperation=true;
	runMapping=true;
	// 打开建图线程
	mappingThread = boost::thread(&FullSystem::mappingLoop, this); // 建图线程单开
	lastRefStopID=0;



	minIdJetVisDebug = -1;
	maxIdJetVisDebug = -1;
	minIdJetVisTracker = -1;
	maxIdJetVisTracker = -1;
}

FullSystem::~FullSystem()
{
	blockUntilMappingIsFinished();

	// 删除new的ofstream
	if(setting_logStuff)
	{
		calibLog->close(); delete calibLog;
		numsLog->close(); delete numsLog;
		coarseTrackingLog->close(); delete coarseTrackingLog;
		//errorsLog->close(); delete errorsLog;
		eigenAllLog->close(); delete eigenAllLog;
		eigenPLog->close(); delete eigenPLog;
		eigenALog->close(); delete eigenALog;
		DiagonalLog->close(); delete DiagonalLog;
		variancesLog->close(); delete variancesLog;
		nullspacesLog->close(); delete nullspacesLog;
	}

	delete[] selectionMap;

	for(FrameShell* s : allFrameHistory)
		delete s;
	for(FrameHessian* fh : unmappedTrackedFrames)
		delete fh;

	delete coarseDistanceMap;
	delete coarseTracker;
	delete coarseTracker_forNewKF;
	delete coarseInitializer;
	delete pixelSelector;
	delete ef;
}

void FullSystem::setOriginalCalib(const VecXf &originalCalib, int originalW, int originalH)
{

}

//* 设置相机响应函数
// 设置Hcalib数组
// 0   ---> 0
// 
// 255 ---> 255
void FullSystem::setGammaFunction(float* BInv)
{
	if(BInv==0) return;

	// copy BInv.
	memcpy(Hcalib.Binv, BInv, sizeof(float)*256);


	// invert.
	for(int i=1;i<255;i++)
	{
		// find val, such that Binv[val] = i.
		// I dont care about speed for this, so do it the stupid way.

		for(int s=1;s<255;s++)
		{
			if(BInv[s] <= i && BInv[s+1] >= i)
			{
				Hcalib.B[i] = s+(i - BInv[s]) / (BInv[s+1]-BInv[s]);
				break;
			}
		}
	}
	Hcalib.B[0] = 0;
	Hcalib.B[255] = 255;
}


// 输出结果
void FullSystem::printResult(std::string file)
{
	boost::unique_lock<boost::mutex> lock(trackMutex);
	boost::unique_lock<boost::mutex> crlock(shellPoseMutex);

	std::ofstream myfile;
	myfile.open (file.c_str());
	myfile << std::setprecision(15);

	for(FrameShell* s : allFrameHistory)
	{
		if(!s->poseValid) continue;

		if(setting_onlyLogKFPoses && s->marginalizedAt == s->id) continue;

		myfile << s->timestamp <<
			" " << s->camToWorld.translation().transpose()<<
			" " << s->camToWorld.so3().unit_quaternion().x()<<
			" " << s->camToWorld.so3().unit_quaternion().y()<<
			" " << s->camToWorld.so3().unit_quaternion().z()<<
			" " << s->camToWorld.so3().unit_quaternion().w() << "\n";
	}
	myfile.close();
}

//@ 使用确定的运动模型对新来的一帧进行跟踪, 得到位姿和光度参数
Vec4 FullSystem::trackNewCoarse(FrameHessian* fh)
{

	assert(allFrameHistory.size() > 0);
	// set pose initialization.

    for(IOWrap::Output3DWrapper* ow : outputWrapper)
        ow->pushLiveFrame(fh);



	FrameHessian* lastF = coarseTracker->lastRef;  // 参考帧

	AffLight aff_last_2_l = AffLight(0,0);
//[ ***step 1*** ] 设置不同的运动状态
	std::vector<SE3,Eigen::aligned_allocator<SE3>> lastF_2_fh_tries;
	printf("size: %d \n", lastF_2_fh_tries.size());
	if(allFrameHistory.size() == 2)
		for(unsigned int i=0;i<lastF_2_fh_tries.size();i++) lastF_2_fh_tries.push_back(SE3());  //? 这个size()不应该是0么
	else
	{
		FrameShell* slast = allFrameHistory[allFrameHistory.size()-2];   // 上一帧
		FrameShell* sprelast = allFrameHistory[allFrameHistory.size()-3];  // 大上一帧
		SE3 slast_2_sprelast;
		SE3 lastF_2_slast;
		{	// lock on global pose consistency!
			boost::unique_lock<boost::mutex> crlock(shellPoseMutex);
			slast_2_sprelast = sprelast->camToWorld.inverse() * slast->camToWorld;  // 上一帧和大上一帧的运动
			lastF_2_slast = slast->camToWorld.inverse() * lastF->shell->camToWorld;	// 参考帧到上一帧运动
			aff_last_2_l = slast->aff_g2l;
		}
		SE3 fh_2_slast = slast_2_sprelast;// assumed to be the same as fh_2_slast. // 当前帧到上一帧 = 上一帧和大上一帧的

		//! 尝试不同的运动
		// get last delta-movement.
		lastF_2_fh_tries.push_back(fh_2_slast.inverse() * lastF_2_slast);	// assume constant motion.
		lastF_2_fh_tries.push_back(fh_2_slast.inverse() * fh_2_slast.inverse() * lastF_2_slast);	// assume double motion (frame skipped)
		lastF_2_fh_tries.push_back(SE3::exp(fh_2_slast.log()*0.5).inverse() * lastF_2_slast); // assume half motion.
		lastF_2_fh_tries.push_back(lastF_2_slast); // assume zero motion.
		lastF_2_fh_tries.push_back(SE3()); // assume zero motion FROM KF.

		//! 尝试不同的旋转变动
		// just try a TON of different initializations (all rotations). In the end,
		// if they don't work they will only be tried on the coarsest level, which is super fast anyway.
		// also, if tracking rails here we loose, so we really, really want to avoid that.
		for(float rotDelta=0.02; rotDelta < 0.05; rotDelta++)
		{
			lastF_2_fh_tries.push_back(fh_2_slast.inverse() * lastF_2_slast * SE3(Sophus::Quaterniond(1,rotDelta,0,0), Vec3(0,0,0)));			// assume constant motion.
			lastF_2_fh_tries.push_back(fh_2_slast.inverse() * lastF_2_slast * SE3(Sophus::Quaterniond(1,0,rotDelta,0), Vec3(0,0,0)));			// assume constant motion.
			lastF_2_fh_tries.push_back(fh_2_slast.inverse() * lastF_2_slast * SE3(Sophus::Quaterniond(1,0,0,rotDelta), Vec3(0,0,0)));			// assume constant motion.
			lastF_2_fh_tries.push_back(fh_2_slast.inverse() * lastF_2_slast * SE3(Sophus::Quaterniond(1,-rotDelta,0,0), Vec3(0,0,0)));			// assume constant motion.
			lastF_2_fh_tries.push_back(fh_2_slast.inverse() * lastF_2_slast * SE3(Sophus::Quaterniond(1,0,-rotDelta,0), Vec3(0,0,0)));			// assume constant motion.
			lastF_2_fh_tries.push_back(fh_2_slast.inverse() * lastF_2_slast * SE3(Sophus::Quaterniond(1,0,0,-rotDelta), Vec3(0,0,0)));			// assume constant motion.
			lastF_2_fh_tries.push_back(fh_2_slast.inverse() * lastF_2_slast * SE3(Sophus::Quaterniond(1,rotDelta,rotDelta,0), Vec3(0,0,0)));	// assume constant motion.
			lastF_2_fh_tries.push_back(fh_2_slast.inverse() * lastF_2_slast * SE3(Sophus::Quaterniond(1,0,rotDelta,rotDelta), Vec3(0,0,0)));	// assume constant motion.
			lastF_2_fh_tries.push_back(fh_2_slast.inverse() * lastF_2_slast * SE3(Sophus::Quaterniond(1,rotDelta,0,rotDelta), Vec3(0,0,0)));	// assume constant motion.
			lastF_2_fh_tries.push_back(fh_2_slast.inverse() * lastF_2_slast * SE3(Sophus::Quaterniond(1,-rotDelta,rotDelta,0), Vec3(0,0,0)));	// assume constant motion.
			lastF_2_fh_tries.push_back(fh_2_slast.inverse() * lastF_2_slast * SE3(Sophus::Quaterniond(1,0,-rotDelta,rotDelta), Vec3(0,0,0)));	// assume constant motion.
			lastF_2_fh_tries.push_back(fh_2_slast.inverse() * lastF_2_slast * SE3(Sophus::Quaterniond(1,-rotDelta,0,rotDelta), Vec3(0,0,0)));	// assume constant motion.
			lastF_2_fh_tries.push_back(fh_2_slast.inverse() * lastF_2_slast * SE3(Sophus::Quaterniond(1,rotDelta,-rotDelta,0), Vec3(0,0,0)));	// assume constant motion.
			lastF_2_fh_tries.push_back(fh_2_slast.inverse() * lastF_2_slast * SE3(Sophus::Quaterniond(1,0,rotDelta,-rotDelta), Vec3(0,0,0)));	// assume constant motion.
			lastF_2_fh_tries.push_back(fh_2_slast.inverse() * lastF_2_slast * SE3(Sophus::Quaterniond(1,rotDelta,0,-rotDelta), Vec3(0,0,0)));	// assume constant motion.
			lastF_2_fh_tries.push_back(fh_2_slast.inverse() * lastF_2_slast * SE3(Sophus::Quaterniond(1,-rotDelta,-rotDelta,0), Vec3(0,0,0)));	// assume constant motion.
			lastF_2_fh_tries.push_back(fh_2_slast.inverse() * lastF_2_slast * SE3(Sophus::Quaterniond(1,0,-rotDelta,-rotDelta), Vec3(0,0,0)));	// assume constant motion.
			lastF_2_fh_tries.push_back(fh_2_slast.inverse() * lastF_2_slast * SE3(Sophus::Quaterniond(1,-rotDelta,0,-rotDelta), Vec3(0,0,0)));	// assume constant motion.
			lastF_2_fh_tries.push_back(fh_2_slast.inverse() * lastF_2_slast * SE3(Sophus::Quaterniond(1,-rotDelta,-rotDelta,-rotDelta), Vec3(0,0,0)));	// assume constant motion.
			lastF_2_fh_tries.push_back(fh_2_slast.inverse() * lastF_2_slast * SE3(Sophus::Quaterniond(1,-rotDelta,-rotDelta,rotDelta), Vec3(0,0,0)));	// assume constant motion.
			lastF_2_fh_tries.push_back(fh_2_slast.inverse() * lastF_2_slast * SE3(Sophus::Quaterniond(1,-rotDelta,rotDelta,-rotDelta), Vec3(0,0,0)));	// assume constant motion.
			lastF_2_fh_tries.push_back(fh_2_slast.inverse() * lastF_2_slast * SE3(Sophus::Quaterniond(1,-rotDelta,rotDelta,rotDelta), Vec3(0,0,0)));	// assume constant motion.
			lastF_2_fh_tries.push_back(fh_2_slast.inverse() * lastF_2_slast * SE3(Sophus::Quaterniond(1,rotDelta,-rotDelta,-rotDelta), Vec3(0,0,0)));	// assume constant motion.
			lastF_2_fh_tries.push_back(fh_2_slast.inverse() * lastF_2_slast * SE3(Sophus::Quaterniond(1,rotDelta,-rotDelta,rotDelta), Vec3(0,0,0)));	// assume constant motion.
			lastF_2_fh_tries.push_back(fh_2_slast.inverse() * lastF_2_slast * SE3(Sophus::Quaterniond(1,rotDelta,rotDelta,-rotDelta), Vec3(0,0,0)));	// assume constant motion.
			lastF_2_fh_tries.push_back(fh_2_slast.inverse() * lastF_2_slast * SE3(Sophus::Quaterniond(1,rotDelta,rotDelta,rotDelta), Vec3(0,0,0)));	// assume constant motion.
		}

		if(!slast->poseValid || !sprelast->poseValid || !lastF->shell->poseValid) // 有不和法的
		{
			lastF_2_fh_tries.clear();
			lastF_2_fh_tries.push_back(SE3());
		}
	}


	Vec3 flowVecs = Vec3(100,100,100);
	SE3 lastF_2_fh = SE3();
	AffLight aff_g2l = AffLight(0,0);


	//! as long as maxResForImmediateAccept is not reached, I'll continue through the options.
	//! I'll keep track of the so-far best achieved residual for each level in achievedRes. 
	//! 把到目前为止最好的残差值作为每一层的阈值
	//! If on a coarse level, tracking is WORSE than achievedRes, we will not continue to save time.
	//! 粗层的能量值大, 也不继续优化了, 来节省时间
	 

	Vec5 achievedRes = Vec5::Constant(NAN);
	bool haveOneGood = false;
	int tryIterations=0;
	//! 逐个尝试
	for(unsigned int i=0;i<lastF_2_fh_tries.size();i++)
	{
//[ ***step 2*** ] 尝试不同的运动状态, 得到跟踪是否良好
		AffLight aff_g2l_this = aff_last_2_l;  // 上一帧的赋值当前帧
		SE3 lastF_2_fh_this = lastF_2_fh_tries[i];
		
		bool trackingIsGood = coarseTracker->trackNewestCoarse(
				fh, lastF_2_fh_this, aff_g2l_this,
				pyrLevelsUsed-1,
				achievedRes);	// in each level has to be at least as good as the last try.
		tryIterations++;

		if(i != 0)
		{
			printf("RE-TRACK ATTEMPT %d with initOption %d and start-lvl %d (ab %f %f): %f %f %f %f %f -> %f %f %f %f %f \n",
					i,
					i, pyrLevelsUsed-1,
					aff_g2l_this.a,aff_g2l_this.b,
					achievedRes[0],
					achievedRes[1],
					achievedRes[2],
					achievedRes[3],
					achievedRes[4],
					coarseTracker->lastResiduals[0],
					coarseTracker->lastResiduals[1],
					coarseTracker->lastResiduals[2],
					coarseTracker->lastResiduals[3],
					coarseTracker->lastResiduals[4]);
		}

//[ ***step 3*** ] 如果跟踪正常, 并且0层残差比最好的还好留下位姿, 保存最好的每一层的能量值
		// do we have a new winner?
		if(trackingIsGood && std::isfinite((float)coarseTracker->lastResiduals[0]) && !(coarseTracker->lastResiduals[0] >=  achievedRes[0]))
		{
			//printf("take over. minRes %f -> %f!\n", achievedRes[0], coarseTracker->lastResiduals[0]);
			flowVecs = coarseTracker->lastFlowIndicators;
			aff_g2l = aff_g2l_this;
			lastF_2_fh = lastF_2_fh_this;
			haveOneGood = true;
		}

		// take over achieved res (always).
		if(haveOneGood)
		{
			for(int i=0;i<5;i++)
			{
				if(!std::isfinite((float)achievedRes[i]) || achievedRes[i] > coarseTracker->lastResiduals[i])	// take over if achievedRes is either bigger or NAN.
					achievedRes[i] = coarseTracker->lastResiduals[i]; // 里面保存的是各层得到的能量值
			}
		}

//[ ***step 4*** ] 小于阈值则暂停, 并且为下次设置阈值
        if(haveOneGood &&  achievedRes[0] < lastCoarseRMSE[0]*setting_reTrackThreshold)
            break;

	}

	if(!haveOneGood)
	{
        printf("BIG ERROR! tracking failed entirely. Take predictred pose and hope we may somehow recover.\n");
		flowVecs = Vec3(0,0,0);
		aff_g2l = aff_last_2_l;
		lastF_2_fh = lastF_2_fh_tries[0];
	}

	//! 把这次得到的最好值给下次用来当阈值
	lastCoarseRMSE = achievedRes;

//[ ***step 5*** ] 此时shell在跟踪阶段, 没人使用, 设置值
	// no lock required, as fh is not used anywhere yet.
	fh->shell->camToTrackingRef = lastF_2_fh.inverse();
	fh->shell->trackingRef = lastF->shell;
	fh->shell->aff_g2l = aff_g2l;
	fh->shell->camToWorld = fh->shell->trackingRef->camToWorld * fh->shell->camToTrackingRef;


	if(coarseTracker->firstCoarseRMSE < 0)
		coarseTracker->firstCoarseRMSE = achievedRes[0];  // 第一次跟踪的平均能量值

    if(!setting_debugout_runquiet)
        printf("Coarse Tracker tracked ab = %f %f (exp %f). Res %f!\n", aff_g2l.a, aff_g2l.b, fh->ab_exposure, achievedRes[0]);



	if(setting_logStuff)
	{
		(*coarseTrackingLog) << std::setprecision(16)
						<< fh->shell->id << " "
						<< fh->shell->timestamp << " "
						<< fh->ab_exposure << " "
						<< fh->shell->camToWorld.log().transpose() << " "
						<< aff_g2l.a << " "
						<< aff_g2l.b << " "
						<< achievedRes[0] << " "
						<< tryIterations << "\n";
	}


	return Vec4(achievedRes[0], flowVecs[0], flowVecs[1], flowVecs[2]);
}

//@ 利用新的帧 fh 对关键帧中的ImmaturePoint进行更新
void FullSystem::traceNewCoarse(FrameHessian* fh)
{
	boost::unique_lock<boost::mutex> lock(mapMutex);

	int trace_total=0, trace_good=0, trace_oob=0, trace_out=0, trace_skip=0, trace_badcondition=0, trace_uninitialized=0;

	Mat33f K = Mat33f::Identity();
	K(0,0) = Hcalib.fxl();
	K(1,1) = Hcalib.fyl();
	K(0,2) = Hcalib.cxl();
	K(1,2) = Hcalib.cyl();

	// 遍历关键帧
	for(FrameHessian* host : frameHessians)		// go through all active frames
	{

		SE3 hostToNew = fh->PRE_worldToCam * host->PRE_camToWorld;
		Mat33f KRKi = K * hostToNew.rotationMatrix().cast<float>() * K.inverse();
		Vec3f Kt = K * hostToNew.translation().cast<float>();

		Vec2f aff = AffLight::fromToVecExposure(host->ab_exposure, fh->ab_exposure, host->aff_g2l(), fh->aff_g2l()).cast<float>();

		for(ImmaturePoint* ph : host->immaturePoints)
		{
			ph->traceOn(fh, KRKi, Kt, aff, &Hcalib, false );
			if(ph->lastTraceStatus==ImmaturePointStatus::IPS_GOOD) trace_good++;
			if(ph->lastTraceStatus==ImmaturePointStatus::IPS_BADCONDITION) trace_badcondition++;
			if(ph->lastTraceStatus==ImmaturePointStatus::IPS_OOB) trace_oob++;
			if(ph->lastTraceStatus==ImmaturePointStatus::IPS_OUTLIER) trace_out++;
			if(ph->lastTraceStatus==ImmaturePointStatus::IPS_SKIPPED) trace_skip++;
			if(ph->lastTraceStatus==ImmaturePointStatus::IPS_UNINITIALIZED) trace_uninitialized++;
			trace_total++;
		}
	}
//	printf("ADD: TRACE: %'d points. %'d (%.0f%%) good. %'d (%.0f%%) skip. %'d (%.0f%%) badcond. %'d (%.0f%%) oob. %'d (%.0f%%) out. %'d (%.0f%%) uninit.\n",
//			trace_total,
//			trace_good, 100*trace_good/(float)trace_total,
//			trace_skip, 100*trace_skip/(float)trace_total,
//			trace_badcondition, 100*trace_badcondition/(float)trace_total,
//			trace_oob, 100*trace_oob/(float)trace_total,
//			trace_out, 100*trace_out/(float)trace_total,
//			trace_uninitialized, 100*trace_uninitialized/(float)trace_total);
}



//@ 处理挑选出来待激活的点
void FullSystem::activatePointsMT_Reductor(
		std::vector<PointHessian*>* optimized,
		std::vector<ImmaturePoint*>* toOptimize,
		int min, int max, Vec10* stats, int tid)
{
	ImmaturePointTemporaryResidual* tr = new ImmaturePointTemporaryResidual[frameHessians.size()];
	for(int k=min;k<max;k++)
	{
		(*optimized)[k] = optimizeImmaturePoint((*toOptimize)[k],1,tr);
	}
	delete[] tr;
}


//@ 激活未成熟点, 加入优化
void FullSystem::activatePointsMT()
{
//[ ***step 1*** ] 阈值计算, 通过距离地图来控制数目
	//currentMinActDist 初值为 2 
	//* 这太牛逼了.....参数
	if(ef->nPoints < setting_desiredPointDensity*0.66)
		currentMinActDist -= 0.8;
	if(ef->nPoints < setting_desiredPointDensity*0.8)
		currentMinActDist -= 0.5;
	else if(ef->nPoints < setting_desiredPointDensity*0.9)
		currentMinActDist -= 0.2;
	else if(ef->nPoints < setting_desiredPointDensity)
		currentMinActDist -= 0.1;

	if(ef->nPoints > setting_desiredPointDensity*1.5)
		currentMinActDist += 0.8;
	if(ef->nPoints > setting_desiredPointDensity*1.3)
		currentMinActDist += 0.5;
	if(ef->nPoints > setting_desiredPointDensity*1.15)
		currentMinActDist += 0.2;
	if(ef->nPoints > setting_desiredPointDensity)
		currentMinActDist += 0.1;

	if(currentMinActDist < 0) currentMinActDist = 0;
	if(currentMinActDist > 4) currentMinActDist = 4;

    if(!setting_debugout_runquiet)
        printf("SPARSITY:  MinActDist %f (need %d points, have %d points)!\n",
                currentMinActDist, (int)(setting_desiredPointDensity), ef->nPoints);



	FrameHessian* newestHs = frameHessians.back();

	// make dist map.
	coarseDistanceMap->makeK(&Hcalib);
	coarseDistanceMap->makeDistanceMap(frameHessians, newestHs);

	//coarseTracker->debugPlotDistMap("distMap");

	std::vector<ImmaturePoint*> toOptimize; toOptimize.reserve(20000); // 待激活的点

//[ ***step 2*** ] 处理未成熟点, 激活/删除/跳过
	for(FrameHessian* host : frameHessians)		// go through all active frames
	{
		if(host == newestHs) continue;

		SE3 fhToNew = newestHs->PRE_worldToCam * host->PRE_camToWorld;
		// 第0层到1层
		Mat33f KRKi = (coarseDistanceMap->K[1] * fhToNew.rotationMatrix().cast<float>() * coarseDistanceMap->Ki[0]);
		Vec3f Kt = (coarseDistanceMap->K[1] * fhToNew.translation().cast<float>());


		for(unsigned int i=0;i<host->immaturePoints.size();i+=1)
		{
			ImmaturePoint* ph = host->immaturePoints[i];
			ph->idxInImmaturePoints = i;

			// delete points that have never been traced successfully, or that are outlier on the last trace.
			if(!std::isfinite(ph->idepth_max) || ph->lastTraceStatus == IPS_OUTLIER)
			{
				//	immature_invalid_deleted++;
				// remove point.
				delete ph;
				host->immaturePoints[i]=0; // 指针赋零
				continue;
			}
			
			//* 未成熟点的激活条件
			// can activate only if this is true.
			bool canActivate = (ph->lastTraceStatus == IPS_GOOD
					|| ph->lastTraceStatus == IPS_SKIPPED
					|| ph->lastTraceStatus == IPS_BADCONDITION
					|| ph->lastTraceStatus == IPS_OOB )
							&& ph->lastTracePixelInterval < 8
							&& ph->quality > setting_minTraceQuality
							&& (ph->idepth_max+ph->idepth_min) > 0;


			// if I cannot activate the point, skip it. Maybe also delete it.
			if(!canActivate)
			{
				//* 删除被边缘化帧上的, 和OOB点
				// if point will be out afterwards, delete it instead.
				if(ph->host->flaggedForMarginalization || ph->lastTraceStatus == IPS_OOB)
				{
					// immature_notReady_deleted++;
					delete ph;
					host->immaturePoints[i]=0;
				}
				// immature_notReady_skipped++;
				continue;
			}


			// see if we need to activate point due to distance map.
			Vec3f ptp = KRKi * Vec3f(ph->u, ph->v, 1) + Kt*(0.5f*(ph->idepth_max+ph->idepth_min));
			int u = ptp[0] / ptp[2] + 0.5f;
			int v = ptp[1] / ptp[2] + 0.5f;

			if((u > 0 && v > 0 && u < wG[1] && v < hG[1]))
			{
				// 距离地图 + 小数点
				float dist = coarseDistanceMap->fwdWarpedIDDistFinal[u+wG[1]*v] + (ptp[0]-floorf((float)(ptp[0])));

				if(dist>=currentMinActDist* ph->my_type) // 点越多, 距离阈值越大
				{
					coarseDistanceMap->addIntoDistFinal(u,v);
					toOptimize.push_back(ph);
				}
			}
			else
			{
				delete ph;
				host->immaturePoints[i]=0;
			}
		}
	}


		//	printf("ACTIVATE: %d. (del %d, notReady %d, marg %d, good %d, marg-skip %d)\n",
		//			(int)toOptimize.size(), immature_deleted, immature_notReady, immature_needMarg, immature_want, immature_margskip);
//[ ***step 3*** ] 优化上一步挑出来的未成熟点, 进行逆深度优化, 并得到pointhessian
	std::vector<PointHessian*> optimized; optimized.resize(toOptimize.size());

	if(multiThreading)
		treadReduce.reduce(boost::bind(&FullSystem::activatePointsMT_Reductor, this, &optimized, &toOptimize, _1, _2, _3, _4), 0, toOptimize.size(), 50);

	else
		activatePointsMT_Reductor(&optimized, &toOptimize, 0, toOptimize.size(), 0, 0);

//[ ***step 4*** ] 把PointHessian加入到能量函数, 删除收敛的未成熟点, 或不好的点
	for(unsigned k=0;k<toOptimize.size();k++)
	{
		PointHessian* newpoint = optimized[k];
		ImmaturePoint* ph = toOptimize[k];

		if(newpoint != 0 && newpoint != (PointHessian*)((long)(-1)))
		{
			newpoint->host->immaturePoints[ph->idxInImmaturePoints]=0;
			newpoint->host->pointHessians.push_back(newpoint);
			ef->insertPoint(newpoint);		// 能量函数中插入点
			for(PointFrameResidual* r : newpoint->residuals)
				ef->insertResidual(r);		// 能量函数中插入残差
			assert(newpoint->efPoint != 0);
			delete ph;
		}
		else if(newpoint == (PointHessian*)((long)(-1)) || ph->lastTraceStatus==IPS_OOB)
		{
			// bug: 原来的顺序错误
			ph->host->immaturePoints[ph->idxInImmaturePoints]=0;
			delete ph;
		}
		else
		{
			assert(newpoint == 0 || newpoint == (PointHessian*)((long)(-1)));
		}
	}

//[ ***step 5*** ] 把删除的点丢掉
	for(FrameHessian* host : frameHessians)
	{
		for(int i=0;i<(int)host->immaturePoints.size();i++)
		{
			if(host->immaturePoints[i]==0)
			{
				//bug 如果back的也是空的呢
				host->immaturePoints[i] = host->immaturePoints.back(); // 没有顺序要求, 直接最后一个给空的
				host->immaturePoints.pop_back();
				i--;
			}
		}
	}


}






void FullSystem::activatePointsOldFirst()
{
	assert(false);
}

//@ 标记要移除点的状态, 边缘化or丢掉
void FullSystem::flagPointsForRemoval()
{
	assert(EFIndicesValid);

	std::vector<FrameHessian*> fhsToKeepPoints;
	std::vector<FrameHessian*> fhsToMargPoints;

	//if(setting_margPointVisWindow>0)
	{	//bug 又是不用的一条语句
		for(int i=((int)frameHessians.size())-1;i>=0 && i >= ((int)frameHessians.size());i--)
			if(!frameHessians[i]->flaggedForMarginalization) fhsToKeepPoints.push_back(frameHessians[i]);

		for(int i=0; i< (int)frameHessians.size();i++)
			if(frameHessians[i]->flaggedForMarginalization) fhsToMargPoints.push_back(frameHessians[i]);
	}



	//ef->setAdjointsF();
	//ef->setDeltaF(&Hcalib);
	int flag_oob=0, flag_in=0, flag_inin=0, flag_nores=0;

	for(FrameHessian* host : frameHessians)		// go through all active frames
	{
		for(unsigned int i=0;i<host->pointHessians.size();i++)
		{
			PointHessian* ph = host->pointHessians[i];
			if(ph==0) continue;

			//* 丢掉相机后面, 没有残差的点
			if(ph->idepth_scaled < 0 || ph->residuals.size()==0)
			{
				host->pointHessiansOut.push_back(ph);
				ph->efPoint->stateFlag = EFPointStatus::PS_DROP;
				host->pointHessians[i]=0;
				flag_nores++;
			}
			//* 把边缘化的帧上的点, 以及受影响较大的点标记为边缘化or删除
			else if(ph->isOOB(fhsToKeepPoints, fhsToMargPoints) || host->flaggedForMarginalization)
			{
				flag_oob++;
				//* 如果是一个内点, 则把残差在当前状态线性化, 并计算到零点残差
				if(ph->isInlierNew())
				{
					flag_in++;
					int ngoodRes=0;
					for(PointFrameResidual* r : ph->residuals)
					{
						r->resetOOB();
						r->linearize(&Hcalib);
						r->efResidual->isLinearized = false;
						r->applyRes(true);
						// 如果是激活(可参与优化)的残差, 则给fix住, 计算res_toZeroF
						if(r->efResidual->isActive())
						{
							r->efResidual->fixLinearizationF(ef);
							ngoodRes++;
						}
					}
					//* 如果逆深度的协方差很大直接扔掉, 小的边缘化掉
                    if(ph->idepth_hessian > setting_minIdepthH_marg)
					{
						flag_inin++;
						ph->efPoint->stateFlag = EFPointStatus::PS_MARGINALIZE;
						host->pointHessiansMarginalized.push_back(ph);
					}
					else
					{
						ph->efPoint->stateFlag = EFPointStatus::PS_DROP;
						host->pointHessiansOut.push_back(ph);
					}


				}
				//* 不是内点直接扔掉
				else
				{
					host->pointHessiansOut.push_back(ph);
					ph->efPoint->stateFlag = EFPointStatus::PS_DROP;


					//printf("drop point in frame %d (%d goodRes, %d activeRes)\n", ph->host->idx, ph->numGoodResiduals, (int)ph->residuals.size());
				}

				host->pointHessians[i]=0; // 把点给删除
			}
		}

		//* 删除边缘化或者删除的点
		for(int i=0;i<(int)host->pointHessians.size();i++)
		{
			if(host->pointHessians[i]==0)
			{
				host->pointHessians[i] = host->pointHessians.back();
				host->pointHessians.pop_back();
				i--;
			}
		}
	}

}

/********************************
 * @ function:
 * 
 * @ param: 	image		标定后的辐照度和曝光时间
 * @			id			
 * 
 * @ note: start from here
 *******************************/
void FullSystem::addActiveFrame( ImageAndExposure* image, int id )
{
	//[ ***step 1*** ] track线程锁
    if(isLost) return;
	boost::unique_lock<boost::mutex> lock(trackMutex);


	//[ ***step 2*** ] 创建FrameHessian和FrameShell, 并进行相应初始化, 并存储所有帧
	// =========================== add into allFrameHistory =========================
	FrameHessian* fh = new FrameHessian();
	FrameShell* shell = new FrameShell();
	shell->camToWorld = SE3(); 		// no lock required, as fh is not used anywhere yet.
	shell->aff_g2l = AffLight(0,0);
    shell->marginalizedAt = shell->id = allFrameHistory.size();
    shell->timestamp = image->timestamp;
    shell->incoming_id = id;
	fh->shell = shell;
	allFrameHistory.push_back(shell);  // 只把简略的shell存起来

	//[ ***step 3*** ] 得到曝光时间, 生成金字塔, 计算整个图像梯度
	// =========================== make Images / derivatives etc. =========================
	fh->ab_exposure = image->exposure_time;
	
	// 生成金字塔，计算整个图像梯度
    fh->makeImages(image->image, &Hcalib);



	//[ ***step 4*** ] 进行初始化
	if(!initialized)
	{
		// use initializer!
		//[ ***step 4.1*** ] 加入第一帧
		if(coarseInitializer->frameID<0)	// first frame set. fh is kept by coarseInitializer.
		{
			coarseInitializer->setFirst(&Hcalib, fh);
		}
		else if(coarseInitializer->trackFrame(fh, outputWrapper))	// if SNAPPED
		{
		//[ ***step 4.2*** ] 跟踪成功, 完成初始化
			initializeFromInitializer(fh);
			lock.unlock();
			deliverTrackedFrame(fh, true);
		}
		else
		{
			// if still initializing
			fh->shell->poseValid = false;
			delete fh;
		}
		return;
	}
	else	// do front-end operation.
	{
//[ ***step 5*** ] 对新来的帧进行跟踪, 得到位姿光度, 判断跟踪状态
		// =========================== SWAP tracking reference?. =========================
		if(coarseTracker_forNewKF->refFrameID > coarseTracker->refFrameID)
		{
			// 交换参考帧和当前帧的coarseTracker
			boost::unique_lock<boost::mutex> crlock(coarseTrackerSwapMutex);
			CoarseTracker* tmp = coarseTracker; coarseTracker=coarseTracker_forNewKF; coarseTracker_forNewKF=tmp;
		}

		//TODO 使用旋转和位移对像素移动的作用比来判断运动状态
		Vec4 tres = trackNewCoarse(fh);
		if(!std::isfinite((double)tres[0]) || !std::isfinite((double)tres[1]) || !std::isfinite((double)tres[2]) || !std::isfinite((double)tres[3]))
        {
            printf("Initial Tracking failed: LOST!\n");
			isLost=true;
            return;
        }
//[ ***step 6*** ] 判断是否插入关键帧
		bool needToMakeKF = false;
		if(setting_keyframesPerSecond > 0)  // 每隔多久插入关键帧
		{
			needToMakeKF = allFrameHistory.size()== 1 ||
					(fh->shell->timestamp - allKeyFramesHistory.back()->timestamp) > 0.95f/setting_keyframesPerSecond;
		}
		else
		{
			Vec2 refToFh=AffLight::fromToVecExposure(coarseTracker->lastRef->ab_exposure, fh->ab_exposure,
					coarseTracker->lastRef_aff_g2l, fh->shell->aff_g2l);

			// BRIGHTNESS CHECK
			needToMakeKF = allFrameHistory.size()== 1 ||
					setting_kfGlobalWeight*setting_maxShiftWeightT *  sqrtf((double)tres[1]) / (wG[0]+hG[0]) +  // 平移像素位移
					setting_kfGlobalWeight*setting_maxShiftWeightR *  sqrtf((double)tres[2]) / (wG[0]+hG[0]) + 	//TODO 旋转像素位移, 设置为0???
					setting_kfGlobalWeight*setting_maxShiftWeightRT * sqrtf((double)tres[3]) / (wG[0]+hG[0]) +	// 旋转+平移像素位移
					setting_kfGlobalWeight*setting_maxAffineWeight * fabs(logf((float)refToFh[0])) > 1 ||		// 光度变化大
					2*coarseTracker->firstCoarseRMSE < tres[0];		// 误差能量变化太大(最初的两倍)

		}




        for(IOWrap::Output3DWrapper* ow : outputWrapper)
            ow->publishCamPose(fh->shell, &Hcalib);



//[ ***step 7*** ] 把该帧发布出去
		lock.unlock();
		deliverTrackedFrame(fh, needToMakeKF);
		return;
	}
}

//@ 把跟踪的帧, 给到建图线程, 设置成关键帧或非关键帧
void FullSystem::deliverTrackedFrame(FrameHessian* fh, bool needKF)
{

	//! 顺序执行
	if(linearizeOperation) 
	{
		if(goStepByStep && lastRefStopID != coarseTracker->refFrameID)
		{
			MinimalImageF3 img(wG[0], hG[0], fh->dI);
			IOWrap::displayImage("frameToTrack", &img);
			while(true)
			{
				char k=IOWrap::waitKey(0);
				if(k==' ') break;
				handleKey( k );
			}
			lastRefStopID = coarseTracker->refFrameID;
		}
		else handleKey( IOWrap::waitKey(1) );



		if(needKF) makeKeyFrame(fh);
		else makeNonKeyFrame(fh);
	}
	else
	{
		boost::unique_lock<boost::mutex> lock(trackMapSyncMutex); // 跟踪和建图同步锁
		unmappedTrackedFrames.push_back(fh);
		if(needKF) needNewKFAfter=fh->shell->trackingRef->id;
		trackedFrameSignal.notify_all();

		while(coarseTracker_forNewKF->refFrameID == -1 && coarseTracker->refFrameID == -1 )
		{
			mappedFrameSignal.wait(lock);  // 当没有跟踪的图像, 就一直阻塞trackMapSyncMutex, 直到notify
		}

		lock.unlock();
	}
}

//@ 建图线程
void FullSystem::mappingLoop()
{
	boost::unique_lock<boost::mutex> lock(trackMapSyncMutex);

	while(runMapping)
	{
		while(unmappedTrackedFrames.size()==0)
		{
			trackedFrameSignal.wait(lock);   // 没有图像等待trackedFrameSignal唤醒
			if(!runMapping) return;
		}

		FrameHessian* fh = unmappedTrackedFrames.front();
		unmappedTrackedFrames.pop_front();


		// guaranteed to make a KF for the very first two tracked frames.
		if(allKeyFramesHistory.size() <= 2)
		{
			lock.unlock();		// 运行makeKeyFrame是不会影响unmappedTrackedFrames的, 所以解锁
			makeKeyFrame(fh);`
			lock.lock();
			mappedFrameSignal.notify_all();  // 结束前唤醒
			continue;
		}

		if(unmappedTrackedFrames.size() > 3)
			needToKetchupMapping=true;


		if(unmappedTrackedFrames.size() > 0) // if there are other frames to tracke, do that first.
		{
			lock.unlock();
			makeNonKeyFrame(fh);
			lock.lock();

			if(needToKetchupMapping && unmappedTrackedFrames.size() > 0)  // 太多了给处理掉
			{
				FrameHessian* fh = unmappedTrackedFrames.front();
				unmappedTrackedFrames.pop_front();
				{
					boost::unique_lock<boost::mutex> crlock(shellPoseMutex);
					assert(fh->shell->trackingRef != 0);
					fh->shell->camToWorld = fh->shell->trackingRef->camToWorld * fh->shell->camToTrackingRef;
					fh->setEvalPT_scaled(fh->shell->camToWorld.inverse(),fh->shell->aff_g2l);
				}
				delete fh;
			}

		}
		else
		{
			if(setting_realTimeMaxKF || needNewKFAfter >= frameHessians.back()->shell->id)  // 后面需要关键帧
			{
				lock.unlock();
				makeKeyFrame(fh);
				needToKetchupMapping=false;
				lock.lock();
			}
			else
			{
				lock.unlock();
				makeNonKeyFrame(fh);
				lock.lock();
			}
		}
		mappedFrameSignal.notify_all();
	}
	printf("MAPPING FINISHED!\n");
}

//@ 结束建图线程
void FullSystem::blockUntilMappingIsFinished()
{
	boost::unique_lock<boost::mutex> lock(trackMapSyncMutex);
	runMapping = false;
	trackedFrameSignal.notify_all();
	lock.unlock();

	mappingThread.join();

}

//@ 设置成非关键帧
void FullSystem::makeNonKeyFrame( FrameHessian* fh)
{
	// needs to be set by mapping thread. no lock required since we are in mapping thread.
	{
		boost::unique_lock<boost::mutex> crlock(shellPoseMutex); // 生命周期结束后自动解锁
		assert(fh->shell->trackingRef != 0);
		// mapping时将它当前位姿取出来得到camToWorld
		fh->shell->camToWorld = fh->shell->trackingRef->camToWorld * fh->shell->camToTrackingRef;
		// 把此时估计的位姿取出来
		fh->setEvalPT_scaled(fh->shell->camToWorld.inverse(),fh->shell->aff_g2l);
	}

	traceNewCoarse(fh);  // 更新未成熟点(深度未收敛的点)
	delete fh;
}

//@ 生成关键帧, 优化, 激活点, 提取点, 边缘化关键帧
void FullSystem::makeKeyFrame( FrameHessian* fh)
{

//[ ***step 1*** ] 设置当前估计的fh的位姿, 光度参数
	// needs to be set by mapping thread
	{	// 同样取出位姿, 当前的作为最终值
		//? 为啥要从shell来设置 ???   答: 因为shell不删除, 而且参考帧还会被优化, shell是桥梁
		boost::unique_lock<boost::mutex> crlock(shellPoseMutex);
		assert(fh->shell->trackingRef != 0);
		fh->shell->camToWorld = fh->shell->trackingRef->camToWorld * fh->shell->camToTrackingRef;
		fh->setEvalPT_scaled(fh->shell->camToWorld.inverse(),fh->shell->aff_g2l); // 待优化值
	}

//[ ***step 2*** ] 把这一帧来更新之前帧的未成熟点
	traceNewCoarse(fh); // 更新未成熟点(深度未收敛的点)

	boost::unique_lock<boost::mutex> lock(mapMutex); // 建图锁

//[ ***step 3*** ] 选择要边缘化掉的帧
	// =========================== Flag Frames to be Marginalized. =========================
	flagFramesForMarginalization(fh);  // TODO 这里没用最新帧，可以改进下

//[ ***step 4*** ] 加入到关键帧序列
	// =========================== add New Frame to Hessian Struct. =========================
	fh->idx = frameHessians.size();
	frameHessians.push_back(fh);
	fh->frameID = allKeyFramesHistory.size();
	allKeyFramesHistory.push_back(fh->shell);
	ef->insertFrame(fh, &Hcalib);

	setPrecalcValues();	// 每添加一个关键帧都会运行这个来设置位姿, 设置位姿线性化点


//[ ***step 5*** ] 构建之前关键帧与当前帧fh的残差(旧的)
	// =========================== add new residuals for old points =========================
	int numFwdResAdde=0;
	for(FrameHessian* fh1 : frameHessians)		// go through all active frames
	{
		if(fh1 == fh) continue;
		for(PointHessian* ph : fh1->pointHessians)  // 全都构造之后再删除
		{
			PointFrameResidual* r = new PointFrameResidual(ph, fh1, fh); // 新建当前帧fh和之前帧之间的残差
			r->setState(ResState::IN);
			ph->residuals.push_back(r);
			ef->insertResidual(r);
			ph->lastResiduals[1] = ph->lastResiduals[0]; // 设置上上个残差
			ph->lastResiduals[0] = std::pair<PointFrameResidual*, ResState>(r, ResState::IN); // 当前的设置为上一个
			numFwdResAdde+=1;
		}
	}



//[ ***step 6*** ] 激活所有关键帧上的部分未成熟点(构造新的残差)
	// =========================== Activate Points (& flag for marginalization). =========================
	activatePointsMT();
	ef->makeIDX();  // ? 为啥要重新设置ID呢, 是因为加新的帧了么



//[ ***step 7*** ] 对滑窗内的关键帧进行优化(说的轻松, 里面好多问题)
	// =========================== OPTIMIZE ALL =========================

	fh->frameEnergyTH = frameHessians.back()->frameEnergyTH;  // 这两个不是一个值么???
	float rmse = optimize(setting_maxOptIterations);





	// =========================== Figure Out if INITIALIZATION FAILED =========================
	//* 所有的关键帧数小于4，认为还是初始化，此时残差太大认为初始化失败
	if(allKeyFramesHistory.size() <= 4)
	{
		if(allKeyFramesHistory.size()==2 && rmse > 20*benchmark_initializerSlackFactor)
		{
			printf("I THINK INITIALIZATINO FAILED! Resetting.\n");
			initFailed=true;
		}
		if(allKeyFramesHistory.size()==3 && rmse > 13*benchmark_initializerSlackFactor)
		{
			printf("I THINK INITIALIZATINO FAILED! Resetting.\n");
			initFailed=true;
		}
		if(allKeyFramesHistory.size()==4 && rmse > 9*benchmark_initializerSlackFactor)
		{
			printf("I THINK INITIALIZATINO FAILED! Resetting.\n");
			initFailed=true;
		}
	}



    if(isLost) return;  // 优化后的能量函数太大, 认为是跟丢了



//[ ***step 8*** ] 去除外点, 把最新帧设置为参考帧
//TODO 是否可以更加严格一些
	// =========================== REMOVE OUTLIER =========================
	removeOutliers();




	{
		boost::unique_lock<boost::mutex> crlock(coarseTrackerSwapMutex);
		coarseTracker_forNewKF->makeK(&Hcalib);  // 更新了内参, 因此重新make
		coarseTracker_forNewKF->setCoarseTrackingRef(frameHessians);



        coarseTracker_forNewKF->debugPlotIDepthMap(&minIdJetVisTracker, &maxIdJetVisTracker, outputWrapper);
        coarseTracker_forNewKF->debugPlotIDepthMapFloat(outputWrapper);
	}


	debugPlot("post Optimize");





//[ ***step 9*** ] 标记删除和边缘化的点, 并删除&边缘化
	// =========================== (Activate-)Marginalize Points =========================
	flagPointsForRemoval();
	ef->dropPointsF();  // 扔掉drop的点
	// 每次设置线性化点都会更新零空间
	getNullspaces(
			ef->lastNullspaces_pose,
			ef->lastNullspaces_scale,
			ef->lastNullspaces_affA,
			ef->lastNullspaces_affB);
	// 边缘化掉点, 加在HM, bM上
	ef->marginalizePointsF();


//[ ***step 10*** ] 生成新的点
	// =========================== add new Immature points & new residuals =========================
	makeNewTraces(fh, 0);





    for(IOWrap::Output3DWrapper* ow : outputWrapper)
    {
        ow->publishGraph(ef->connectivityMap);
        ow->publishKeyframes(frameHessians, false, &Hcalib);
    }



	// =========================== Marginalize Frames =========================
//[ ***step 11*** ] 边缘化掉关键帧
	//* 边缘化一帧要删除or边缘化上面所有点
	for(unsigned int i=0;i<frameHessians.size();i++)
		if(frameHessians[i]->flaggedForMarginalization)
			{marginalizeFrame(frameHessians[i]); i=0;}



	printLogLine();
    //printEigenValLine();

}

//@ 从初始化中提取出信息, 用于跟踪.
void FullSystem::initializeFromInitializer(FrameHessian* newFrame)
{
	boost::unique_lock<boost::mutex> lock(mapMutex);

	//[ ***step 1*** ] 把第一帧设置成关键帧, 加入队列, 加入EnergyFunctional
	// add firstframe.
	FrameHessian* firstFrame = coarseInitializer->firstFrame;  // 第一帧增加进地图
	firstFrame->idx = frameHessians.size(); // 赋值给它id (0开始)
	frameHessians.push_back(firstFrame);  	// 地图内关键帧容器
	firstFrame->frameID = allKeyFramesHistory.size();  	// 所有历史关键帧id
	allKeyFramesHistory.push_back(firstFrame->shell); 	// 所有历史关键帧
	ef->insertFrame(firstFrame, &Hcalib);
	setPrecalcValues();   		// 设置相对位姿预计算值

	//int numPointsTotal = makePixelStatus(firstFrame->dI, selectionMap, wG[0], hG[0], setting_desiredDensity);
	//int numPointsTotal = pixelSelector->makeMaps(firstFrame->dIp, selectionMap,setting_desiredDensity);

	firstFrame->pointHessians.reserve(wG[0]*hG[0]*0.2f); // 20%的点数目
	firstFrame->pointHessiansMarginalized.reserve(wG[0]*hG[0]*0.2f); // 被边缘化
	firstFrame->pointHessiansOut.reserve(wG[0]*hG[0]*0.2f); // 丢掉的点

	//[ ***step 2*** ] 求出平均尺度因子
	float sumID=1e-5, numID=1e-5;
	for(int i=0;i<coarseInitializer->numPoints[0];i++)
	{
		//? iR的值到底是啥
		sumID += coarseInitializer->points[0][i].iR; // 第0层点的中位值, 相当于
		numID++;
	}
	float rescaleFactor = 1 / (sumID / numID);  // 求出尺度因子

	// randomly sub-select the points I need.
	// 目标点数 / 实际提取点数
	float keepPercentage = setting_desiredPointDensity / coarseInitializer->numPoints[0];

    if(!setting_debugout_runquiet)
        printf("Initialization: keep %.1f%% (need %d, have %d)!\n", 100*keepPercentage,
                (int)(setting_desiredPointDensity), coarseInitializer->numPoints[0] );

	//[ ***step 3*** ] 创建PointHessian, 点加入关键帧, 加入EnergyFunctional
	for(int i=0;i<coarseInitializer->numPoints[0];i++)
	{
		if(rand()/(float)RAND_MAX > keepPercentage) continue; // 如果提取的点比较少, 不执行; 提取的多, 则随机干掉

		Pnt* point = coarseInitializer->points[0]+i;
		ImmaturePoint* pt = new ImmaturePoint(point->u+0.5f,point->v+0.5f,firstFrame,point->my_type, &Hcalib);

		if(!std::isfinite(pt->energyTH)) { delete pt; continue; }  // 点值无穷大

		// 创建ImmaturePoint就为了创建PointHessian? 是为了接口统一吧
		pt->idepth_max=pt->idepth_min=1;
		PointHessian* ph = new PointHessian(pt, &Hcalib);
		delete pt;
		if(!std::isfinite(ph->energyTH)) {delete ph; continue;}

		ph->setIdepthScaled(point->iR*rescaleFactor);  //? 为啥设置的是scaled之后的
		ph->setIdepthZero(ph->idepth);			//! 设置初始先验值, 还有神奇的求零空间方法
		ph->hasDepthPrior=true;
		ph->setPointStatus(PointHessian::ACTIVE);	// 激活点

		firstFrame->pointHessians.push_back(ph);
		ef->insertPoint(ph);
	}


	//[ ***step 4*** ] 设置第一帧和最新帧的待优化量, 参考帧
	SE3 firstToNew = coarseInitializer->thisToNext;
	firstToNew.translation() /= rescaleFactor;


	// really no lock required, as we are initializing.
	{
		boost::unique_lock<boost::mutex> crlock(shellPoseMutex);
		firstFrame->shell->camToWorld = SE3();		// 空的初值?
		firstFrame->shell->aff_g2l = AffLight(0,0);
		firstFrame->setEvalPT_scaled(firstFrame->shell->camToWorld.inverse(),firstFrame->shell->aff_g2l);
		firstFrame->shell->trackingRef=0;
		firstFrame->shell->camToTrackingRef = SE3();

		newFrame->shell->camToWorld = firstToNew.inverse();
		newFrame->shell->aff_g2l = AffLight(0,0);
		newFrame->setEvalPT_scaled(newFrame->shell->camToWorld.inverse(),newFrame->shell->aff_g2l);
		newFrame->shell->trackingRef = firstFrame->shell;
		newFrame->shell->camToTrackingRef = firstToNew.inverse();

	}

	initialized=true;
	printf("INITIALIZE FROM INITIALIZER (%d pts)!\n", (int)firstFrame->pointHessians.size());
}

//@ 提取新的像素点用来跟踪
void FullSystem::makeNewTraces(FrameHessian* newFrame, float* gtDepth)
{
	pixelSelector->allowFast = true;  //bug 没卵用
	//int numPointsTotal = makePixelStatus(newFrame->dI, selectionMap, wG[0], hG[0], setting_desiredDensity);
	int numPointsTotal = pixelSelector->makeMaps(newFrame, selectionMap,setting_desiredImmatureDensity);

	newFrame->pointHessians.reserve(numPointsTotal*1.2f);
	//fh->pointHessiansInactive.reserve(numPointsTotal*1.2f);
	newFrame->pointHessiansMarginalized.reserve(numPointsTotal*1.2f);
	newFrame->pointHessiansOut.reserve(numPointsTotal*1.2f);


	for(int y=patternPadding+1;y<hG[0]-patternPadding-2;y++)
	for(int x=patternPadding+1;x<wG[0]-patternPadding-2;x++)
	{
		int i = x+y*wG[0];
		if(selectionMap[i]==0) continue;

		ImmaturePoint* impt = new ImmaturePoint(x,y,newFrame, selectionMap[i], &Hcalib);
		if(!std::isfinite(impt->energyTH)) delete impt;  // 投影得到的不是有穷数
		else newFrame->immaturePoints.push_back(impt);

	}
	//printf("MADE %d IMMATURE POINTS!\n", (int)newFrame->immaturePoints.size());

}

//* 计算frameHessian的预计算值, 和状态的delta值
//@ 设置关键帧之间的关系
void FullSystem::setPrecalcValues()
{
	for(FrameHessian* fh : frameHessians)
	{
		fh->targetPrecalc.resize(frameHessians.size()); // 每个目标帧预运算容器, 大小是关键帧数
		for(unsigned int i=0;i<frameHessians.size();i++)  //? 还有自己和自己的???
			fh->targetPrecalc[i].set(fh, frameHessians[i], &Hcalib); // 计算Host 与 target之间的变换关系
	}

	ef->setDeltaF(&Hcalib);
}


void FullSystem::printLogLine()
{
	if(frameHessians.size()==0) return;

    if(!setting_debugout_runquiet)
        printf("LOG %d: %.3f fine. Res: %d A, %d L, %d M; (%'d / %'d) forceDrop. a=%f, b=%f. Window %d (%d)\n",
                allKeyFramesHistory.back()->id,
                statistics_lastFineTrackRMSE,
                ef->resInA,
                ef->resInL,
                ef->resInM,
                (int)statistics_numForceDroppedResFwd,
                (int)statistics_numForceDroppedResBwd,
                allKeyFramesHistory.back()->aff_g2l.a,
                allKeyFramesHistory.back()->aff_g2l.b,
                frameHessians.back()->shell->id - frameHessians.front()->shell->id,
                (int)frameHessians.size());


	if(!setting_logStuff) return;

	if(numsLog != 0)
	{
		(*numsLog) << allKeyFramesHistory.back()->id << " "  <<
				statistics_lastFineTrackRMSE << " "  <<
				(int)statistics_numCreatedPoints << " "  <<
				(int)statistics_numActivatedPoints << " "  <<
				(int)statistics_numDroppedPoints << " "  <<
				(int)statistics_lastNumOptIts << " "  <<
				ef->resInA << " "  <<
				ef->resInL << " "  <<
				ef->resInM << " "  <<
				statistics_numMargResFwd << " "  <<
				statistics_numMargResBwd << " "  <<
				statistics_numForceDroppedResFwd << " "  <<
				statistics_numForceDroppedResBwd << " "  <<
				frameHessians.back()->aff_g2l().a << " "  <<
				frameHessians.back()->aff_g2l().b << " "  <<
				frameHessians.back()->shell->id - frameHessians.front()->shell->id << " "  <<
				(int)frameHessians.size() << " "  << "\n";
		numsLog->flush();
	}


}



void FullSystem::printEigenValLine()
{
	if(!setting_logStuff) return;
	if(ef->lastHS.rows() < 12) return;


	MatXX Hp = ef->lastHS.bottomRightCorner(ef->lastHS.cols()-CPARS,ef->lastHS.cols()-CPARS);
	MatXX Ha = ef->lastHS.bottomRightCorner(ef->lastHS.cols()-CPARS,ef->lastHS.cols()-CPARS);
	int n = Hp.cols()/8;
	assert(Hp.cols()%8==0);

	// sub-select
	for(int i=0;i<n;i++)
	{
		MatXX tmp6 = Hp.block(i*8,0,6,n*8);
		Hp.block(i*6,0,6,n*8) = tmp6;

		MatXX tmp2 = Ha.block(i*8+6,0,2,n*8);
		Ha.block(i*2,0,2,n*8) = tmp2;
	}
	for(int i=0;i<n;i++)
	{
		MatXX tmp6 = Hp.block(0,i*8,n*8,6);
		Hp.block(0,i*6,n*8,6) = tmp6;

		MatXX tmp2 = Ha.block(0,i*8+6,n*8,2);
		Ha.block(0,i*2,n*8,2) = tmp2;
	}

	VecX eigenvaluesAll = ef->lastHS.eigenvalues().real();
	VecX eigenP = Hp.topLeftCorner(n*6,n*6).eigenvalues().real();
	VecX eigenA = Ha.topLeftCorner(n*2,n*2).eigenvalues().real();
	VecX diagonal = ef->lastHS.diagonal();

	std::sort(eigenvaluesAll.data(), eigenvaluesAll.data()+eigenvaluesAll.size());
	std::sort(eigenP.data(), eigenP.data()+eigenP.size());
	std::sort(eigenA.data(), eigenA.data()+eigenA.size());

	int nz = std::max(100,setting_maxFrames*10);

	if(eigenAllLog != 0)
	{
		VecX ea = VecX::Zero(nz); ea.head(eigenvaluesAll.size()) = eigenvaluesAll;
		(*eigenAllLog) << allKeyFramesHistory.back()->id << " " <<  ea.transpose() << "\n";
		eigenAllLog->flush();
	}
	if(eigenALog != 0)
	{
		VecX ea = VecX::Zero(nz); ea.head(eigenA.size()) = eigenA;
		(*eigenALog) << allKeyFramesHistory.back()->id << " " <<  ea.transpose() << "\n";
		eigenALog->flush();
	}
	if(eigenPLog != 0)
	{
		VecX ea = VecX::Zero(nz); ea.head(eigenP.size()) = eigenP;
		(*eigenPLog) << allKeyFramesHistory.back()->id << " " <<  ea.transpose() << "\n";
		eigenPLog->flush();
	}

	if(DiagonalLog != 0)
	{
		VecX ea = VecX::Zero(nz); ea.head(diagonal.size()) = diagonal;
		(*DiagonalLog) << allKeyFramesHistory.back()->id << " " <<  ea.transpose() << "\n";
		DiagonalLog->flush();
	}

	if(variancesLog != 0)
	{
		VecX ea = VecX::Zero(nz); ea.head(diagonal.size()) = ef->lastHS.inverse().diagonal();
		(*variancesLog) << allKeyFramesHistory.back()->id << " " <<  ea.transpose() << "\n";
		variancesLog->flush();
	}

	std::vector<VecX> &nsp = ef->lastNullspaces_forLogging;
	(*nullspacesLog) << allKeyFramesHistory.back()->id << " ";
	for(unsigned int i=0;i<nsp.size();i++)
		(*nullspacesLog) << nsp[i].dot(ef->lastHS * nsp[i]) << " " << nsp[i].dot(ef->lastbS) << " " ;
	(*nullspacesLog) << "\n";
	nullspacesLog->flush();

}

void FullSystem::printFrameLifetimes()
{
	if(!setting_logStuff) return;


	boost::unique_lock<boost::mutex> lock(trackMutex);

	std::ofstream* lg = new std::ofstream();
	lg->open("logs/lifetimeLog.txt", std::ios::trunc | std::ios::out);
	lg->precision(15);

	for(FrameShell* s : allFrameHistory)
	{
		(*lg) << s->id
			<< " " << s->marginalizedAt
			<< " " << s->statistics_goodResOnThis
			<< " " << s->statistics_outlierResOnThis
			<< " " << s->movedByOpt;



		(*lg) << "\n";
	}





	lg->close();
	delete lg;

}


void FullSystem::printEvalLine()
{
	return;
}





}
