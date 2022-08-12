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
#define MAX_ACTIVE_FRAMES 100

 
#include "util/globalCalib.h"
#include "vector"
 
#include <iostream>
#include <fstream>
#include "util/NumType.h"
#include "FullSystem/Residuals.h"
#include "util/ImageAndExposure.h"


namespace dso
{

//* 求得两个参考帧之间的光度仿射变换系数
// 设from是 i->j(ref->tar);  to是 k->j; 则结果是 i->k 的变换系数.
inline Vec2 affFromTo(const Vec2 &from, const Vec2 &to)	// contains affine parameters as XtoWorld.
{
	return Vec2(from[0] / to[0], (from[1] - to[1]) / to[0]);
}


//提前声明下
struct FrameHessian;
struct PointHessian;

class ImmaturePoint;
class FrameShell;

class EFFrame;
class EFPoint;

//? 这是干什么用的? 是为了求解时候的数值稳定? 
#define SCALE_IDEPTH 1.0f			//!< 逆深度的比例系数  // scales internal value to idepth.
#define SCALE_XI_ROT 1.0f			//!< 旋转量(so3)的比例系数
#define SCALE_XI_TRANS 0.5f			//!< 平移量的比例系数, 尺度?
#define SCALE_F 50.0f   			//!< 相机焦距的比例系数
#define SCALE_C 50.0f				//!< 相机光心偏移的比例系数
#define SCALE_W 1.0f				//!< 不知道...
#define SCALE_A 10.0f				//!< 光度仿射系数a的比例系数
#define SCALE_B 1000.0f				//!< 光度仿射系数b的比例系数

//上面的逆
#define SCALE_IDEPTH_INVERSE (1.0f / SCALE_IDEPTH)
#define SCALE_XI_ROT_INVERSE (1.0f / SCALE_XI_ROT)
#define SCALE_XI_TRANS_INVERSE (1.0f / SCALE_XI_TRANS)
#define SCALE_F_INVERSE (1.0f / SCALE_F)
#define SCALE_C_INVERSE (1.0f / SCALE_C)
#define SCALE_W_INVERSE (1.0f / SCALE_W)
#define SCALE_A_INVERSE (1.0f / SCALE_A)
#define SCALE_B_INVERSE (1.0f / SCALE_B)

//* 其中带0的是FEJ用的初始状态, 不带0的是更新的状态
struct FrameFramePrecalc
{
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
	// static values
	static int instanceCounter;
	FrameHessian* host;	// defines row
	FrameHessian* target;	// defines column

	
	// precalc values
	Mat33f PRE_RTll; // host 到 target 之间优化后旋转矩阵 R
	Mat33f PRE_KRKiTll; // k*R*k_inv
	Mat33f PRE_RKiTll;  // R*k_inv
	Mat33f PRE_RTll_0; // host 到 target之间初始的旋转矩阵, 优化更新前

	Vec2f PRE_aff_mode; // 能量函数对仿射系数处理后的, 总系数
	float PRE_b0_mode; // host的光度仿射系数b

	Vec3f PRE_tTll; //  host 到 target之间优化后的平移 t
	Vec3f PRE_KtTll; // K*t
	Vec3f PRE_tTll_0; //  host 到 target之间初始的平移, 优化更新前

	float distanceLL; // 两帧间距离


    inline ~FrameFramePrecalc() {}
    inline FrameFramePrecalc() {host=target=0;}
	void set(FrameHessian* host, FrameHessian* target, CalibHessian* HCalib);
};




//* 相机位姿+相机光度Hessian
struct FrameHessian
{
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
	EFFrame* efFrame;		//!< 帧的能量函数

	// constant info & pre-calculated values
	//DepthImageWrap* frame;
	FrameShell* shell;		//!< 帧的"壳", 保存一些不变的,要留下来的量

	//* 图像导数[0]:辐照度  [1]:x方向导数  [2]:y方向导数, （指针表示图像）
	Eigen::Vector3f* dI;				//!< 图像导数  // trace, fine tracking. Used for direction select (not for gradient histograms etc.)
	Eigen::Vector3f* dIp[PYR_LEVELS];	//!< 各金字塔层的图像导数  // coarse tracking / coarse initializer. NAN in [0] only.
	float* absSquaredGrad[PYR_LEVELS];  //!< x,y 方向梯度的平方和 // only used for pixel select (histograms etc.). no NAN.
	




	//* 都是ID
	int frameID;					//!< 所有关键帧的序号(FrameShell)	// incremental ID for keyframes only!
	static int instanceCounter;		//!< 计数器
	int idx;						//!< 激活关键帧的序号(FrameHessian)

	// Photometric Calibration Stuff
	float frameEnergyTH;	//!< 阈值 // set dynamically depending on tracking residual
	float ab_exposure;

	bool flaggedForMarginalization;

	std::vector<PointHessian*> pointHessians;				//!< contains all ACTIVE points.
	std::vector<PointHessian*> pointHessiansMarginalized;	//!< contains all MARGINALIZED points (= fully marginalized, usually because point went OOB.)
	std::vector<PointHessian*> pointHessiansOut;			//!< contains all OUTLIER points (= discarded.).
	std::vector<ImmaturePoint*> immaturePoints;				//!< contains all OUTLIER points (= discarded.).

	//* 零空间, 好奇怎么求???
	Mat66 nullspaces_pose;
	Mat42 nullspaces_affine;
	Vec6 nullspaces_scale;

	// variable info.
	SE3 worldToCam_evalPT;		//!< 在估计的相机位姿
	// [0-5: 位姿左乘小量. 6-7: a,b 光度仿射系数]
	//* 这三个是与线性化点的增量, 而光度参数不是增量, state就是值
	Vec10 state_zero;   		//!< 固定的线性化点的状态增量, 为了计算进行缩放
	Vec10 state_scaled;			//!< 乘上比例系数的状态增量, 这个是真正求的值!!!
	Vec10 state;				//!< 计算的状态增量
	//* step是与上一次优化结果的状态增量, [8 ,9]直接就设置为0了
	Vec10 step;					//!< 求解正规方程得到的增量
	Vec10 step_backup;			//!< 上一次的增量备份
	Vec10 state_backup;			//!< 上一次状态的备份

	//内联提高效率, 返回上面的值
    EIGEN_STRONG_INLINE const SE3 &get_worldToCam_evalPT() const {return worldToCam_evalPT;}
    EIGEN_STRONG_INLINE const Vec10 &get_state_zero() const {return state_zero;}
    EIGEN_STRONG_INLINE const Vec10 &get_state() const {return state;}
    EIGEN_STRONG_INLINE const Vec10 &get_state_scaled() const {return state_scaled;}
    EIGEN_STRONG_INLINE const Vec10 get_state_minus_stateZero() const {return get_state() - get_state_zero();} //x小量可以直接减


	// precalc values
	SE3 PRE_worldToCam;			//!< 预计算的, 位姿状态增量更新到位姿上
	SE3 PRE_camToWorld;
	std::vector<FrameFramePrecalc,Eigen::aligned_allocator<FrameFramePrecalc>> targetPrecalc; //!< 对于其它帧的预运算值
	MinimalImageB3* debugImage;	//!< 小图???


    inline Vec6 w2c_leftEps() const {return get_state_scaled().head<6>();}  //* 返回位姿状态增量
    inline AffLight aff_g2l() const {return AffLight(get_state_scaled()[6], get_state_scaled()[7]);} //* 返回光度仿射系数
    inline AffLight aff_g2l_0() const {return AffLight(get_state_zero()[6]*SCALE_A, get_state_zero()[7]*SCALE_B);} //* 返回线性化点处的仿射系数增量


	//* 设置FEJ点状态增量
	void setStateZero(const Vec10 &state_zero);
	//* 设置增量, 同时复制state和state_scale
	inline void setState(const Vec10 &state)
	{
		this->state = state;
		state_scaled.segment<3>(0) = SCALE_XI_TRANS * state.segment<3>(0);
		state_scaled.segment<3>(3) = SCALE_XI_ROT * state.segment<3>(3);
		state_scaled[6] = SCALE_A * state[6];
		state_scaled[7] = SCALE_B * state[7];
		state_scaled[8] = SCALE_A * state[8];
		state_scaled[9] = SCALE_B * state[9];
		//位姿更新
		PRE_worldToCam = SE3::exp(w2c_leftEps()) * get_worldToCam_evalPT();
		PRE_camToWorld = PRE_worldToCam.inverse();
		//setCurrentNullspace();
	};
	//* 设置增量, 传入state_scaled
	inline void setStateScaled(const Vec10 &state_scaled)
	{

		this->state_scaled = state_scaled;
		state.segment<3>(0) = SCALE_XI_TRANS_INVERSE * state_scaled.segment<3>(0);
		state.segment<3>(3) = SCALE_XI_ROT_INVERSE * state_scaled.segment<3>(3);
		state[6] = SCALE_A_INVERSE * state_scaled[6];
		state[7] = SCALE_B_INVERSE * state_scaled[7];
		state[8] = SCALE_A_INVERSE * state_scaled[8];
		state[9] = SCALE_B_INVERSE * state_scaled[9];
		
		PRE_worldToCam = SE3::exp(w2c_leftEps()) * get_worldToCam_evalPT();
		PRE_camToWorld = PRE_worldToCam.inverse();
		//setCurrentNullspace();
	};
	//* 设置当前位姿, 和状态增量, 同时设置了FEJ点
	inline void setEvalPT(const SE3 &worldToCam_evalPT, const Vec10 &state)
	{

		this->worldToCam_evalPT = worldToCam_evalPT;
		setState(state);
		setStateZero(state); 
	};


	//* 设置当前位姿, 光度仿射系数, FEJ点
	inline void setEvalPT_scaled(const SE3 &worldToCam_evalPT, const AffLight &aff_g2l)
	{
		Vec10 initial_state = Vec10::Zero();
		initial_state[6] = aff_g2l.a; // 直接设置光度系数a和b
		initial_state[7] = aff_g2l.b;
		this->worldToCam_evalPT = worldToCam_evalPT;
		setStateScaled(initial_state);
		setStateZero(this->get_state());
	};

	//* 释放该帧内存
	void release();

	inline ~FrameHessian()
	{
		assert(efFrame==0);
		release(); instanceCounter--;
		for(int i=0;i<pyrLevelsUsed;i++)
		{
			delete[] dIp[i];
			delete[]  absSquaredGrad[i];

		}



		if(debugImage != 0) delete debugImage;
	};
	inline FrameHessian()
	{
		instanceCounter++;  //! 若是发生拷贝, 就不会增加了
		flaggedForMarginalization=false;
		frameID = -1;
		efFrame = 0;
		frameEnergyTH = 8*8*patternNum;



		debugImage=0;
	};

    void makeImages(float* color, CalibHessian* HCalib);
	
	//* 获得先验信息矩阵， 怎么感觉除了第一帧没什么用
	inline Vec10 getPrior()
	{
		Vec10 p =  Vec10::Zero();
		
		if(frameID==0)  //* 第一帧就用初始值做先验
		{
			p.head<3>() = Vec3::Constant(setting_initialTransPrior);
			p.segment<3>(3) = Vec3::Constant(setting_initialRotPrior);
			// 用位运算, 有点东西
			if(setting_solverMode & SOLVER_REMOVE_POSEPRIOR) p.head<6>().setZero();

			p[6] = setting_initialAffAPrior; // 1e14
			p[7] = setting_initialAffBPrior; // 1e14
		}
		else //* 否则根据模式决定
		{
			if(setting_affineOptModeA < 0) //* 小于零是固定的不优化
				p[6] = setting_initialAffAPrior;
			else
				p[6] = setting_affineOptModeA;   // 1e12

			if(setting_affineOptModeB < 0)
				p[7] = setting_initialAffBPrior;
			else
				p[7] = setting_affineOptModeB;  // 1e8
		}
		//? 8,9是干嘛的呢???  没用....
		p[8] = setting_initialAffAPrior;
		p[9] = setting_initialAffBPrior;
		return p;
	}


	inline Vec10 getPriorZero()
	{
		return Vec10::Zero();
	}

};

//* 相机内参Hessian, 响应函数
struct CalibHessian
{
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
	static int instanceCounter;
	// * 4×1的向量
	VecC value_zero;				//!< FEJ固定点  			
	VecC value_scaled;				//!< 乘以scale的内参
	VecCf value_scaledf;			//!< float型的内参
	VecCf value_scaledi;			//!< 逆, 应该是求导用为, 1/fx, 1/fy, -cx/fx, -cy/fy
	VecC value;						//!< 没乘scale的
	VecC step;						//!< 迭代中的增量
	VecC step_backup;				//!< 上一次增量备份
	VecC value_backup;				//!< 上一次值的备份
	VecC value_minus_value_zero;	//!< 减去线性化点

    inline ~CalibHessian() {instanceCounter--;}
	inline CalibHessian()
	{

		VecC initial_value = VecC::Zero();
		//* 初始化内参
		initial_value[0] = fxG[0];
		initial_value[1] = fyG[0];
		initial_value[2] = cxG[0];
		initial_value[3] = cyG[0];

		setValueScaled(initial_value);
		value_zero = value;
		value_minus_value_zero.setZero();

		instanceCounter++;
		//响应函数
		for(int i=0;i<256;i++)
			Binv[i] = B[i] = i;		// set gamma function to identity
	};


	// normal mode: use the optimized parameters everywhere!
    inline float& fxl() {return value_scaledf[0];}
    inline float& fyl() {return value_scaledf[1];}
    inline float& cxl() {return value_scaledf[2];}
    inline float& cyl() {return value_scaledf[3];}
    inline float& fxli() {return value_scaledi[0];}
    inline float& fyli() {return value_scaledi[1];}
    inline float& cxli() {return value_scaledi[2];}
    inline float& cyli() {return value_scaledi[3];}


	//* 通过value设置
	inline void setValue(const VecC &value)
	{
		// [0-3: Kl, 4-7: Kr, 8-12: l2r] what's this, stereo camera???
		this->value = value;
		value_scaled[0] = SCALE_F * value[0];
		value_scaled[1] = SCALE_F * value[1];
		value_scaled[2] = SCALE_C * value[2];
		value_scaled[3] = SCALE_C * value[3];

		this->value_scaledf = this->value_scaled.cast<float>();
		this->value_scaledi[0] = 1.0f / this->value_scaledf[0];
		this->value_scaledi[1] = 1.0f / this->value_scaledf[1];
		this->value_scaledi[2] = - this->value_scaledf[2] / this->value_scaledf[0];
		this->value_scaledi[3] = - this->value_scaledf[3] / this->value_scaledf[1];
		this->value_minus_value_zero = this->value - this->value_zero;
	};
	//* 通过value_scaled赋值
	inline void setValueScaled(const VecC &value_scaled)
	{
		this->value_scaled = value_scaled;
		this->value_scaledf = this->value_scaled.cast<float>();
		value[0] = SCALE_F_INVERSE * value_scaled[0];
		value[1] = SCALE_F_INVERSE * value_scaled[1];
		value[2] = SCALE_C_INVERSE * value_scaled[2];
		value[3] = SCALE_C_INVERSE * value_scaled[3];

		this->value_minus_value_zero = this->value - this->value_zero;
		this->value_scaledi[0] = 1.0f / this->value_scaledf[0];
		this->value_scaledi[1] = 1.0f / this->value_scaledf[1];
		this->value_scaledi[2] = - this->value_scaledf[2] / this->value_scaledf[0];
		this->value_scaledi[3] = - this->value_scaledf[3] / this->value_scaledf[1];
	};

	//* gamma函数, 相机的响应函数G和G^-1, 映射到0~255
	float Binv[256];
	float B[256];

	//* 响应函数的导数
	EIGEN_STRONG_INLINE float getBGradOnly(float color)
	{
		int c = color+0.5f;
		if(c<5) c=5;
		if(c>250) c=250;
		return B[c+1]-B[c];
	}
	//* 响应函数逆的导数
	EIGEN_STRONG_INLINE float getBInvGradOnly(float color)
	{
		int c = color+0.5f;
		if(c<5) c=5;
		if(c>250) c=250;
		return Binv[c+1]-Binv[c];
	}
};

//* 点Hessian
// hessian component associated with one point.
struct PointHessian
{
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
	static int instanceCounter;
	EFPoint* efPoint; 						//!< 点的能量函数

	// static values
	float color[MAX_RES_PER_POINT];			// colors in host frame
	float weights[MAX_RES_PER_POINT];		// host-weights for respective residuals.



	float u,v;							//!< 像素点的位置
	int idx;							//!< 
	float energyTH;						//!< 光度误差阈值
	FrameHessian* host;					//!< 主帧
	bool hasDepthPrior;					//!< 初始化得到的点是有深度先验的, 其它没有

	float my_type;//不同类型点, 显示用

	float idepth_scaled;				//!< target还是host上点逆深度 ??
	float idepth_zero_scaled;			//!< FEJ使用, 点在host上x=0初始逆深度
	float idepth_zero;					//!< 缩放了scale倍的固定线性化点逆深度
	float idepth;						//!< 缩放scale倍的逆深度
	float step;							//!< 迭代优化每一步增量
	float step_backup;					//!< 迭代优化上一步增量的备份
	float idepth_backup;				//!< 上一次的逆深度值

	float nullspaces_scale;				//!< 零空间 ?
	float idepth_hessian;				//!< 对应的hessian矩阵值
	float maxRelBaseline;				//!< 衡量该点的最大基线长度
	int numGoodResiduals;
	
	enum PtStatus {ACTIVE=0, INACTIVE, OUTLIER, OOB, MARGINALIZED};  // 这些状态都没啥用.....
	PtStatus status;

    inline void setPointStatus(PtStatus s) {status=s;}

	//* 各种设置逆深度
	inline void setIdepth(float idepth) {
		this->idepth = idepth;
		this->idepth_scaled = SCALE_IDEPTH * idepth;
    }
	inline void setIdepthScaled(float idepth_scaled) {
		this->idepth = SCALE_IDEPTH_INVERSE * idepth_scaled;
		this->idepth_scaled = idepth_scaled;
    }
	inline void setIdepthZero(float idepth) {
		idepth_zero = idepth;
		idepth_zero_scaled = SCALE_IDEPTH * idepth;
		nullspaces_scale = -(idepth*1.001 - idepth/1.001)*500; //? 为啥这么求
    }

	//* 点的残差值
	std::vector<PointFrameResidual*> residuals;					// only contains good residuals (not OOB and not OUTLIER). Arbitrary order.
	std::pair<PointFrameResidual*, ResState> lastResiduals[2]; 	// contains information about residuals to the last two (!) frames. ([0] = latest, [1] = the one before).


	void release();

	PointHessian(const ImmaturePoint* const rawPoint, CalibHessian* Hcalib);
    inline ~PointHessian() {assert(efPoint==0); release(); instanceCounter--;}

	//@ 判断其它帧上的点是否不值得要了
	inline bool isOOB(const std::vector<FrameHessian*>& toKeep, const std::vector<FrameHessian*>& toMarg) const
	{

		int visInToMarg = 0;
		for(PointFrameResidual* r : residuals)
		{
			if(r->state_state != ResState::IN) continue;  
			for(FrameHessian* k : toMarg)
				if(r->target == k) visInToMarg++;  // 在要边缘化掉的帧被观测的数量
		}
		//[1]: 原本是很好的一个点，但是边缘化一帧后，残差变太少了, 边缘化or丢掉
		if((int)residuals.size() >= setting_minGoodActiveResForMarg &&  // 残差数大于一定数目
				numGoodResiduals > setting_minGoodResForMarg+10 &&
				(int)residuals.size()-visInToMarg < setting_minGoodActiveResForMarg) //剩余残差足够少
			return true;



		//[2]: 最新一帧的投影在图像外了, 看不见了, 边缘化or丢掉
		// 或者满足以下条件,
		if(lastResiduals[0].second == ResState::OOB) return true;   //上一帧是OOB
		//[3]: 残差比较少, 新加入的, 不边缘化
		if(residuals.size() < 2) return false;	//观测较少不设置为OOB
		//[4]: 前两帧投影都是外点, 边缘化or丢掉
		if(lastResiduals[0].second == ResState::OUTLIER && lastResiduals[1].second == ResState::OUTLIER) return true; //前两帧都是外点
		return false;
	}

	//内点条件
	inline bool isInlierNew()
	{
		return (int)residuals.size() >= setting_minGoodActiveResForMarg
                    && numGoodResiduals >= setting_minGoodResForMarg;
	}

};





}

