# slam学习笔记

## 目录
- [slam简介](#slam简介)
- [三维空间刚体运动](#三维空间刚体运动)
- [Eigen基本使用](#Eigen基本使用)
- [李群和李代数](#李群和李代数)
- [相机与图像](#相机与图像)
- [非线性优化](#非线性优化)
- [特征点法](#特征点法)
- [直接法](#直接法)
- [协方差矩阵](#协方差矩阵)
- [卡尔曼滤波](#卡尔曼滤波)
- [图优化](#图优化)
- [回环检测](#回环检测)


## slam简介

### 引入

我在什么地方？--定位  
周围的环境是什么样？--建图

----
传感器：GPS，轮式编码器，激光雷达，惯性测量单元（Inertial Measurement Unit,IMU)，相机等

----
相机：单目（Monocular）,双目（Stereo）和深度相机（RGB-D)

单目：成本低，尺度不确定

### 经典视觉slam框架

* 传感器数据：在在视觉 SLAM 中主要为相机图像信息的读取和预处理。如果在机器人中，还可能有码盘、惯性传感器等信息的读取和同步。
* 前端--视觉里程计（Visual Odometry,VO)：视觉里程计任务是估算相邻图像间相机的运动，以及局部地图的样子。VO 又称为前端（Front End）。
* 后端--非线性优化(Optimization):后端接受不同时刻视觉里程计测量的相机位姿，以及回环检测的信息，对它们进行优化，得到全局一致的轨迹和地图。由于接在 VO 之后，又称为后端（Back End）
* 回环检测(Loop Closing):回环检测判断机器人是否曾经到达过先前的位置。如果检测到回环，它会把信息提供给后端进行处理。
* 建图(Mapping):它根据估计的轨迹，建立与任务要求对应的地图。

### SLAM问题的数学表述
运动方程
<div align="center">
    <img src="https://latex.codecogs.com/svg.image?x_{k}=f(x_{k-1},u_{k},w_{k})" title="https://latex.codecogs.com/svg.image?x_{k}=f(x_{k-1},u_{k},w_{k})" />
</div>

- $ u_{k} $为传感器的读数
- $ w_{k} $为噪声

观测方程

$$
z_{k, j}=h(y_{j}, x_{k}, v_{k, j})
$$

- 在$ x_{k} $位置看到路标点$ y_{i} $产生观测数据$ z_{k, j} $
- $ v_{k, j} $为观测噪声

### 编程基础

```cpp
#include <iostream>
using namespace std;

int main(int argc, char** argv){
    cout<<"Hello SLAM!"<<endl;
    return 0;
}
```
编译
```
g++ helloSLAM.cpp
```

CMakeLists.txt
```cmake
# 声明要求的 cmake 最低版本
cmake_minimum_required( VERSION 2.8 )

# 声明一个 cmake 工程
project( HelloSLAM )

# 添加一个可执行程序
# 语法：add_executable( 程序名 源代码文件 ）
add_executable( helloSLAM helloSLAM.cpp )

```

使用库
```cmake
cmake_minimum_required( VERSION 2.8 )

# 声明一个 cmake 工程
project( HelloSLAM )

add_library( hello_shared SHARED libHelloSLAM.cpp )

# 添加一个可执行程序
# 语法：add_executable( 程序名 源代码文件 ）
add_executable( useHello useHello.cpp )
target_link_libraries( useHello hello_shared )

```

<div align="right">
    <b><a href="#目录">↥ Back To Top</a></b>
</div>


## 三维空间刚体运动

### 旋转矩阵

内积



<div align="center">
    <img src="https://latex.codecogs.com/svg.image?a\cdot&space;b=a^{T}b=\sum_{i=1}^{3}a_{i}b_{i}=\left|a&space;\right|\left|b&space;\right|cos\left<a,b&space;\right>"   />
</div>

外积

<div align="center">
    <img src="https://latex.codecogs.com/svg.image?a\times&space;b=\begin{bmatrix}i&space;&j&space;&space;&k&space;&space;\\a_{1}&space;&a_{2}&space;&space;&a_{3}&space;&space;\\b_{1}&space;&b_{2}&space;&space;&b_{3}&space;&space;\\\end{bmatrix}=\begin{bmatrix}a_{2}b_{3}-a_{3}b_{2}&space;\\a_{3}b_{1}-a_{1}b_{3}\\a_{1}b_{2}-a_{2}b_{1}\\\end{bmatrix}=\begin{bmatrix}0&space;&-a_{3}&space;&space;&a_{2}&space;&space;\\a_{3}&space;&0&space;&space;&-a_{1}&space;&space;\\-a_{2}&space;&a_{1}&space;&space;&0&space;&space;\\\end{bmatrix}b\triangleq&space;a^{\land}b&space;" title="https://latex.codecogs.com/svg.image?a\times b=\begin{bmatrix}i &j &k \\a_{1} &a_{2} &a_{3} \\b_{1} &b_{2} &b_{3} \\\end{bmatrix}=\begin{bmatrix}a_{2}b_{3}-a_{3}b_{2} \\a_{3}b_{1}-a_{1}b_{3}\\a_{1}b_{2}-a_{2}b_{1}\\\end{bmatrix}=\begin{bmatrix}0 &-a_{3} &a_{2} \\a_{3} &0 &-a_{1} \\-a_{2} &a_{1} &0 \\\end{bmatrix}b\triangleq a^{\land}b " />
</div>

坐标系的欧式变换

<div align="center">
    <img src="https://latex.codecogs.com/svg.image?\begin{bmatrix}e_{1}&space;&e_{2}&space;&space;&e_{3}&space;&space;\\\end{bmatrix}\begin{bmatrix}&space;a_{1}\\&space;a_{2}\\a_{3}\\\end{bmatrix}=\begin{bmatrix}e_{1}'&space;&e_{2}'&space;&space;&e_{3}'&space;&space;\\\end{bmatrix}\begin{bmatrix}&space;a_{1}'\\&space;a_{2}'\\a_{3}'\\\end{bmatrix}" title="https://latex.codecogs.com/svg.image?\begin{bmatrix}e_{1} &e_{2} &e_{3} \\\end{bmatrix}\begin{bmatrix} a_{1}\\ a_{2}\\a_{3}\\\end{bmatrix}=\begin{bmatrix}e_{1}' &e_{2}' &e_{3}' \\\end{bmatrix}\begin{bmatrix} a_{1}'\\ a_{2}'\\a_{3}'\\\end{bmatrix}" />
    <br /><br /><img src="https://latex.codecogs.com/svg.image?\begin{bmatrix}&space;a_{1}\\&space;a_{2}\\a_{3}\\\end{bmatrix}=\begin{bmatrix}e_{1}^{T}e_{1}'&space;&e_{1}^{T}e_{2}'&space;&space;&e_{1}^{T}e_{3}'&space;&space;\\e_{2}^{T}e_{1}'&space;&e_{2}^{T}e_{2}'&space;&space;&e_{2}^{T}e_{3}'&space;&space;\\e_{3}^{T}e_{1}'&space;&e_{3}^{T}e_{2}'&space;&space;&e_{3}^{T}e_{3}'&space;&space;\\\end{bmatrix}\begin{bmatrix}&space;a_{1}'\\&space;a_{2}'\\a_{3}'\\\end{bmatrix}\triangleq&space;Ra'" title="https://latex.codecogs.com/svg.image?\begin{bmatrix} a_{1}\\ a_{2}\\a_{3}\\\end{bmatrix}=\begin{bmatrix}e_{1}^{T}e_{1}' &e_{1}^{T}e_{2}' &e_{1}^{T}e_{3}' \\e_{2}^{T}e_{1}' &e_{2}^{T}e_{2}' &e_{2}^{T}e_{3}' \\e_{3}^{T}e_{1}' &e_{3}^{T}e_{2}' &e_{3}^{T}e_{3}' \\\end{bmatrix}\begin{bmatrix} a_{1}'\\ a_{2}'\\a_{3}'\\\end{bmatrix}\triangleq Ra'" />
    <br /><br /><img src="https://latex.codecogs.com/svg.image?SO(n)=\begin{Bmatrix}R\in&space;\mathbb{R}^{n\times&space;n}|RR^{T}=I,det(R)=1\end{Bmatrix}" title="https://latex.codecogs.com/svg.image?SO(n)=\begin{Bmatrix}R\in \mathbb{R}^{n\times n}|RR^{T}=I,det(R)=1\end{Bmatrix}" />
    <br /><br /><img src="https://latex.codecogs.com/svg.image?a'=R^{-1}a=R^{T}a" title="https://latex.codecogs.com/svg.image?a'=R^{-1}a=R^{T}a" />
    <br /><br /><img src="https://latex.codecogs.com/svg.image?a'=Ra&plus;t" title="https://latex.codecogs.com/svg.image?a'=Ra+t" />
</div>


变换矩阵与齐次坐标
<div align="center">
    <img src="https://latex.codecogs.com/svg.image?b=R_{1}a&plus;t_{1},c=R_{2}b&plus;t_{2}" title="https://latex.codecogs.com/svg.image?b=R_{1}a+t_{1},c=R_{2}b+t_{2}" />
    <br /><br /><img src="https://latex.codecogs.com/svg.image?c=R_{2}(R_{1}a&plus;t_{1})&plus;t{2}" title="https://latex.codecogs.com/svg.image?c=R_{2}(R_{1}a+t_{1})+t{2}" />
    <br /><br /><img src="https://latex.codecogs.com/svg.image?\begin{bmatrix}&space;a'\\1\end{bmatrix}=\begin{bmatrix}R&space;&t&space;&space;\\0^{T}&space;&1&space;&space;\\\end{bmatrix}\begin{bmatrix}&space;a\\1\end{bmatrix}\triangleq&space;T\begin{bmatrix}&space;a\\1\end{bmatrix}" title="https://latex.codecogs.com/svg.image?\begin{bmatrix} a'\\1\end{bmatrix}=\begin{bmatrix}R &t \\0^{T} &1 \\\end{bmatrix}\begin{bmatrix} a\\1\end{bmatrix}\triangleq T\begin{bmatrix} a\\1\end{bmatrix}" />
    <br /><br /><img src="https://latex.codecogs.com/svg.image?SE(3)=\begin{Bmatrix}T=\begin{bmatrix}R&space;&t&space;&space;\\0^{T}&space;&1&space;&space;\\\end{bmatrix}\in&space;\mathbb{R}^{4\times&space;4}|R&space;\in&space;SO(3),t\in&space;\mathbb{R}^{3}\end{Bmatrix}" title="https://latex.codecogs.com/svg.image?SE(3)=\begin{Bmatrix}T=\begin{bmatrix}R &t \\0^{T} &1 \\\end{bmatrix}\in \mathbb{R}^{4\times 4}|R \in SO(3),t\in \mathbb{R}^{3}\end{Bmatrix}" />
    <br /><br /><img src="https://latex.codecogs.com/svg.image?T^{-1}=\begin{bmatrix}R^{T}&space;&-R^{T}t&space;&space;\\0^{T}&space;&1&space;&space;\\\end{bmatrix}" title="https://latex.codecogs.com/svg.image?T^{-1}=\begin{bmatrix}R^{T} &-R^{T}t \\0^{T} &1 \\\end{bmatrix}" />
</div>

### 旋转向量和欧拉角
旋转向量
```
方向与旋转轴一致，长度等于旋转角。（轴角）
```
欧拉角
```
绕物体的 Z 轴旋转，得到偏航角 yaw；
绕旋转之后的 Y 轴旋转，得到俯仰角 pitch；
绕旋转之后的 X 轴旋转，得到滚转角 roll。

万向锁问题（Gimbal Lock）:在俯仰角为±90◦ 时，第一次旋转与第三次旋转将使用同一个轴，使得系统丢失了一个自由度（由三次
旋转变成了两次旋转）。这被称为奇异性问题，在其他形式的欧拉角中也同样存在。
```
### 四元数
一个四元数 q 拥有一个实部和三个虚部
<div align="center">
    <img src="https://latex.codecogs.com/svg.image?q=q_{0}&plus;q_{1}i&plus;q_{2}j&plus;q_{3}k" title="https://latex.codecogs.com/svg.image?q=q_{0}+q_{1}i+q_{2}j+q_{3}k" />
    <br /><br /><img src="https://latex.codecogs.com/svg.image?\left\{\begin{matrix}&space;i^{2}=j^{2}=k^{2}=-1\\&space;ij=k,ji=-k\\&space;jk=i,kj=-i\\&space;ki=j,ik=-j\end{matrix}\right." title="https://latex.codecogs.com/svg.image?\left\{\begin{matrix} i^{2}=j^{2}=k^{2}=-1\\ ij=k,ji=-k\\ jk=i,kj=-i\\ ki=j,ik=-j\end{matrix}\right." />
</div>

由轴角$ n $, $ \theta $指定旋转，三维点p旋转后变成p'。

<div align="center">
    <img src="https://latex.codecogs.com/svg.image?p=[0,x,y,z]=[0,v]" title="https://latex.codecogs.com/svg.image?p=[0,x,y,z]=[0,v]" />
    <br /><br /><img src="https://latex.codecogs.com/svg.image?q=[cos\frac{\theta}{2},nsin\frac{\theta}{2}]" title="https://latex.codecogs.com/svg.image?q=[cos\frac{\theta}{2},nsin\frac{\theta}{2}]" />
    <br /><br /><img src="https://latex.codecogs.com/svg.image?p'=qpq^{-1}" title="https://latex.codecogs.com/svg.image?p'=qpq^{-1}" />
</div>

### 相似、仿射、射影变换
欧式变换（6自由度）

<div align="center">
    <img src="https://latex.codecogs.com/svg.image?T=\begin{bmatrix}R&space;&t&space;&space;\\0^{T}&space;&1&space;&space;\\\end{bmatrix}" title="https://latex.codecogs.com/svg.image?T=\begin{bmatrix}R &t \\0^{T} &1 \\\end{bmatrix}" />
</div>
相似变换（7自由度）

<div align="center">
    <img src="https://latex.codecogs.com/svg.image?T_{s}=\begin{bmatrix}sR&space;&t&space;&space;\\0^{T}&space;&1&space;&space;\\\end{bmatrix}" title="https://latex.codecogs.com/svg.image?T_{s}=\begin{bmatrix}sR &t \\0^{T} &1 \\\end{bmatrix}" />
</div>
仿射变换（12自由度）

<div align="center">
    <img src="https://latex.codecogs.com/svg.image?T_{A}=\begin{bmatrix}A&space;&t&space;&space;\\0^{T}&space;&1&space;&space;\\\end{bmatrix}" title="https://latex.codecogs.com/svg.image?T_{A}=\begin{bmatrix}A &t \\0^{T} &1 \\\end{bmatrix}" />
</div>
射影变换（15自由度）

<div align="center">
    <img src="https://latex.codecogs.com/svg.image?T_{P}=\begin{bmatrix}A&space;&t&space;&space;\\a^{T}&space;&v&space;&space;\\\end{bmatrix}" title="https://latex.codecogs.com/svg.image?T_{P}=\begin{bmatrix}A &t \\a^{T} &v \\\end{bmatrix}" />
</div>
<div align="right">
    <b><a href="#目录">↥ Back To Top</a></b>
</div>


## Eigen基本使用
CMakeLists.txt
```cmake
include_directories("/usr/include/eigen3")
```
各个模块

<!-- <div align="center">

| 模块 | 头文件 | 描述|
| :---: | :---: |  :---: |
| Core | #include<Eigen/Core> |Martix和Array类，基础的线性代数运算和数组操作 |
| Geometry | #include<Eigen/Geometry> |旋转、平移、缩放、2维和3维的各种变换 |
| LU | #include<Eigen/LU> |求逆，行列式，LU分解 |
| Cholesky | #include<Eigen/Cholesky> |LLT和DLT Cholesky分解 |
| Householder | #include<Eigen/Householder> |豪斯霍尔德变换，用于线性代数运算 |
| SVD | #include<Eigen/SVD> |SVD分解 |
| QR | #include<Eigen/QR> |QR分解 |
| Eigenvalues | #include<Eigen/Eigenvalues> |特征值，特征向量分解 |   
| Spares | #include<Eigen/Spares> |稀疏矩阵的存储和一些基本的线性运算 |
| Dense | #include<Eigen/Dense> |包括了Core/Geometry/LU/Cholesky/SVD/QR/Eigenvalues模块 |
| Eigen | #include<Eigen/Eigen> |包括Dense和Sparse(整合库) |
  
</div> -->

    
 运动相关
 ```
旋转矩阵（3 × 3）：Eigen::Matrix3d。
旋转向量（3 × 1）：Eigen::AngleAxisd。
欧拉角（3 × 1）：Eigen::Vector3d。
四元数（4 × 1）：Eigen::Quaterniond。
欧氏变换矩阵（4 × 4）：Eigen::Isometry3d。
仿射变换（4 × 4）：Eigen::Affine3d。
射影变换（4 × 4）：Eigen::Projective3d。
```
<div align="right">
    <b><a href="#目录">↥ Back To Top</a></b>
</div>

## 李群和李代数
    
## 相机与图像
 
<div align="right">
    <b><a href="#目录">↥ Back To Top</a></b>
</div>



## 非线性优化
 
<div align="right">
    <b><a href="#目录">↥ Back To Top</a></b>
</div>


## 特征点法
### 特征点
#### 常用特征点
- SIFT(尺度不变特征变换，Scale-Invariant Feature Transform) （1000个特征5228.7ms)
- SURF（1000个特征217.3ms)
- ORB（Oriented FAST and Rotated BRIEF）（1000个特征15.3ms)

#### 人工特征点的性质
- 可重复性（Repeatability）：相同的“区域”可以在不同的图像中被找到。
- 可区别性（Distinctiveness）：不同的“区域”有不同的表达。
- 高效率（Efficiency）：同一图像中，特征点的数量应远小于像素的数量。
- 本地性（Locality）：特征仅与一小片图像区域相关。


#### 特征点的构成
 - 关键点（Key-point）
 - 描述子（Descriptor）

#### ORB特征
- FAST角点

- BRIEF描述子（Binary Robust Independent Elementary Features）

#### 特征匹配
- 暴力匹配（Brute-Force Matcher）
- 快速近似最近邻（FLANN）

### 2D-2D：对极几何

### 三角测量

### 3D-2D：PnP

### 3D-3D：ICP


<div align="right">
    <b><a href="#目录">↥ Back To Top</a></b>
</div>


## 直接法
### 引入

### 光流（Optical Flow）

### 直接法（Direct Methods）

### 直接法优缺点总结

 
<div align="right">
    <b><a href="#目录">↥ Back To Top</a></b>
</div>


## 协方差矩阵

- 方差
- 
<div align="center">
 
  <br /><img src="https://latex.codecogs.com/svg.image?{\sigma&space;_{x}}^{2}=\frac{1}{n-1}\sum_{i=1}^{n}(x_{i}-\bar{x})^{2}" title="https://latex.codecogs.com/svg.image?{\sigma _{x}}^{2}=\frac{1}{n-1}\sum_{i=1}^{n}(x_{i}-\bar{x})^{2}" />
  
</div>

- 协方差


<div align="center">
 
  <br /><img src="https://latex.codecogs.com/svg.image?\sigma(x,y)=\frac{1}{n-1}\sum_{i=1}^{n}(x_{i}-\bar{x})(y_{i}-\bar{y})" title="https://latex.codecogs.com/svg.image?\sigma(x,y)=\frac{1}{n-1}\sum_{i=1}^{n}(x_{i}-\bar{x})(y_{i}-\bar{y})" />
    <br /><br /><img src="https://latex.codecogs.com/svg.image?Cov(X,Y)=E[(X-\bar{x})(Y-\bar{y})]" title="https://latex.codecogs.com/svg.image?Cov(X,Y)=E[(X-\bar{x})(Y-\bar{y})]" />
  
</div>

- 相关系数

<div align="center">
 
  <br /><img src="https://latex.codecogs.com/svg.image?\rho&space;=\frac{Cov(X,Y)}{\sigma&space;_{X}\sigma_{Y}}" title="https://latex.codecogs.com/svg.image?\rho =\frac{Cov(X,Y)}{\sigma _{X}\sigma_{Y}}" />
  
</div>

- 协方差矩阵

<div align="center">
 
  <br /><img src="https://latex.codecogs.com/svg.image?\sigma(x_{m},x_{k})=\frac{1}{n-1}\sum_{i=1}^{n}(x_{mi}-\bar{x_{m}})(x_{ki}-\bar{x_{k}})" title="https://latex.codecogs.com/svg.image?\sigma(x_{m},x_{k})=\frac{1}{n-1}\sum_{i=1}^{n}(x_{mi}-\bar{x_{m}})(x_{ki}-\bar{x_{k}})" />
    <br /><br /><img src="https://latex.codecogs.com/svg.image?\sum=\begin{bmatrix}\sigma(x_{1},x_{1})&space;&...&space;&space;&\sigma(x_{1},x_{d})&space;&space;\\...&space;&...&space;&space;&...&space;&space;\\\sigma(x_{d},x_{1})&space;&...&space;&space;&\sigma(x_{d},x_{d})&space;&space;\\\end{bmatrix}\in&space;\mathbb{R}^{d\times&space;d}" title="https://latex.codecogs.com/svg.image?\sum=\begin{bmatrix}\sigma(x_{1},x_{1}) &... &\sigma(x_{1},x_{d}) \\... &... &... \\\sigma(x_{d},x_{1}) &... &\sigma(x_{d},x_{d}) \\\end{bmatrix}\in \mathbb{R}^{d\times d}" />
  
</div>

- 信息矩阵

<div align="center">
    <br /><img src="https://latex.codecogs.com/svg.image?\Omega&space;=\Sigma&space;^{-1}" title="https://latex.codecogs.com/svg.image?\Omega =\Sigma ^{-1}" />
</div>


<div align="right">
    <b><a href="#目录">↥ Back To Top</a></b>
</div>

## 卡尔曼滤波
* [卡尔曼滤波推导](https://blog.csdn.net/qq_25458977/article/details/111597163?ops_request_misc=%257B%2522request%255Fid%2522%253A%2522165277069516780366540200%2522%252C%2522scm%2522%253A%252220140713.130102334..%2522%257D&request_id=165277069516780366540200&biz_id=0&utm_medium=distribute.pc_search_result.none-task-blog-2~all~sobaiduend~default-1-111597163-null-null.142^v10^pc_search_result_control_group,157^v4^control&utm_term=%E5%8D%A1%E5%B0%94%E6%9B%BC%E6%BB%A4%E6%B3%A2%E5%8D%8F%E6%96%B9%E5%B7%AE%E7%9F%A9%E9%98%B5&spm=1018.2226.3001.4187)

 
<div align="right">
    <b><a href="#目录">↥ Back To Top</a></b>
</div>


## 图优化
 
<div align="right">
    <b><a href="#目录">↥ Back To Top</a></b>
</div>


## 回环检测
 
<div align="right">
    <b><a href="#目录">↥ Back To Top</a></b>
</div>


