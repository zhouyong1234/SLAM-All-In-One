# AMR软件

## Calib

### 参考博客

[goldqiu：十四.激光和惯导LIO-SLAM框架学习之惯导内参标定](https://zhuanlan.zhihu.com/p/434710744)

[goldqiu：十五.激光和惯导LIO-SLAM框架学习之惯导与雷达外参标定（1）](https://zhuanlan.zhihu.com/p/434718435)

[goldqiu：二十.激光、视觉和惯导LVIO-SLAM框架学习之相机内参标定](https://zhuanlan.zhihu.com/p/446297673)

[goldqiu：二十一.激光、视觉和惯导LVIO-SLAM框架学习之相机与雷达外参标定（1）](https://zhuanlan.zhihu.com/p/446297974)

### 测试框架和功能包

#### Lidar_IMU标定

[here](https://github.com/goldqiu/Unmanned-car-with-SLAM-and-Navigation/tree/main/Software/Calib/Lidar_imu/lidar_align_imu)

#### SBG_IMU内参标定

标定误差和偏置

[here](https://github.com/goldqiu/Unmanned-car-with-SLAM-and-Navigation/tree/main/Software/Calib/SBG_IMU/imu_calib)

标定重力加速度

[here](https://github.com/goldqiu/Unmanned-car-with-SLAM-and-Navigation/tree/main/Software/Calib/SBG_IMU/imu_gravity)

## Localization

### 参考博客

[goldqiu：二十五.SLAM中Mapping和Localization区别和思考](https://zhuanlan.zhihu.com/p/494959811)

[goldqiu：一.全局定位--开源定位框架LIO-SAM_based_relocalization实录数据集测试](https://zhuanlan.zhihu.com/p/494961228)

[goldqiu：二.全局定位--开源定位框架livox-relocalization实录数据集测试](https://zhuanlan.zhihu.com/p/496006244)

## Mapping

### 参考博客

[goldqiu：一：Tixiao Shan最新力作LVI-SAM(Lio-SAM+Vins-Mono)，基于视觉-激光-惯导里程计的SLAM框架，环境搭建和跑通过程](https://zhuanlan.zhihu.com/p/369154727)

[goldqiu：二.激光SLAM框架学习之A-LOAM框架---介绍及其演示](https://zhuanlan.zhihu.com/p/423077984)

[goldqiu：三.激光SLAM框架学习之A-LOAM框架---项目工程代码介绍---1.项目文件介绍（除主要源码部分）](https://zhuanlan.zhihu.com/p/423245638)

[goldqiu：四.激光SLAM框架学习之A-LOAM框架---项目工程代码介绍---2.scanRegistration.cpp--前端雷达处理和特征提取](https://zhuanlan.zhihu.com/p/423300393)

[goldqiu：五.激光SLAM框架学习之A-LOAM框架---项目工程代码介绍---3.laserOdometry.cpp--前端雷达里程计和位姿粗估计](https://zhuanlan.zhihu.com/p/423323274)

[goldqiu：六.激光SLAM框架学习之A-LOAM框架---项目工程代码介绍---4.laserMapping.cpp--后端建图和帧位姿精估计（优化）](https://zhuanlan.zhihu.com/p/423348129)

[goldqiu：八.激光SLAM框架学习之LeGO-LOAM框架---框架介绍和运行演示](https://zhuanlan.zhihu.com/p/427840280)

[goldqiu：十.激光SLAM框架学习之LeGO-LOAM框架---算法原理和改进、项目工程代码](https://zhuanlan.zhihu.com/p/429096191)

[goldqiu：十一.激光惯导LIO-SLAM框架学习之LIO-SAM框架---框架介绍和运行演示](https://zhuanlan.zhihu.com/p/433113761)

[goldqiu：十二.激光SLAM框架学习之livox-loam框架安装和跑数据集](https://zhuanlan.zhihu.com/p/432520314)

[goldqiu：十七.激光和惯导LIO-SLAM框架学习之IMU和IMU预积分](https://zhuanlan.zhihu.com/p/437378667)

[goldqiu：十九.激光和惯导LIO-SLAM框架学习之项目工程代码介绍---代码框架和一些文件解释](https://zhuanlan.zhihu.com/p/444858817)

[goldqiu：二十二.香港大学火星实验室R3LIVE框架跑官方数据集](https://zhuanlan.zhihu.com/p/456548828)

[goldqiu：二十三.激光和惯导LIO-SLAM框架学习之LIO-SAM项目工程代码介绍---基础知识](https://zhuanlan.zhihu.com/p/445862584)

[goldqiu：二十四-香港大学火星实验室FAST-LIO2框架跑官方数据集](https://zhuanlan.zhihu.com/p/481546652)

### 测试框架

[A-LOAM](https://github.com/goldqiu/Unmanned-car-with-SLAM-and-Navigation/tree/main/Software/Mapping/%E6%B5%8B%E8%AF%95%E6%A1%86%E6%9E%B6/A-LOAM)

[gmapping](https://github.com/goldqiu/Unmanned-car-with-SLAM-and-Navigation/tree/main/Software/Mapping/%E6%B5%8B%E8%AF%95%E6%A1%86%E6%9E%B6/gmapping)

[lego-LOAM](https://github.com/goldqiu/Unmanned-car-with-SLAM-and-Navigation/tree/main/Software/Mapping/%E6%B5%8B%E8%AF%95%E6%A1%86%E6%9E%B6/lego-LOAM)

[LIO-SAM](https://github.com/goldqiu/Unmanned-car-with-SLAM-and-Navigation/tree/main/Software/Mapping/%E6%B5%8B%E8%AF%95%E6%A1%86%E6%9E%B6/LIO-SAM)

[Livox_mapping](https://github.com/goldqiu/Unmanned-car-with-SLAM-and-Navigation/tree/main/Software/Mapping/%E6%B5%8B%E8%AF%95%E6%A1%86%E6%9E%B6/Livox_mapping)

## Middleware

### Map_Conversion

[here](https://github.com/goldqiu/Unmanned-car-with-SLAM-and-Navigation/tree/main/Software/Middleware/Map_Conversion)

## Routh_Planning

### 参考博客

[goldqiu：一.路径规划---二维路径规划仿真实现-gmapping+amcl+map_server+move_base](https://zhuanlan.zhihu.com/p/455852721)

[goldqiu：二.路径规划---二维路径规划实车实现---gmapping+amcl+map_server+move_base](https://zhuanlan.zhihu.com/p/457770455)

### 框架

[Move_base_planning_2d](https://github.com/goldqiu/Unmanned-car-with-SLAM-and-Navigation/tree/main/Software/Routh_Planning/Move_base_planning_2d)

年前调通的move_base基本功能，采用的是gmapping+amcl+move_base+map_server

## 脚本文件

[scout_robot.sh](https://github.com/goldqiu/Unmanned-car-with-SLAM-and-Navigation/blob/main/Software/scout_robot.sh)

