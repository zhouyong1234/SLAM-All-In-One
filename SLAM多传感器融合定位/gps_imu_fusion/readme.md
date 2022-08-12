### Forked from https://github.com/zm0612/eskf-gps-imu-fusion

# 卡尔曼融合IMU与GPS数据
结果如下：

![融合IMU数据之后的GPS轨迹效果](/data/raw_data1/results/trajectory.png)
![融合IMU数据之后的GPS轨迹效果](/data/raw_data1/results/xyz_view.png)

蓝色轨迹：ground truth（图片看不清）

红色轨迹：fuse IMU and GPS

绿色轨迹：GPS

已经实现的滤波方法：

EKF：[基于导航信息的EKF滤波算法实现（附源码）](https://blog.csdn.net/qq_38650944/article/details/123594568?spm=1001.2014.3001.5502)

基于高精度IMU模型的ESKF（fork代码的原作者的实现，这里表示感谢）：[【附源码+代码注释】误差状态卡尔曼滤波(error-state Kalman Filter)，扩展卡尔曼滤波，实现GPS+IMU融合，EKF ESKF GPS+IMU](https://blog.csdn.net/u011341856/article/details/114262451)

Joan Sola大神的Quaternion kinematics for the error-state KF：[Quaternion kinematics for error state kalman filter实现GPS+IMU融合，（附源码）](https://blog.csdn.net/qq_38650944/article/details/123580686)

TODO： 

UKF滤波，准备参考[无损卡尔曼滤波UKF与多传感器融合](https://blog.csdn.net/Young_Gy/article/details/78542754)，后面有时间补上

欢迎大家交流呀！
## 1.  依赖库

Eigen

```shell
sudo apt-get install libeigen3-dev
```

Yaml

```shell
sudo apt-get install libyaml-cpp-dev
```

## 2. 编译

```shell
cd eskf-gps-imu-fusion
mkdir build
cd build
cmake ..
make 
```

## 3. 运行
本代码现在支持EKF，基于高精度IMU模型的ESKF和Joan Sola大神的Quaternion kinematics for the 
error-state KF。调试好的数据有两组。

如果想尝试不同的方法和不同的数据，只需要修改config.yaml里面的配置文件即可

```shell
cd eskf-gps-imu-fusion/build
./gps_imu_fusion
```

## 4.轨迹显示

执行完`./gps_imu_fusion`会生成轨迹文件

```shell
cd eskf-gps-imu-fusion/data
evo_traj kitti fused.txt gt.txt measured.txt -p
```

> 需要安装evo，可以参考博客中的介绍：https://blog.csdn.net/u011341856/article/details/104594392?spm=1001.2014.3001.5501


