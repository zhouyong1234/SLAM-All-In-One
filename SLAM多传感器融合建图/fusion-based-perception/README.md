# 深蓝学院多传感器融合感知课程

项目实现了Lidar与Camera的后融合感知算法，融合的算法基于扩展卡尔曼滤波(Extended Kalman Filter,EKF)。输入数据为Lidar检测结果以及Camera检测结果，检测算法与Apollo 6.0一致，Lidar检测算法为PointPillars，Camera检测算法为YOLO。
该项目可以扩展至Lidar, Camera, Radar3种传感器的感知融合。

## 编译环境

平台：ubuntu ≥ 16.04

ROS版本 ≥ kinetic

## 使用方法

使用catkin_make编译代码并source所在的工作空间

使用以下命令启动可视化

```shell
$ roslaunch kit_perception rviz.launch
```

提供了两个数据集供测试使用，分别为单物体数据集和多物体数据集

使用以下命令使用单物体数据集启动程序

```shell
$ roslaunch kit_perception single_obj_fusion.launch
```

使用以下命令使用多物体数据集启动程序

```shell
$ roslaunch kit_perception mutil_obj_fusion.launch
```

### 可视化说明

在rviz内蓝色物体为融合结果，白色物体为雷达观测
