# **rs_driver** 

## 1 工程简介

  **rs_driver**为速腾聚创跨平台的雷达驱动内核，方便用户二次开发使用。

### 1.1 雷达型号支持

- RS16
- RS32
- RSBP
- RS128
- RS80
- RSM1-B3
- RSHELIOS



## 2 编译与安装

**rs_driver**目前支持下列系统和编译器：

- Windows
  - MSVC  (VS2017 & VS2019 已测试)
  - Mingw-w64 (x86_64-8.1.0-posix-seh-rt_v6-rev0 已测试)

- Ubuntu (16.04, 18.04, 20.04)
  - gcc (4.8+)

### 2.1 依赖库的安装

**rs_driver** 依赖下列的第三方库，在编译之前需要先安装的它们：

- Boost
- pcap
- PCL (非必须，如果不需要可视化工具可忽略)

- Eigen3 (非必须，如果不需要内置坐标变换可忽略)

#### 2.1.1 Ubuntu中的依赖库安装

```shell
sudo apt-get install libboost-dev libpcap-dev libpcl-dev libeigen3-dev
```

#### 2.1.2 Windows下的依赖库安装

##### Boost

Windows下需要从源码编译Boost库，请参考[官方指南](https://www.boost.org/doc/libs/1_67_0/more/getting_started/windows.html)。编译安装完成之后，将Boost的路径添加到系统环境变量```BOOST_ROOT```。

如果使用MSVC，也可以选择直接下载相应版本的预编译的[安装包](https://boost.teeks99.com/)。

##### Pcap

首先，安装[pcap运行库](https://www.winpcap.org/install/bin/WinPcap_4_1_3.exe)。

然后，下载[开发者包](https://www.winpcap.org/install/bin/WpdPack_4_1_2.zip)到任意位置，然后将```WpdPack_4_1_2/WpdPack``` 的路径添加到环境变量```PATH```。

##### PCL

*注：如果不编译可视化工具，可不编译安装PCL库*

(1) MSVC

如果使用MSVC编译器，可使用PCL官方提供的[安装包](https://github.com/PointCloudLibrary/pcl/releases)安装。

安装过程中选择 “Add PCL to the system PATH for xxx”:

![](./doc/img/install_pcl.PNG)

(2) Mingw-w64

PCL官方并没有提供mingw编译的库，所以需要按照[官方教程](https://pointclouds.org/documentation/tutorials/compiling_pcl_windows.html), 从源码编译PCL并安装。




## 3 使用方式

### 3.1 安装使用

*注：在Windows中，**rs_driver** 暂不支持安装使用。*

安装驱动

```sh
cd rs_driver
mkdir build && cd build
cmake .. && make -j4
sudo make install
```

使用时，需要在```CMakeLists```文件中使用find_package()指令找到**rs_driver**，然后链接相关库。

```cmake
find_package(rs_driver REQUIRED)
include_directories(${rs_driver_INCLUDE_DIRS})
target_link_libraries(project ${rs_driver_LIBRARIES})
```

### 3.2 作为子模块使用

在```CMakeLists```文件中将**rs_driver**作为子模块添加到工程内，使用find_package()指令找到**rs_driver**，然后链接相关库。

```cmake
add_subdirectory(${PROJECT_SOURCE_DIR}/rs_driver)
find_package(rs_driver REQUIRED)
include_directories(${rs_driver_INCLUDE_DIRS})
target_link_libraries(project ${rs_driver_LIBRARIES})
```



## 4 快速上手

[在线连接雷达](doc/howto/how_to_online_use_driver.md)

[解析pcap包](doc/howto/how_to_decode_pcap.md)



## 5 示例程序 & 可视化工具

### 5.1 示例程序

**rs_driver**提供了两个示例程序，存放于```rs_driver/demo``` 中：

- demo_online.cpp
- demo_pcap.cpp

用户可参考示例程序编写代码调用接口。若希望编译这两个示例程序，执行CMake配置时加上参数：

```bash
cmake -DCOMPILE_DEMOS=ON ..
```

### 5.2 可视化工具

**rs_driver**提供了一个基于PCL的点云可视化工具，存放于```rs_driver/tool``` 中：

- rs_driver_viewer.cpp

若希望编译可视化工具，执行CMake配置时加上参数：

```bash
cmake -DCOMPILE_TOOLS=ON ..
```

具体使用请参考[可视化工具操作指南](doc/howto/how_to_use_rs_driver_viewer.md) 



## 6 坐标变换

 **rs_driver**提供了内置的坐标变换功能，可以直接输出经过坐标变换后的点云，节省了用户对点云进行坐标变换的额外操作耗时。若希望启用此功能，执行CMake配置时加上参数：

```bash
cmake -DENABLE_TRANSFORM=ON ..
```

具体使用请参考[坐标变换功能简介](doc/howto/how_to_use_transformation_function.md) 



## 7 其他资料

请根据以下路径去查看相关文件。

- 参数定义: ```rs_driver/src/rs_driver/driver/driver_param.h```
- 点云消息定义: ```rs_driver/src/rs_driver/msg/point_cloud_msg.h```
- 接口定义: ```rs_driver/src/rs_driver/api/lidar_driver.h```
- 错误码定义: ```rs_driver/src/rs_driver/common/error_code.h```

组播模式:  [组播模式](doc/howto/how_to_use_multi_cast_function.md) 