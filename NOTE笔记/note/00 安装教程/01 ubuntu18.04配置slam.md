# 01 ubuntu18.04配置slam

## 1 安装前请看说明
0 本教程是基于ubuntu18.04环境，其他版本可能会出错，但具体步骤都是一样，遇到错误百度一下

1 只跑 orbslam 就安装 Eigen，OpenCV，Pangolin，（g2o，DBoW2 库源码有）

2 运行 slambook2 里面的所有代码，都需要安装

3 先安装 Eigen3 后才能安装 Sophus（Sophus 分模板和非模板，book2 是模板，book1 是非模板）

4 国内网络环境不友好，有梯子的先上个梯子，有梯子安装敢敢单单！！！本教程默认有梯子
    没梯子去百度网盘下载或者本人gitee仓库下载对应的源码：
	
[百度网盘地址](https://pan.baidu.com/s/15ZwYkGkkIczqcdAyEoW2KA) 提取码: 8t1f    
 源码位于该百度网盘目录下： 
   `./slam/安装包+opencv等三方库/slam第三方库/*`

[gitee仓库源码地址](https://gitee.com/linClubs/slam_lib)

从百度网盘和gitee仓库下载，建议使用鼠标点击下载， 然后右击解压；命令行下载和解压会出问题！！！
	
5 编译时，电脑卡，将make -j*代码的 j 后的参数改小一点，2，4，8，16，j 也行，直接make也行

----------------------------------------------------------

## 2.0 更换国内源

刚装的新系统18.04
1 ：更换国内源

CLI跟GUI2两种方式，[参考链接](https://zhuanlan.zhihu.com/p/61228593?ivk_sa=1024320u)

 ### CLI：

```text
# 首先备份源列表
sudo cp /etc/apt/sources.list /etc/apt/sources.list_backup
```
```text
# 打开sources.list文件
sudo gedit /etc/apt/sources.list
```

sources.list文件里添加清华源

```shell
# 清华源
deb https://mirrors.tuna.tsinghua.edu.cn/ubuntu/ bionic main restricted universe multiverse
deb-src https://mirrors.tuna.tsinghua.edu.cn/ubuntu/ bionic main restricted universe multiverse
deb https://mirrors.tuna.tsinghua.edu.cn/ubuntu/ bionic-updates main restricted universe multiverse
deb-src https://mirrors.tuna.tsinghua.edu.cn/ubuntu/ bionic-updates main restricted universe multiverse
deb https://mirrors.tuna.tsinghua.edu.cn/ubuntu/ bionic-backports main restricted universe multiverse
deb-src https://mirrors.tuna.tsinghua.edu.cn/ubuntu/ bionic-backports main restricted universe multiverse
deb https://mirrors.tuna.tsinghua.edu.cn/ubuntu/ bionic-security main restricted universe multiverse
deb-src https://mirrors.tuna.tsinghua.edu.cn/ubuntu/ bionic-security main restricted universe multiverse
deb https://mirrors.tuna.tsinghua.edu.cn/ubuntu/ bionic-proposed main restricted universe multiverse
deb-src https://mirrors.tuna.tsinghua.edu.cn/ubuntu/ bionic-proposed main restricted universe multiverse
```

```shell
bionic代表版本号，bionic代表18.04其他ubuntu需要替换版本号，百度一下
```
### GUI：
下面三张图
![[Pasted image 20220107134427.png]]
![[Pasted image 20220107134052.png]]
![[Pasted image 20220107134218.png]]
2
~~~shell
sudo apt update
sudo apt upgrade
~~~


## 2.1 eigen

[eigen官方下载链接](https://gitlab.com/libeigen/eigen/-/releases)

orbslam3用3.3版本的，本人orb3使用的是3.3.4版本。学习slam十四讲和orb2建议安装3.3以下的版本（3.2.10）
3.3.4为例：

```shell
wget https://gitlab.com/libeigen/eigen/-/archive/3.3.4/eigen-3.3.4.zip
unzip eigen-3.3.4.zip
cd eigen-3.3.4
mkdir build
cd build
cmake ..
make
sudo make install
```

查看eigen版本：
`sudo gedit /usr/include/eigen3/Eigen/src/Core/util/Macros.h`

-------------------------------------------------------------------

## 2.2 Pangolin

[github链接h](ttps://github.com/stevenlovegrove/Pangolin.git)

v6版本为例：

先安装依赖：
依赖一般有 Glew、CMake、 Boost 、Python2/Python3

------------------------------------
```shell
sudo apt-get install libglew-dev

sudo apt-get install cmake

sudo apt-get install libboost-dev libboost-thread-dev libboost-filesystem-dev

```
安装：
```shell
wget https://github.com/stevenlovegrove/Pangolin/archive/refs/tags/v0.6.tar.gz
tar -zxvf v0.6.tar.gz
cd Pangolin-0.6
mkdir build
cd build
cmake ..
make -j8
sudo make install
```

----------------------------------------------

## 2.3 安装opencv与opencv_contrib

下载opencv和contrib对应版本，不要安装会出错

[github链接h](https://github.com/opencv)

1 依赖

[compiler]

`sudo apt-get install build-essential`

[required]

```shell
sudo apt-get install cmake git libgtk2.0-dev pkg-config libavcodec-dev

libavformat-dev libswscale-dev

```

[optional]
安装libjasper-dev依赖会找不到，先执行下面代码：
```shell
sudo add-apt-repository "deb http://security.ubuntu.com/ubuntu xenial-security main"
sudo apt update
sudo apt install libjasper1 libjasper-dev
```

接着安装可选依赖：

```shell
sudo apt-get install python-dev python-numpy libtbb2 libtbb-dev libjpeg-dev

libpng-dev libtiff-dev libjasper-dev libdc1394-22-dev

```

2 安装
以3.4.16为例子，强烈建议上梯子，上梯子会省很多麻烦
没梯子的安装可以参考该视频，[视频链接](https://www.bilibili.com/video/BV17K4y1h7uK/)

下载opencv和contrib
将2个文件放在同一个目录下，并重新命名（去掉版本号），也可以不用重新命名
```shell
wget https://github.com/opencv/opencv/archive/3.4.16.zip
unzip 3.4.16.zip
mv opencv-3.4.16 ./opencv
wget https://github.com/opencv/opencv_contrib/archive/refs/tags/3.4.16.tar.gz
tar -zxvf 3.4.16.tar.gz
mv opencv_contrib-3.4.16 ./opencv_contrib
```

编译：
这些opencv_contrib的路径自己改成自己的，我放在~/software/下，和opencv同级

```shell
cd opencv
mkdir build
cd build
cmake -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=/usr/local -D WITH_VTK=ON .. D OPENCV_EXTRA_MODULES_PATH=~/software/opencv_contrib/modules/ ..
make -j8
sudo make install
```

查看opencv版本
`pkg-config --modversion opencv`

--------------------------------------------------

## 2.4 安装g2o

1 依赖：
```shell
sudo apt-get install cmake libeigen3-dev libsuitesparse-dev
sudo apt-get install qtdeclarative5-dev qt5-qmake libqglviewer-dev-qt5
```


下面演示从g2o官方拉取，
```shell
wget https://github.com/RainerKuemmerle/g2o/archive/refs/tags/20201223_git.tar.gz
tar -zxvf 20201223_git.tar.gz
mv g2o-20201223_git g2o
cd g2o
mkdir build
cd build
cmake ..
make -j8
sudo make install
```


### 由于orbslam2 源码自带g2o库，建议直接编译自带的

从g2o官网拉去的可能版本不兼容，可以导致编译出问题。2种途径：

1  下载ORB_SLAM2源码，编译里面的g2o库；

2 本人的[gitee上下载](https://gitee.com/linClubs/slam_lib/blob/master/g2o.zip), 用命令行操作下载解压有问题，用鼠标点击下载，然后然后右击解压到此处,解压后，编译安装跟上面命令行操作一样



------------------------------------
## 2.5 安装Sophus

先安装 Eigen3 后才能安装 Sophus（Sophus 分模版和非模版，book2 是模板，book1 是非模板）

这里安装的Sophus模版类

[github地址](https://github.com/strasdat/Sophus/tags)

```shell
wget https://github.com/strasdat/Sophus/archive/refs/tags/v1.0.0.tar.gz
tar -zxvf v1.0.0.tar.gz
cd Sophus-1.0.0
mkdir build
cd build
cmake ..
make -j8
sudo make install
```

## 2.6 ceres安装

[官网](http://www.ceres-solver.org/)
1 依赖
```shell
sudo apt-get install liblapack-dev libsuitesparse-dev libcxsparse3.1.2 libgflags-dev libgoogle-glog-dev libgtest-dev
```

可能会出现无法定位libcxsoarse3.1.2的问题
```shell
# 第一步，打开sources.list
 sudo gedit /etc/apt/sources.list
 
# 第二步，将下面的源粘贴到最上方sources.list
deb http:``//cz.archive.ubuntu.com/ubuntu trusty main universe

# 第三步，更新源
sudo apt-get update

# 第四步，重新输入依赖项安装命令安装依赖项
sudo apt-get install liblapack-dev libsuitesparse-dev libcxsparse3.1.2 libgflags-dev libgoogle-glog-dev libgtest-dev
```

2 编译安装
ceres1.14.0版本
```shell
wget https://github.com/ceres-solver/ceres-solver/archive/refs/tags/1.14.0.tar.gz
tar -zxvf 1.14.0.tar.gz
cd ceres-solver-1.14.0
mkdir build
cd build
cmake ..
make -j8
sudo make install
```

## 3 ROS1安装
ubuntu18.04支持的ros1版本为melodic, 上梯子，梯子没有，用手机热点，报错就百度一下
ros2安装foxy版本，只支持ubuntu20.04

在20.04下安装ros1后编译orbslam3，python版本会出问题，python环境切换了，还是遇到编译问题，太浪费时间了，直接选择了18.04,不想折腾了

[安装参考链接](https://www.guyuehome.com/34365)

添加中科大的ROS镜像源

```shell
sudo sh -c ‘. /etc/lsb-release && echo “deb http://mirrors.ustc.edu.cn/ros/ubuntu/ $DISTRIB_CODENAME main” > /etc/apt/sources.list.d/ros-latest.list’
```

配置key
```shell
sudo apt-key adv —keyserver hkp://keyserver.ubuntu.com:80 —recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
```

更新并安装完全版
```shell
sudo apt update
sudo apt install ros-melodic-desktop-full
sudo apt install ros-melodic-rqt
```

初始化rosdep

```shell
sudo rosdep init
rosdep update
```

sudo: rosdep：找不到命令,原因是缺少python-rosdep这个包，安装即可。
[修改报错参考链接](https://blog.csdn.net/weixin_44911075/article/details/114103319)
```shell
sudo apt-get install python-rosdep 
```
rosdep update 失败就多试几次


初始化环境
```shell
echo “source /opt/ros/melodic/setup.bash” >> ~/.bashrc$ source ~/.bashrc
```

安装额外依赖包
    
```shell
sudo apt-get install python-rosinstall python-rosinstall-generator python-wstool build-essential
 ```
 
 验证安装成功，小乌龟例子
 开三个终端分别执行下面三条命令：
 ```shell
roscore
```
 
 ```shell
rosrun turtlesim turtlesim_node
```

```shell
rosrun turtlesim turtle_teleop_key
```