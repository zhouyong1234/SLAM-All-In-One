 

IMU-相机标定

--单目标定，双目标定，IMU标定，IMU-双目联合标定

  

**目录**

[1  安装Kalibr与code_utils、imu_utils 4](#_Toc1169612097 )

[1.1 imu_utils编译（用于标定imu） 4](#_Toc1035380398 )

[1.1.1  注意事项 4](#_Toc96208487 )

[1.1.2  编译 5](#_Toc106423263 )

[1.2  录制IMU数据（使用安卓手机录制） 5](#_Toc764847441 )

[1.2.1  IMU录制说明： 6](#_Toc1183082882 )

[1.2.2  打开手机app，查看rostopic list 6](#_Toc675139129 )

[1.2.3  录制imu 8](#_Toc1215139395 )

[1.2.4  查看包信息 8](#_Toc94164771 )

[1.2.5  播放bag数据 8](#_Toc501790916 )

[1.3  Kalibr安装 9](#_Toc569292269 )

[1.3.1 安装依赖环境 9](#_Toc656846988 )

[1.3.2  下载Kalibr功能包 10](#_Toc1924247384 )

[1.3.3  报错信息 10](#_Toc925432021 )

[2  IMU标定 11](#_Toc588970771 )

[2.3  IMU标定 11](#_Toc1467243884 )

[2.4 IMU 标定说明： 11](#_Toc2073974004 )

[2.4.1 启动my_imu.lanuch 11](#_Toc553047572 )

[2.4.1 标定结果 13](#_Toc1563242797 )

[3 kalibr 标定相机 14](#_Toc1619610511 )

[3.1 kalibr   标定说明 14](#_Toc320367084 )

[3.1.1  生成标定格 14](#_Toc1470913297 )

[3.1.2  拍摄标定板 18](#_Toc1268560252 )

[3.1.3 相机话题降频 19](#_Toc986323454 )

[3.1.3  相机标定 19](#_Toc1118430081 )

[3.2  单目标定 22](#_Toc1394690692 )

[3.2.1  OpenCV代码标定 22](#_Toc32219100 )

[3.2.2  kalibr标定单目(kalibr好像不能进行单目标定) 22](#_Toc1688339417 )

[3.3  双目标定 23](#_Toc984474539 )

[3.3.1 ros 工具包标定 23](#_Toc769666670 )

[3.3.2  kalibr双目标定 24](#_Toc298546588 )

[4  IMU与双目联合标定 29](#_Toc6602988 )

[4.1  录制数据 29](#_Toc1805047068 )

[4.2  查看话题的频率 29](#_Toc394755075 )

[4.3  数据集进行imu_stereo标定 32](#_Toc113026251 )

[4.4  标定结果 33](#_Toc422410862 )

  

# **1 安装Kalibr与****code_utils****、****imu_utils**

3个包都新建工作空间进行编译，kalibr编译比较久（20-30分钟左右），建议先编译后2个

参考链接：[https://zhuanlan.zhihu.com/p/39881693](https://zhuanlan.zhihu.com/p/39881693)7 具体操作看下面

## **1.1** **imu_utils编译（用于标定imu）**

参考链接：https://www.codeleading.com/article/55921249470/

imu_utils下载地址为：[https://github.com/gaowenliang/imu_utils](https://github.com/gaowenliang/imu_utils)

code_utils下载地址为： [https://github.com/gaowenliang/code_utils](https://github.com/gaowenliang/code_utils)

### **1.1.1 注意事项**

在编译之前我们先注意三个问题，避免入坑

（1）全局安装**ceres库**，code_imu依赖ceres。**注意先安装ceres（本人安装的ceres1.14）库依赖与eigen3.2的版本**

（2）**不要同时把imu_utils和code_utils一起放到src下进行编译**。

**由于imu_utils 依赖 code_utils，所以先把code_utils放在工作空间的src下面，进行编译。然后再将imu_utils放到src下面**，再编译。

否则会报如下错误：

![[wps1.jpg]]

**3 编译报错：**

![[wps2.jpg]]

**安装ceres后安装依赖：**

**sudo apt-get install libdw-dev**

### **1.1.2编译**

![[wps3.jpg]]

## **1.2 录制IMU数据（使用安卓手机录制）**

**安卓发布camera与IMU数据参考：**

[https://www.cnblogs.com/MingruiYu/p/12404730.html](https://www.cnblogs.com/MingruiYu/p/12404730.html)（**该链接含有单目标定**）

**录制imu.bag**

先roscore再启动安卓上的app，不然app会黑屏，（失败）

录制IMU时，将IMU传感器**静止**放置2个小时左右，

——————————————————————————-

**roscore**

打开手机app，查看rostopic list

**rostopic list**

**rosbag** **record** **/android/im****u ~****/output/imu_bag/imu_data**

**其他传感器发布的imu数据直接用rosbag record 录制就行**

### **1.2.1 IMU录制说明：**

启动ros

**roscore**

### **1.2.2 打开手机app，查看rostopic list**

**rostopic list**

可以看到/android/imu(imu数据)与/android/image_raw/compressed（图像数据），/android/camera_info(相机参数rostopic echo /android/camera_info)

![[wps4.jpg]]

**rostopic echo /****android/****camera_info**

安卓相机参数如下：（安卓相机变焦的，畸变也处理了）

![[wps5.jpg]]

**rostopic echo /android/imu**

查看imu数据，如下图

![[wps6.jpg]]

### **1.2.3 录制imu**

rosbag record topic_name -o 文件名

**rosbag** **record** **/android/im****u ~****/output/imu_bag/imu_data**

两个小时以后按Ctrl+C完成录制。

![[wps7.jpg]]

### **1.2.4查看包信息**

**rosbag info imu_data_2021-11-09-19-30-28.bag**

### **1.2.5播放bag数据**

**rosbag play -r 200** **/home/lin/output/imu_bag****/imu.bag** 

-r 频率  后面的数大一些，就快一些（可以不要）, 最后面是你录制的包的路径

![[wps8.jpg]]

## **1.3 Kalibr安装**

参考链接https://blog.csdn.net/hltt3838/article/details/116064390

**错误！未找到图形项目表。**

### **1.3.1****安装依赖环境**

**sudo apt-get install python-setuptools python-rosinstall ipython libeigen3-dev libboost-all-dev doxygen libopencv-dev**

**sudo apt-get install ros-melodic-vision-opencv ros-melodic-image-transport-plugins ros-melodic-cmake-modules software-properties-common**

**sudo apt-get install libpoco-dev python-matplotlib python-scipy python-git python-pip ipython libtbb-dev libblas-dev liblapack-dev**

**sudo apt-get install python-catkin-tools libv4l-dev**

**sudo pip install python-igraph --upgrade**

**很多依赖包已经以前已经装好了，个别问题百度一下，我基本没装**

### **1.3.2 下载Kalibr功能包**

**mkdir -p ~/kalibr_ws/src**

**cd ~/kalibr_ws/src**

**cd ~/kalibr_workspace/src**

**git clone https://github.com/ethz-asl/Kalibr.git**

**cd ~/kalibr_workspace**

**catkin build -DCMAKE_BUILD_TYPE=Release -j4**

### **1.3.3 报错信息**

1 no matching function for call to  ‘getOptimalNewCameraMatrix’

解决的办法：（其他人的很麻烦）

在文件 kalibr/aslam_cv/aslam_imgproc/include/aslam/implementation/aslamcv_helper.hpp 里面，将"CvSize" 改为 "cv::Size"  即可(**总共修改3处)**

**其他一些小问题基本都可以百度解决。**

  

# **2 IMU标定**

imu_utils编译好后（用于标定imu）

IMU需要标定的参数主要是**确定性误差**和**随机误差，**确定性误差主要标定**bias，scale和misalignment，**随机误差主要标定**noise和random walk**，**imu_utils是用于求取****随机误差**的开源工具。

## **2.3 IMU标定**

参考链接https://blog.csdn.net/sinat_25923849/article/details/107867407

**先录制IMU（参考1.2），本人测试IMU运动的数据，标定不出来，****IMU静止录制数据**

**-------------------------------------------------------------------**

**cd imu_ws source ./devel/setup.bash**

**roslaunch imu_utils my_imu.launch**

**发布imu数据**

**先运行上面程序，再打开新终端执行下面，上面这条程序会等一会才结束**

**rosbag play -r 200 imu_utils/imu.bag (这里要写你录制的包的路径)**

**----------------------------------------------------**

## **2.4 IMU标定说明：**

### **2.4.1启动my_imu.lanuch**

**roslaunch imu_utils my_imu.launch**

![[wps9.jpg]]

**wiat for imu data****，这里还没发布imu，发布运行完后还需要****等一会才标定结束****，不是程序卡了。**如果是先发布数据再launch启动，就重新发布一次

注意这里的launch文件,我用安卓手机（/android/imu**其他传感器的换对应topic**），所以my_imu.launch的内容为：

![[wps10.jpg]]

launch里面imu_topic设置成对应的，imu_name设置随便设置（生成文件名字），data_save_path默认是当前的路径下data目录下

这里有一个max_time_min表示使用bag数据的最大时长，单位是分钟，默认是120分钟，程序会在最大时间截断读取数据。

### **2.4.1标定结果**

在**imu_utils/data里面生成了my_imu_param.yaml文件**，还有生成了一下其他文件my_imu*(acc、gyr等)

![[wps11.jpg]]

![[wps12.jpg]]

  

# **3 kalibr标定相机**

**kalibr****标定结果**当中，Kalibr输出的内参格式为：**fx，fy，cx，cy**，畸变参数为：**k1，k2，p1，p2**（径向畸变参数k1、k2，切向畸变参数p1，p2）

**ROS标定**工具直接输出了内参矩阵**K=[fx, 0, cx; 0, fy, cy; 0, 0, 1]**，畸变参数为**[k1, k2, p1, p2, k3]**

**OpenCV**当中利用内参矩阵和畸变参数进行图像校正时，采用的是五位畸变参数，**与ROS标定的畸变参数格式**相同。但是径向畸变的参数k1、k2、k3当中，k3的影响较小，**ROS标定工具的输出中k3均为0，**所以在利用Kalibr标定后，直接在畸变参数中最后一位加0变成五位即可调用OpenCV校正。

**------------------------------------------------------------------**

**cd kalibr_ws**

**source devel/setup.sh**

**kalibr_create_target_pdf --type apriltag** **--nx 8 --ny 6 --tsize 0.1 --tspace 0.1**

**参数说明在3.1.1**

**------------------------------------------------------------------**

## **3.1 kalibr标定说明**

### **3.1.1生成标定格**

kalibr支持3种标定板：四月格、棋盘格、圆形格

[https://github.com/ethz-asl/kalibr/wiki/calibration-targets](https://github.com/ethz-asl/kalibr/wiki/calibration-targets)

![[wps13.jpg]]

代码：生成标定板（四月格）

**cd kalibr_ws**

**source devel/setup.sh**

**kalibr_create_target_pdf --type apriltag** **--nx 8 --ny 6 --tsize 0.1 --tspace 0.1**

**kalibr_create_target_pdf**(具体参数解释见上面图片)

**april****grid.yaml配置文件**

target_type: 'aprilgrid' #gridtype

tagCols: 6               #number of apriltags

tagRows: 6               #number of apriltags

tagSize: 0.088           #size of apriltag, edge to edge [m]

tagSpacing: 0.3          #ratio of space between tags to tagSize

 #example: tagSize=2m, spacing=0.5m --> tagSpacing=0.25[-]

报错信息：

No module named pyx

![[wps14.jpg]]

执行：

**sudo apt-get install python-pyx**

![[wps15.jpg]]

再执行：

**kalibr_create_target_pdf --type apriltag** **--nx 8 --ny 6 --tsize 0.1 --tspace 0.1**

![[wps16.jpg]]

生成pdf在当前目录下，ls查看，target.pdf如下图

![[wps17.jpg]]

![[wps18.jpg]]

kalibr_create_target_pdf --type apriltag --nx 6 --ny 6 --tsize 0.088 --tspace 0.3

**april****grid.yaml配置文件**

target_type: 'aprilgrid' #gridtype

tagCols: 6               #number of apriltags

tagRows: 6               #number of apriltags

tagSize: 0.088           #size of apriltag, edge to edge [m]

tagSpacing: 0.3          #ratio of space between tags to tagSize

 #example: tagSize=2m, spacing=0.5m --> tagSpacing=0.25[-]

**生成棋盘格**

**kalibr_create_target_pdf --type checkerboard --nx 6 --ny 6 --csx 0.03 --csy 0.02**

![[wps19.jpg]]

使用以下配置支持标准棋盘模式：

**checkerboard.yaml**

target_type: 'checkerboard' #gridtype

targetCols: 6               #number of internal chessboard corners

targetRows: 7               #number of internal chessboard corners

rowSpacingMeters: 0.06      #size of one chessboard square [m]

colSpacingMeters: 0.06      #size of one chessboard square [m]

![[wps20.jpg]]

圆形格子自行百度一下，四月格精度高一点，棋盘格是常用的，建议用四月格

#### **3.1.1.1标定板配置信息说明**

参数详细链接:

[https://blog.csdn.net/u010368556/article/details/86245616](https://blog.csdn.net/u010368556/article/details/86245616)

![[wps21.jpg]]

![[wps22.jpg]]

### **3.1.2拍摄标定板**

打开安卓手机摄像头，对准标定板录制数据bag，**不断变化手机与标定板位置**

**--------------------------------------------**

**rosbag record -O stereo_calibra.bag [topic] [topic]**

**我运行的(安卓单目标定)：**

**rosbag record -****o** **mono****_calibra.bag** **/android/image_raw/compressed**

**-----------------------------------**

topic根据自己发布的rostopic对应，可以同时录多个topic，图像数据（进行双目、多目标定），IMU数据（camera—IMU外参标定），后面不在叙述数据录制

而且应当保持多个相机具有相对固定的安装位置，整个相机系统保持不动，移动标定板来录制 rosbag，

### **3.1.3** **相机话题降频**

rosrun topic_tools throttle messages /YOUR_IAMGE_TOPIC0 4.0 /image0

rosrun topic_tools throttle messages /YOUR_IAMGE_TOPIC1 4.0 /image1

降频后的话题分别是 /image0  /image1  ... ，更改了话题名称以便区分

### **3.1.3 相机标定**

**kalibr_calibrate_cameras --target data/kalibr_sample/static/april_6x6.yaml --bag data/kalibr_sample/static/static.bag --models pinhole-equi pinhole-equi omni-radtan omni-radtan --topics /cam0/image_raw /cam1/image_raw /cam2/image_raw /cam3/image_raw**

------------------------------------------

报错：No module named igraph:  下页有报错信息修改

![[wps23.jpg]]

这里的相机模型选择 **pinhole-radtan** 是最常用的相机模型，包括了径向畸变和切向畸变；排列顺序中相机模型与相机话题对应，顺序和数量都必须一致，否则会报错。

**--bag filename.bag**

包含数据的ROS包（上一步录制的图片数据包）

**--topics TOPIC_0 ... TOPIC_N袋子**

中所有相机主题的列表。匹配--models的顺序

**--models MODEL_0 ... MODEL_N**

要安装的相机/失真模型列表。匹配--topics的顺序（请参阅支持的模型）

[https://github.com/ethz-asl/kalibr/wiki/supported-models](https://github.com/ethz-asl/kalibr/wiki/supported-models)

![[wps24.jpg]]

**--target target.yaml** 

校准目标配置（请参阅“ 校准目标”）**（4.1中生成的标定板参数）**

报错：No module named igraph: ** 报错信息修改，如下图：**

![[wps25.jpg]]

  

## **3.2 单目标定**

### **3.2.1 OpenCV代码标定**

opencv代码例子：[https://www.cnblogs.com/MingruiYu/p/12404730.html](https://www.cnblogs.com/MingruiYu/p/12404730.html)

### **3.2.2 kalibr标定单目(****kalibr好像不能进行单目标定****)**

**想到了一个方法**，可以参考3.3的双目（先看双目标定3.3节），录制一个双目（左右都是同一个视频流进行标定）

，录制参考3.3.3节

666666^-^   居然成功了，

-----------------------------------

![[wps26.jpg]]录制数据如下：

  

### **3.2.3 结果显示**

运行代码：

-------------------------------------------------

kalibr_calibrate_cameras --topics /left /cam0/image_raw --bag ~/output/imu_bag/mono_2021-11-11-11-25-40.bag --bag-from-to 2 4 --target imu_bag/dynamic/april_6x6.yaml --models pinhole-radtan pinhole-radtan

----------------------------------------------------

![[wps27.jpg]]运行报错可能大家参数没给正确，或者数据录制有问题

**可以看出T_cn_cnm1:的旋转部分为单位阵，平移部分为0向量，标定基本正确。**

**但是不推荐使用上面方法，录制视频麻烦。建议直接使用opencv、matlab、ROS自带的标注。**

  

### **3.2.4 ros标定**

已电脑摄像头为例子

自行百度安装ros启动电脑摄像功能包。

-----------------------------------------------

启动

**roscore**

启动相机：

**roslaunch usb_cam usb_cam-test.launch**

启动标定程序

**rosrun camera_calibration cameracalibrator.py --siz8x6 --square 0.02 image:=/usb_cam/image_raw camera:=/usb_cam**

![[wps28.jpg]]

![[wps29.jpg]]不断移动标定板，采集了40张不同位置的图像后，左边CALIBRATE图标会变亮，点击该图标进行标定。

再次移动标定板，直到save与commit图标被点亮，依次点击2个图标。

![[wps30.jpg]]

保存结果保存在：

![[wps31.jpg]]

**cd /tmp**

移动到你想保存的路径下

**mv calibrationdata.tar.gz /output/**

解压：

**tar -xzf calibrationdata.tar.gz**

![[wps32.jpg]]里面除了采集的40张图片，还有2个结果文件，其中一个：

  

## **3.3 双目标定**

参考链接：https://blog.csdn.net/luoshi006/article/details/80035342

### **3.3.1 ros工具包标定**

1 首先启动摄像头程序；

2 启动 camera_calibration：

--------------------------------------------------------

**rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.03 --no-service-check --approximate=0.1 right:=/cam1/image_raw left:=/cam0/image_raw right_camera:=/cam1 left_camera:=/cam0**

**--------------------------------------------------------------------**

--size 8x6 注意是字母 x，他是棋盘格内部角点个数。  
--square 0.03 ，表示棋盘格为正方形，边长3cm。

指定数据话题：可以rostopic list查看，切记不要左右搞错了

right：left：指定话题

right_camera、left_camera指定左右相机

![[wps33.jpg]]

### **3.3.2 kalibr双目标定**

--------------------------------------------------------

**kalibr_calibrate_cameras --bag ~/output/imu_bag/dynamic/dynamic.bag --topics /cam0/image_raw /cam1/image_raw --models pinhole-equi pinhole-equi --target ~/output/imu_bag/dynamic/april_6x6.yaml**

**--------------------------------------------------------------------**

**1 采集数据**：双目对着标定板进行拍摄，双目直接的相对位置不能变，不断移动，旋转标定板。

这里利用数据集的包进行标定验证，下面会用小觅相机采集数据进行标定，从https://pan.baidu.com/s/1bWQT7g提取码2g2t下载数据包，解压放在工作空间里

运行该数据集查看rostopic：

![[wps34.jpg]]

可以看出有/cam0/image_raw, /cam1/image_raw, /imu0三个话题；

由于该数据有单目1700多张图片数据，全部使用标定时间过长，所以采用录制一段进行标定：

### **3.3.3录制代码**

**先roscore,**每段代码开个终端：下面两句：topic发布频率降低至4HZ，话题名改为最后的/left或者/right。

rosrun topic_tools throttle messages /cam0/image_raw 4.0 /left

advertised as /left

rosrun topic_tools throttle messages /cam1/image_raw 4.0 /right

advertised as /right

发布：播放上面下载的数据集：play后跟路径

rosbag play ~/output/imu_bag/dynamic/dynamic.bag

录制：record后是话题名称这里录制左右话题，-o后是保存路径

rosbag record /left /right -o ~/output/imu_bag/stereo.bag

我只运行了10秒左右，终端上面的代码，生成新的数据集stereo.bag进行测试。

**我运行的标定代码：**

**-------------------------------------------**

**cd kalibr_ws/**

source devel/setup.sh

**kalibr_calibrate_cameras --bag ~/output/imu_bag/stereo_2021-11-10-23-11-20.bag --topics /left /right --models pinhole-equi pinhole-equi --target ~/output/imu_bag/dynamic/april_6x6.yaml**

**-----------------------------------------**

**上面代码的参数参考3.1.3**

![[wps35.jpg]]

**结果显示：生成了3个文件：**

![[wps36.jpg]![[wps37.jpg]]

文件1

camchain-homelinoutputimu_bagstereo_2021-11-10-23-11-20.yaml

![[wps38.jpg]]

![[wps39.jpg]]参数说明链接：https://blog.csdn.net/u010368556/article/details/86245616

文件2

![[wps40.jpg]]results-cam-homelinoutputimu_bagstereo_2021-11-10-23-11-20.txt

文件3

report-cam-homelinoutputimu_bagstereo_2021-11-10-23-11-20.pdf

![[wps41.jpg]]

  

# **4 IMU与相机联合标定**

## **4.1 IMU与双目联合标定**

[https://blog.csdn.net/heyijia0327/article/details/83583360/](https://blog.csdn.net/heyijia0327/article/details/83583360/)

在**标定完双目内外参数以后, 可以接着标定和 IMU 之间的外参数**了. 这次采用同样的标定板, **采集数据的最佳频率为图像20Hz, IMU 200 Hz (当然其他频率也没问题).**

### **4.1.1 录制数据**

采用上面的双目标定时使用的数据集，录制含imu的数据。

**采用传感器实时录制时，注意事项：**

采集数据的起始和结束阶段注意别晃动太大，如从桌子上拿起或者放下。如果有这样的动作，在标定阶段应该跳过bag数据集的首尾的数据.

采集数据的时候应该给imu各个轴足够的激励，如先依次绕各个轴运动，运动完后来个在空中画8字之类的操作，当然也要注意别运动太剧烈，图像都模糊了。

![[wps42.jpg]]

**rviz展示下载数据集的某一帧（双目）**

### **4.1.2查看话题的频率**

**，**image采用4HZ，imu就采用40hz

rosrun rqt_topic rqt_topic

![[wps43.jpg]]

**运行代码参考3.3.2中的录制数据：如下图**

![[wps44.jpg]]

**联合标定代码：**

https://blog.csdn.net/mxdsdo09/article/details/83514310

**kalibr_calibrate_imu_camera --target xx/target.yaml --cam xx/camchain.yaml --imu xx/imu.yaml --bag xx/xx.bag --bag-from-to 5 45**

即可完成标定。（**5-45表示取的标定数据长度****，开始部分数据和结束部分可能会有晃动，把它去除**）在背后加上--time—calibration可以标定IMU相对于camera的延时。

**我运行的代码，自己根据路径修改：**

我用的原始数据：不用录制，因为下面代码有--bag-from-to 4 45（标定数据长度）

---------------------------------------------------

**kalibr_calibrate_imu_camera --target ~/output/imu_bag/dynamic/april_6x6.yaml --cam ~/kalibr_ws/output/camchain-homelinoutputimu_bagstereo_imu_2021-11-11-00-22-45.yaml --imu ~/kalibr_ws/src/imu_utils/data/my_imu_imu_param.yaml --bag ~/output/imu_bag/stereo_imu_2021-11-11-00-22-45.bag --bag-from-to 4 45**

**---------------------------------------**

**报错：**

![[wps45.jpg]]

**sudo apt-get install python-scipy**

**--------------------------------------------------------**

### **4.1.3 数据集进行imu_stereo标定**

参考链接：https://www.it610.com/article/1294697911111720960.htm

**由于之前标定的IMU文件是安卓手机的，这里不使用，直接使用数据集里面的标定参数。**

这里利用数据集的包进行标定验证，下面会用小觅相机采集数据进行标定，从https://pan.baidu.com/s/1bWQT7g提取码2g2t下载数据包，解压:查看文件目录：

![[wps46.jpg]]

直接在上面的目录下打开终端运行下面代码，如下图所示：

**代码：**

**kalibr_calibrate_imu_camera --target april_6x6.yaml --cam camchain.yaml --imu imu_adis16448.yaml --bag dynamic.bag --bag-from-to 5 6**

![[wps47.jpg]]

**终于跑完！！！   ^-^ 实属不易**

### **4.1.4 标定结果**

**运行完成后结果：**

![[wps48.jpg]]

![[wps49.jpg]]

![[wps50.jpg]]

**camchain-imucam-dynamic.yaml文件说明**

**参数说明参考链接：**

https://blog.csdn.net/u010368556/article/details/86245616

![[wps51.jpg]]

![[wps52.jpg]]

  

## **4.2 IMU与单目联合标定**

calib可以标定单目+imu，bag内只放单目图像，修改带参运行中的参数即可。

### **4.2.1数据制作**

录制单目+imu数据集参考4.1.1节。（left与imu0）

录制代码：

rosbag record /left /imu -o mono_imu.bag

### **4.2.2 标定**

标定代码：

--------------------------------

**kalibr_calibrate_imu_camera --target april_6x6.yaml --cam mono.yaml --imu imu.yaml --bag ../mono_imu_2021-11-11-10-15-37.bag --bag-from-to 5 40**

**-----------------------------**

配置文件说明：

1 mono.yaml 单目配置文件；标定时修改对应的参数

![[wps53.jpg]]

----------------------------------------------

cam0:

 camera_model: pinhole # 基于小孔成像原理的相机

 intrinsics: [461.487246372674, 460.1113992557959, 356.39105303227853, 231.15719697054647] # [fx,fy,u,v]

 distortion_model: equidistant #畸变模型

 distortion_coeffs: [-0.0016509958435871643, 0.02437222940989351, -0.03582816956989852,

 0.019860839087717054] #畸变系数

 resolution: [752,480]

 rostopic: /left

-----------------------------------------------------------

2 imu.yaml配置文件

标定时修改对应的参数，修改对应的参数

![[wps54.jpg]]

--------------------------------------------

rostopic: /imu

update_rate: 200.0 #Hz

accelerometer_noise_density: 0.01 #continous

accelerometer_random_walk: 0.0002

gyroscope_noise_density: 0.005 #continous

gyroscope_random_walk: 4.0e-06

![[wps55.jpg]]----------------------------------------

  

### **4.2.3 结果显示**

生成了4个文件

![[wps56.jpg]]

camchain-imucam-..mono_imu_2021-11-11-10-15-37.yaml

![[wps57.jpg]]