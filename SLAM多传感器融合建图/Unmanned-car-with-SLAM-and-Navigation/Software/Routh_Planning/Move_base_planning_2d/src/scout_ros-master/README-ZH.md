# 松灵机器人产品SCOUT 的ROS package

## ROS Packages 说明

* scout_bringup: launch and configuration files to start ROS nodes 
* scout_base: a ROS wrapper around Scout SDK to monitor and control the robot
* scout_msgs: scout related message definitions
* (scout_ros: meta package for the Scout robot ROS packages)


下图是是整个ros package的一个基本框架说明，或许它可以帮助你理解你理解整个ros package内部是如何工作的，他们之间的是相互联系的。
其中最底层的是移动机器人底盘，它通过can或者usart实现运行在计算平台的sdk进行基本信息的获取，具体可以根据wrp_sdk了解更多信息。 仿真部分是基于Webots，构建起的仿真环境。

<img src="./docs/diagram.png" height="135" >

其中上图中紫色部分是包含在这个ros-package中的部分。

## 通讯接口设置

### 设置串口

通常来说，usb转串口设备在Linux或者Ubuntu系统中会被自动识别为“/dev/ttyUSB0” 或者看起来类似的设备，这个可以通过指令查询
```
$ ls -l /dev/ttyUSB*
```
如果在打开设备的操作过程中出现了"... permission denied ..."的错误，有可能因为权限的原因造成的，您需要向您的用户帐户授予端口访问权限，可以通过如下指令：

```
$ sudo usermod -a -G dialout $USER
```

需要重新登录账户才能使刚刚的操作生效。
### 配置 CAN-TO-USB适配器

1.  设置CAN转USB适配器，启用gs_usb内核模块（本指令需要搭配相应的硬件设备才可以使用，需要Linux内部版本>4.5）
   
    ```
    $ sudo modprobe gs_usb
    ```

2. 设置can设备参数
   
   ```
   $ sudo ip link set can0 up type can bitrate 500000
   ```

3. 如果在前面的步骤中没有发生错误，您可以使用以下指令查看can设备
   
   ```
   $ ifconfig -a
   ```

4. 安装和使用can-utils来测试硬件
   
    ```
    $ sudo apt install can-utils
    ```

5. 测试指令
   
    ```
    # receiving data from can0
    $ candump can0
    # send data to can0
    $ cansend can0 001#1122334455667788
    ```

文件中提供了 "./scripts"文件夹里的两个脚本以便于设置。您可以在首次安装时运行“ ./setup_can2usb.bash”，并在每次拔出和重新插入适配器时运行“ ./bringup_can2usb.bash”以启动设备。

##  ROS package 的基础使用

1. 安装 ROS packages 依赖

    ```
    $ sudo apt install ros-melodic-teleop-twist-keyboard
    ```

    如果你使用是 Kinetic版本，把上述指令中的“melodic” 改成 “kinetic” 即可

2. 将scout ros package 下载至的您的catkin 工作空间，并编译

    (下面的操作是假定你的catkin编译工作空间在: ~/catkin_ws/src 目录下）

    ```
    $ cd ~/catkin_ws/src
    $ git clone --recursive https://github.com/agilexrobotics/ugv_sdk.git
    $ git clone https://github.com/agilexrobotics/scout_ros.git
    $ cd ..
    $ catkin_make
    ```
    如果你买的小车版本是1.0通信协议版本，执行以下指令切换ugv_sdk的版本至1.0版本
    ```
    $ cd ugv_sdk && git checkout master
    ```
    然后重新编译

3. 启动 ROS nodes

* 启动scout车节点 

    ```
    $ roslaunch scout_bringup scout_minimal.launch
    ```
    
* 启动scout-mini车节点 

    ```
    $ roslaunch scout_bringup scout_mini_minimal.launch
    ```

* Start the keyboard tele-op node

    ```
    $ roslaunch scout_bringup scout_teleop_keyboard.launch
    ```

    **SAFETY PRECAUSION**: 

    The default command values of the keyboard teleop node are high, make sure you decrease the speed commands before starting to control the robot with your keyboard! Have your remote controller ready to take over the control whenever necessary. 
