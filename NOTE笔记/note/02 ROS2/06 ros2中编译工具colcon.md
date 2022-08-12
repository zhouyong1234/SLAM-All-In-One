# colcon 扣阔嗯
## 6.1 colcon是啥
colcon功能包构建工具
用来编译代码的，ros2没有安装colcon

## 6.2 安装colcon
`sudo apt-get install python3-colcon-common-extensions`
## 6.3 创建工作空间
```
mkdir -p colcon/src
cd colcon/src
git clone https://github.com/ros2/examples src/examples -b foxy
``` 
-b 指明分支,git仓库中
### git下载慢，采用代理下载
https://ghproxy.com
`git clone https://ghproxy.com/https://github.com/ros2/examples src/examples -b foxy`
-b foxy表示分支
## 6.4 编译
```
cd ~/colcon_ws
colcon build
source ~/colcon/install/setup.bash   #让系统能找到该节点，跟devel一样的效果
```
运行一个订阅者：
`ros2 run examples_rclcpp_minimal_subscriber subscriber_member_function`
发布者：
`ros2 run examples_rclcpp_minimal_publisher publisher_member_function`

### 6.4.1 只编译一个功能包
`colcon build --packages-select YOUR_PKG_NAME`
`colcon build --packages-select turtlesim`

## 6.5 python代码
允许通过更改src下的部分文件来改变install（重要）

每次调整 python 脚本时都不必重新build了
`colcon build --symlink-install`

因为ros2的build没有ros中的devel概念了，如果想达到devel目录那样的效果，就需要加这个参数。没有学过ros1的请主动忽略这句话。