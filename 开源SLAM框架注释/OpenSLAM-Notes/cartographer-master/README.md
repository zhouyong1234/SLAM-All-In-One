# cartographer 代码分析 

![ed9c66bb-9beb-46e4-bb9d-c7bb56511c3b.png](https://storage.live.com/items/24342272185BBA7E!4900?authkey=AJzdbBYZIQ_AuAo)


代码主要分为两个部分，其一为cartographer的核心实现，另一个为cartographer的ros封装壳。首先介绍其ros封装，可以看到大概的调用流程，然后再深入源码去剖析其实现过程。但是其代码可以说十分的繁琐且复杂，在只能大致理清楚其逻辑。

## Cartographer-ROS 

根据运行的命令 `roslaunch cartographer_ros offline_backpack_2d.launch bag_filenames:=${HOME}/Downloads/b2-2016-04-05-14-44-52.bag`可以知道，运行文件`offline_backpack_2d.launch`启动算法。

* 加载配置文件 `backpack_2d.lua`
* 调用launch文件 `offline_node.launch`

因此，深入文件 `offline_node.launch`

* 运行节点 `rviz`
* 运行节点 `cartographer_occupancy_grid_node`
* 运行节点 `cartographer_offline_node`

因此对于cartographer-ros来说，主要的节点就是这两个，其在如下文件中分别实现

* `offine_node_main.cc`
* `occupancy_grid_node.cc`

### occupancy_grid_node

![1c165611-d49b-437a-82c1-8b90f1c165dc.png](https://storage.live.com/items/24342272185BBA7E!4901?authkey=AJzdbBYZIQ_AuAo)

主要实现的功能有
* 新建一个定时器，定时发布全局地图信息
* 构造回调函数，在回调函数内处理子地图列表信息

其子列表信息回调函数流程如下：

* 设置所有之前的子地图为待删除子地图
* 在待删除子地图中去掉当前子地图列表中还存在的
* 获取新子地图信息 `FetchSubmapTextures`
  * 发布一个srv，获取压缩后的子地图栅格信息
  * 对栅格地图信息进行解压
* 转换子地图信息格式

而发布流程则为

* 将所有子地图构造成一个图片(调用cairo实现)
* 转换为ROS格式并发布 

### offine_node_main 

在main函数中

* 调用函数`CreateMapBuilder`构造了一个`MapBuilder`类
* 调用`RunOfflineNode`函数，传入`MapBuilder`类指针

其中，`RunOfflineNode`函数则为离线运行节点的主要逻辑

![a804f097-6dbd-42a6-a137-1d9c762d1067.png](https://storage.live.com/items/24342272185BBA7E!4902?authkey=AJzdbBYZIQ_AuAo)

可以从图中看到，函数主要工作为
* 调用`AddOffineTrajectory()`函数，实际上该函数主要调用的是`map_builder`类的`AddTrajectoryBUilder`函数。
* 创建ROS相关的发布以及订阅消息的处理，并定时发布可视化信息
* 构造`SensorBridge`类，并处理传感器数据,实际上则是调用`TrajectoryBuilderInterface`类的传感器数据处理
* 读取参数配置文件，保存地图信息等其他工作

从输入输出的角度来看整体的代码，我们需要弄清楚本节点发布的ROS数据具体有哪些，是如何获得的，并且是如何处理这些传感器数据。

**输入**
* 传递传感器数据到核心cartographer代码
直接由`SensorBridge`类使用`TrajectoryBuilderInterface`类指针调用其传感器数据处理函数。
**输出**
* 发布消息到ROS 
  * 定时发送轨迹、子地图列表、约束项等信息
  * 在请求时返回子地图的栅格地图数据,调用函数`HandleSubmapQuery`，实际上调用的是`map_builder_->SubmapToProto`函数获取压缩后的栅格信息

因此，整理如下

* 调用`map_builder`类的`AddTrajectoryBuilder`函数,进行初始化
* 调用`TrajectoryBuilderInterface`类相关的传感器处理函数处理传感器信息
* 调用`map_builder_->SubmapToProto`函数等获取处理结果

最终，我们可以看到实际上交互的内容并不多，在获取栅格地图信息上由于数据量较大进行了数据压缩，其他的都是直接通过指针获取得到数据。因此我们接下来看主体代码时只要集中在前面两点上即可。

## Cartographer 主体

根据上文结论，接下来分为两个部分进行介绍。

### 调用`map_builder`类的`AddTrajectoryBuilder`函数

这里根据配置参数的选择，才有了2D激光数据和3D激光数据的区别，根据不同的传感器数据配置2D或者3D对应的类，由于在代码上没有太大的区别逻辑都是几乎一模一样的（在处理IMU数据上有一些区别，3D激光必须要IMU,2D可以不要）。因此后面都是以2D类举例解释。

1. 构造局部轨迹生成类 `LocalTrajectoryBuilder2D `
2. 使用局部轨迹生成类，构造全局轨迹生成类 `GlobalTrajectoryBuilder`
3. 使用全局轨迹生成类，构造轨迹生成管理类指针 `CollatedTrajectoryBuilder`
4. 将轨迹生成管理类指针保存在一个vector中
 
![a2a9b290-5b44-4c96-aa8e-27bbe5add24c.png](https://storage.live.com/items/24342272185BBA7E!4903?authkey=AJzdbBYZIQ_AuAo)

这里就非常的绕，需要仔细思考，所有的轨迹生成类均为`TrajectoryBuilderInterface`的子类，因此需要特别注意，使用`TrajectoryBuilderInterface`类指针是具体指向的是哪一个子类的实现。**最后返回的是`CollatedTrajectoryBuilder`类的指针，因此下面调用的是该之类的传感器数据处理函数。**

* 其他工作
  * 纯定位模式的配置
  * 初始位置的设置 

###  调用`TrajectoryBuilderInterface`类函数处理传感器数据

由上面分析可知这里的`TrajectoryBuilderInterface`类是父类，而实际调用的是子类`CollatedTrajectoryBuilder`。

#### `CollatedTrajectoryBuilder`类

主要功能：
* 构造`Collator`类，管理所有的传感器数据，设置回调函数为`HandleCollatedSensorData()`,在`Collator`类内所有的传感器数据都被表达成通用的传感器数据结构。
* 调用`Collator`类处理传感器数据，在将传感器数据转换成通用数据后，调用回调函数`HandleCollatedSensorData()`
* 在回调函数中调用全局轨迹生成类`GlobalTrajectoryBuilder`对传感器数据进行处理

#### `GlobalTrajectoryBuilder`类

在全局轨迹生成类中将传感器数据进行了分类，其中激光雷达数据、IMU数据和里程计数据用于生成局部轨迹，其他的传感器数据如GPS信息、路标点信息等则直接被添加到了位姿图优化类`PoseGraph2D`类中。

* 针对里程计、IMU和激光信息，调用局部路径生成`LocalTrajectoryBuilder2D`类进行处理
* 处理结束后将其添加到`PoseGraph2D`类中进行优化
* 将其他传感器数据同样添加到`PoseGraph2D`类中进行优化

因此，具体的实现部分在`LocalTrajectoryBuilder2D`类中添加传感器数据部分，以及`PoseGraph2D`类中添加传感器数据作为节点部分。

#### `LocalTrajectoryBuilder2D`类

* 构造`PoseExtrapolator`位姿外推类，类似于卡尔曼滤波器的功能，利用之前的传感器信息和当前激光采集时间，预测当前位置和速度。
* 根据预测速度，对激光数据进行运动畸变矫正
* 调用`CeresScanMatcher2D`类，进行激光数据前端匹配的计算当前帧与当前子地图的位置关系。
* 激光达到一定距离则调用`submap_2d`类插入到子地图中
* 利用匹配结果更新`PoseExtrapolator`的估计，为下一次做准备

其中，CSM前端匹配算法是，子地图-激光帧的匹配。具体原理部分可参考原论文，这里的重点不是理论。另外`submap_2d`类插入激光数据到子地图中，对子地图进行了管理，具体的子地图管理策略为。

![ca49fc0c-34c1-4e8a-a1a1-d0252af40310.png](https://storage.live.com/items/24342272185BBA7E!4904?authkey=AJzdbBYZIQ_AuAo)

其中，激光数据插入地图调用的是`probability_grid_range_data_inserter_2d`类中的函数，其原理为占用栅格地图更新原理，可参考注释和相关资料理解，这里不再赘述。

#### `PoseGraph2D`类

![c5c34f33-4a7a-490b-9678-6c8edf2fdf5d.png](https://storage.live.com/items/24342272185BBA7E!4905?authkey=AJzdbBYZIQ_AuAo)

* 在收到局部轨迹生成类得到的子地图后，首先添加到构造的优化问题`OptimizationProblem2D`类中。
* 然后调用`fast_correlative_scan_matcher_2d.cc`文件中的算法进行回环，主要就是利用分支定界算法在一定大小的窗口内进行搜索匹配。
* 回环检测结束后，无论是否成功都将优化求解问题添加到线程池中
* 其他传感器数据采集到也会将优化问题添加到线程池中进行求解

分支定界方法用于寻找回环约束的具体实现与原理较为复杂，可以参考论文和代码注释进行学习，这同样不是本文的重点。最后，将所有传感器的数据添加到`OptimizationProblem2D`类中构造了后端优化问题，接下来我们将求解这样一个最终的优化问题。

#### `OptimizationProblem2D`类

优化问题的求解调用函数`OptimizationProblem2D::Solve`，其主要流程和普通的Ceres优化流程没有什么区别，就是按照Ceres的套路来。其中最为关键问题在于图优化中的节点和边如何构造以及误差函数的计算，这里我们采用因子图的方式表达这样一个图模型。

![f596373b-b182-47cd-8d94-dcf0cc68fa17.png](https://storage.live.com/items/24342272185BBA7E!4906?authkey=AJzdbBYZIQ_AuAo)

最后我们看一下误差函数,定义在`SpaCostFunction2D`类中，非常非常的简单，就是优化结果不能和之前匹配得到的相对位置相差太多。

```
  bool operator()(const T* const start_pose, const T* const end_pose,
                  T* e) const {
    // 误差计算函数  
    // ScaleError 基于旋转和平移项不同的权重，保证收敛
    // ComputeUnscaledError 真正计算误差的函数
    const std::array<T, 3> error =
        ScaleError(ComputeUnscaledError(
                       transform::Project2D(observed_relative_pose_.zbar_ij),
                       start_pose, end_pose),
                   observed_relative_pose_.translation_weight,
                   observed_relative_pose_.rotation_weight);
	// 保存误差
    std::copy(std::begin(error), std::end(error), e);
    return true;
  }
```
这里里程计的误差函数和匹配的误差函数是相同的，可以认为结果是两个里程计的可变加权和，其他的误差函数如路标点等这里不再过多展开，都是一样的。

至此，我们将cartographer代码的整体流程过了一遍，其原理不难，但是源码过于复杂，其中还有许许多多的细节，需要花费大量时间仔细阅读才能真正熟悉它。

