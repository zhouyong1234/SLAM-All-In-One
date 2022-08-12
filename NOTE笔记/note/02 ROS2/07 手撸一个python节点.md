工作空间>功能包>节点
## 7.1 创建工作空间

![[Screenshot from 2021-12-27 18-46-15.png]]

![[Screenshot from 2021-12-27 18-48-24.png]]

创建python节点
![[Screenshot from 2021-12-27 18-50-36.png]]


工作空间就行一个文件夹
```
mkdir -p colcon_ws/src
cd colcon/src
code ./ 						#用vscode打开
```

跟ros1不同，ros1还需要初始化工作空间步骤

## 7.2 创建功能包
`ros2 pkg create <name> --build-type --dependencies<依赖名称>`

创建一个名叫billage_li(李家村)的的python功能包

`ros2 pkg create village_li --build-type ament_python --dependencies rclpy`
`tree` 查看目录结构
```
--build-type 指定该包的编译类型 三种：ament_python;ament_cmake;cmake
		     不写build-type默认ament_cmake
--dependencies 指定功能包的依赖 rclpy
```

## 7.3 创建节点
编写python节点，在__init__.py同级目录下创建一个li4.py的文件

非OOP面向过程
OOP面向对象
 ## 7.4 编写ros2节点程序步骤
 1 导入库文件
 ```
 import rclpy
 from rclpy.node import Node
 ```
 2 初始化客户端库
 `rclpy.init(args=args)` #初始化rclpy
 3 新建节点
 `node = Node("li4")`
 4 spin循环节点
 5 关闭客服端库
 `rclpy.shutdown()`			
 
### ros2怎么找到我们编写的程序

# 节点名称设置在下图中，如图设置
 
 ![[Screenshot from 2021-12-27 19-02-11.png]]

```
"li4_node = village_li.li4:main"
```
vinllage文件下li.py文件下main函数
### 编译，并查看

![[Screenshot from 2021-12-27 19-02-00.png]]
 
 linux下过滤
`ros2 pkg list | grep village`

ctrl+shift +5 vscode中增加终端	