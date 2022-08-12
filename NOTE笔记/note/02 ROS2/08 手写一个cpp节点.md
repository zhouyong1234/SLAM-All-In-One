## 8.1 创建功能包
```
ros2 pkg create --build-type ament_cmake --dependenies rclcpp
```
## 8.2 编写cpp节点
在src下创建wang2.cpp写程序

```
#include<rclcpp/rclcpp.hpp>

class SingleDogNode : public rclcpp::Node
{
private:
    /* data */
public:

    SingleDogNode(std::string name):Node(name)
    {
        RCLCPP_INFO(this->get_logger(), "大家好，我是单身汉%s.",name.c_str());
    }
};

int main(int argc,char **argv){
    rclcpp::init(argc,argv);
    auto node = std::make_shared<SingleDogNode> ("wang2");
    rclcpp::spin(node);
    rclcpp::shutdown();
    //return 0;


}
```

## 8.3 在CMakeLists.txt修改
系统找写的cpp文件，需要修改cmakelists.txt文件，在该文件最后加上
在wang2.cpp中输入上面的内容后，还需要修改一下CmakeLists.txt。

在CmakeLists.txt最后一行加入下面两行代码。

```
add_executable(wang2_node src/wang2.cpp)
ament_target_dependencies(wang2_node rclcpp)
```
添加这两行代码的目的是让编译器编译wang2.cpp这个文件，不然不会主动编译。接着在上面两行代码下面添加下面的代码。

```
install(TARGETS
  wang2_node
  DESTINATION lib/${PROJECT_NAME}
)
```

这个是C++比Python要麻烦的地方，需要手动将编译好的文件安装到`install/village_wang/lib/village_wang`下

## 8.4 编译运行节点
单独编译某个功能包(village_wang2功能包)
`colcon build --packages-select village_wang2`

## 8.5 添加环境变量
`source install/setup.bash`

## 8.6 运行节点
`ros2 run village_wang2 wang2_node`



