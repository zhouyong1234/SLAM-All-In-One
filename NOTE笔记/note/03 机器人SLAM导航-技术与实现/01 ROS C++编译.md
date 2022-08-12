## 1  ros
ros参考ros2笔记

通信机制（话题、服务、动作、参数）
ros中其他概念：
	paramater、tf（广播tf变换，监听tf变化）、urdf（机械模型，几何物体形状，部件的tf关系）、launch（node协同工作）

## 1.2 C++编程范式
### 1.2.0 头文件，源文件
头文件：*.h  、*.hpp,源文件：*.cc 、*.cpp

## 1.2.1  c++三种编译方式
#### 1: g++编译代码
采用g++编译是c++代码最之间的方式，编译分4步：预处理、编译、汇编、连接

test.cpp 预编译->test.i编译->test.s汇编->test.o链接->test
                                                                    libxx.o
```													
cd demo
g++ foo.cpp main.cpp -o demo
./demo
```
上面的编译命令执行后，对foo.cpp与main.cpp进行编译，最后生成了可执行程序文件demo

#### 2：make编译代码
当程序庞大后会涉及很多*.cpp和外部依赖库，逐一输出g++的命令将不是很方便，这时候可以编写makeflie文件来编写编译脚本，然后使用make命令进行编译。
makefile文件：
```
start:
	g++ -o foo.o -c foo.cpp
	g++ -o main.o -c main.cpp
	g++ -o demo foo.o main.o
clean:
	rm -rf foo.o main.o

```
#### 3： 使用cmake编译代码 如下
makefile降低了编译难度，但是程序有很多依赖和关联，全部手动去维护这些依赖很麻烦，
Cmake可以自动处理程序之间的关系，并产生相应的makefile文件，然后make编译。

cmake编译过程如下：
### 1.2.2 头文件foo.h

```
#ifndef FOO_H_
#define FOO_H_
#include<string>

namespace foo  //5行 命名空间
{
    class MyPrint   // 8-14行 声明类,类最有有个分号;
    {
        public:
            MyPrint(std::string output); // 10行 析构函数
            void EXcutePrint();          // 11行 成员函数

            std::string output_;         // 13行 成员变量
    };
}

#endif

/*
  1,2,17行 宏定义，防止头文件的重复包含与编译
    5行 命名空间，防止出现重复的函数与变量名字
    8-14行 声明类，不具体实现类，具体定义放在对应的*.cpp，
    10行 构造函数（带一个字符串的参数）
    11行 成员函数
    13行 成员变量
*/

```

### 1.2.3 foo.cpp
```
#include"foo.h"        # 头文件  
#include<iostream>  
#include<string>  
  
namespace foo       # 命名空间
{  
    MyPrint::MyPrint(std::string output):output_(output)  
    {  
        std::cout<<"class MyPrint created aobject!";  
        std::cout<<std::endl;  
    }  
    void MyPrint::EXcutePrint()  
    {  
        std::cout<<output_<<std::endl;  
    }  
  
}  
  
# 7-11行 构造函数的实现  
# 13-16行 打印函数的具体实现
```


### 1.2.4 main.cpp

```
#include"foo.h"

int main(int argc,char** argv)
{
    foo::MyPrint my_print("I can output string!");
    my_print.EXcutePrint();
    return 0;
}
```

### 1.2.5 CMakeLists.txt
```
cmake_minimum_required(VERSION 2.8) # 声明CMake最低版本
project(Demo) # 声明CMake的工程名

# 设置头文件搜索路径
include_directories( ${PROJECT_BINRARY_DIR})

# 创建库文件
add_library(foo foo.cpp)
# 创建可执行文件
add_executable(demo mian.cpp)
# 为可执行文件链接依赖库
target_link_libraries(demo foo)
```
