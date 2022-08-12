# How to online use driver

## 1 Introduction

This document will show you how to use the API to get point cloud from LiDAR.

## 2 Steps

Please follow the steps below to do advanced development.

### 2.1 Define a point type

Now the driver will automatically detect and assign value to the following six variables.

- x ------ The x coordinate of point.
- y ------ The y coordinate of point.
- z ------ The z coordinate of point.
- intensity ------ The intensity of point.
- timestamp ------ The timestamp of point. If ```use_lidar_clock``` is set to ```true```, this timestamp will be lidar time, otherwise will be system time.
- ring ------ The ring ID of the point, which represents the row number. e.g. For RS80, the range of ring ID is 0~79 (from bottom to top).

Here are some examples: 

- The point type contains **x, y, z** 

  ```c++
  struct PointXYZ ///< user defined point type
  {
    float x;
    float y;
    float z;
    ...
  };
  ```

- The point type contains **x, y, z, intensity**

  ```c++
  struct PointXYZI ///< user defined point type
  {
    float x;
    float y;
    float z;
    uint8_t intensity;
    ...
  };
  ```

  Or you can use **pcl::PointXYZI** directly

- The point type contains **x, y, z, intensity, timestamp, ring**

  ```c++
  struct PointXYZIRT ///< user defined point type
  {
    float x;
    float y;
    float z;
    uint8_t intensity;
    double timestamp;
    uint16_t ring;
    ...
  };
  ```

The only thing user need to pay attention to is that the variable name must obey the rules -- **x, y, z, intensity, timestamp, ring**, otherwise the variable can not be assigned as expected.

### 2.2 Define a point cloud callback function

The template argument PointXYZI is the point type we defined in 2.1. When point cloud message is ready, this function will be called by driver. **Note! Please don't add any time-consuming operations in this function!** User can make a copy of the message and process it in another thread.  Or user can add some quick operations such like ros publish in the callback function.

```c++
void pointCloudCallback(const PointCloudMsg<PointXYZI> &msg)
{
  RS_MSG << "msg: " << msg.seq << " point cloud size: " << msg.point_cloud_ptr->size() << RS_REND;
}
```

### 2.3 Define a exception callback function

Define the exception callback function. When driver want to send out infos or error codes, this function will be called. Same as the previous callback function, please **do not add any time-consuming operations in this callback function!**

```c++
void exceptionCallback(const Error &code)
{
  RS_WARNING << "Error code : " << code.toString() << RS_REND;
}
```

### 2.4 Instantiate the driver object

Instanciate a driver object.

```c++
LidarDriver<PointXYZI> driver;          ///< Declare the driver object
```

### 2.5 Define the parameter and configure the parameter

 The msop port and difop port number of lidar can be got from wireshark(a network socket capture software). The default value is ```msop-6699``` and ```difop-7788```. User also need to make sure the ```lidar_type``` is set correctly.

```c++
RSDriverParam param;                      		  ///< Create a parameter object
param.input_param.msop_port = 6699;              ///< Set the lidar msop port number the default 6699
param.input_param.difop_port = 7788;             ///< Set the lidar difop port number the default 7788
param.lidar_type = LidarType::RS16;             ///< Set the lidar type. Make sure this type is correct
```

### 2.6 Register the point cloud callback and exception callback

Register the callback functions we defined in 2.2 and 2.3. **The exception callback function must be registered before the init() function is called because  error may occur during the initialization**.

```c++
driver.regRecvCallback(pointCloudCallback); ///< Register the point cloud callback function into the driver
driver.regExceptionCallback(exceptionCallback);  ///<Register the exception callback function into the driver
```

#### 2.7 Call the driver initialization function

Call the initialization function and pass the parameter into the driver. Remember to check whether the initialization is successful, if not, please check the error code, and modify parameters.

```c++
if (!driver.init(param))                         ///< Call the init function and pass the parameter
{
  RS_ERROR << "Driver Initialize Error..." << RS_REND;
  return -1;
}
```

#### 2.8 Start the driver

Call the start function to start the driver.

```c++
driver.start();                                  ///< Call the start function. The driver thread will start
```



## 3 Point cloud storage order

In rs_driver, the point cloud is stored in **column major order**, which means if there is  a point msg.point_cloud_ptr->at(i) , the next point on the same ring should be msg.point_cloud_ptr->at(i+msg.height). User can set the parameter ```saved_by_rows``` to ```true``` to make the point cloud stored in **row major order**.



### *Congratulations! You have finished the demo tutorial of RoboSense LiDAR driver! You can find the complete demo code in the demo folder under the project directory. Feel free to connect us if you have any question about the driver.*