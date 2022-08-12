# How to use transformation function

## 1 Introduction

This document will show you how to use the built in coordinate transformation function. Before reading this document, please make sure you have read the [Online connect LiDAR](how_to_online_use_driver.md) or [Offline decode pcap bag](how_to_offline_decode_pcap.md).

The rotation order of the transformation is **yaw - pitch - row**.

The unit of x, y, z, is ```m```, and the unit of roll, pitch, yaw, is ```radian```.

## 2 Steps

### 2.1 Enable

To enable the coordinate transformation function, set the following option to ```ON``` when executing cmake command:

```bash
cmake -DENABLE_TRANSFORM=ON ..
```

### 2.2 Set up parameters

Set up the transformation parameters. The default value of each parameter is ```0```.  Here is an example for offline decoding pcap bag ( x=1, y=0, z=2.5, roll=0.1, pitch=0.2, yaw=1.57 ): 

```c++
RSDriverParam param;                                             ///< Create a parameter object
param.input_param.read_pcap = true;                              ///< Set read_pcap to true
param.input_param.pcap_path = "/home/robosense/rs16.pcap";  ///< Set the pcap file directory
param.input_param.msop_port = 6699;             ///< Set the lidar msop port number, the default is 6699
param.input_param.difop_port = 7788;            ///< Set the lidar difop port number, the default is 7788
param.lidar_type = LidarType::RS16;             ///< Set the lidar type. Make sure this type is correct
param.decoder_param.transform_param.x = 1;		///< unit: m
param.decoder_param.transform_param.y = 0;		///< unit: m
param.decoder_param.transform_param.z = 2.5;	///< unit: m
param.decoder_param.transform_param.roll = 0.1; ///< unit: radian
param.decoder_param.transform_param.pitch = 0.2;///< unit: radian
param.decoder_param.transform_param.yaw = 1.57; ///< unit: radian

```

