# How to use multi-cast function

## 1 Introduction

This document will show you how to use rs_driver to receive point cloud from the LiDAR working in multi-cast mode.

## 2 Steps (Linux)

Suppose the multi-cast address of LiDAR is ```224.1.1.1```.  

### 2.1 Set up parameters

Set up the  parameter ```multi_cast_address```. Here is an example for offline decoding pcap bag.

```c++
RSDriverParam param;                                             ///< Create a parameter object
param.input_param.read_pcap = true;                              ///< Set read_pcap to true
param.input_param.pcap_path = "/home/robosense/rs16.pcap";  ///< Set the pcap file directory
param.input_param.msop_port = 6699;             ///< Set the lidar msop port number, the default is 6699
param.input_param.difop_port = 7788;            ///< Set the lidar difop port number, the default is 7788
param.lidar_type = LidarType::RS16;             ///< Set the lidar type. Make sure this type is correct 
param.input_param.multi_cast_address = "224.1.1.1"; ///< Set the multi-cast address.

```

### 2.2 Set up the PC ip address

In multi-cast case, the ip address of PC has no limit but user need to make sure **the PC and LiDAR are in the same net work**, which means PC can ping to LiDAR.

### 2.3 Add the PC to the group

Use the following command to check the PC's ethernet card name:

```bash
ifconfig
```

Then execute the following command to add the PC to the group (replace the ```enp2s0``` with your ethernet card name):

```
sudo route add -net 224.0.0.0/4 dev enp2s0
```

### 2.4 Run

Run the program. 











