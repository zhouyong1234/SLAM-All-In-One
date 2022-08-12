# How to use visualization tool

## 1 Introduction

This document will show you how to use the visualization tool to watch point cloud from LiDAR.

## 2 Run

If user install the **rs_driver** in the previous step, the tool can be start by the following command:

```bash
rs_driver_viewer
```

Otherwise, the tool need to be start with the absolute path:

```bash
./rs_driver/build/tool/rs_driver_viewer 
```

### 2.1 Arguments

- -h/--help

   print the argument menu 

- -msop

   Msop port number of LiDAR, the default value is *6699*

- -difop

   Difop port number of LiDAR, the default value is *7788*

- -type

   Typer of LiDAR, the default value is *RS16*

- -x

   Transformation parameter, default is 0, unit: m

- -y

   Transformation parameter, default is 0, unit: m

- -z

   Transformation parameter, default is 0, unit: m

- -roll

   Transformation parameter, default is 0, unit: radian

- -pitch

   Transformation parameter, default is 0, unit: radian

- -yaw

   Transformation parameter, default is 0, unit: radian

- -pcap

   The absolute path of pcap file. If this argument is set, the driver will work in off-line mode and the pcap file. Otherwise the driver work in online mode.

**The point cloud transformation function can only be used when the cmake option ENABLE_TRANSFORM is set to ON.**

## 3 Examples

- Online decode a RS128 LiDAR, which msop port is ```9966``` and difop port is ```8877```

  ```bash
  rs_driver_viewer -msop 9966 -difop 8877 -type RS128 
  ```

- Offline decode a RSHELIOS LiDAR with a pcap file.

  ```bash
  rs_driver_viewer -pcap /home/robosense/helios.pcap -type RSHELIOS
  ```

- Online decode a RS16 LiDAR with the coordinate transformation parameter x=1.5, y=2, z=0, roll=1.57, pitch=0, yaw=0

  ```bash
  rs_driver_viewer -type RS16 -x 1.5 -y 2 -roll 1.57 
  ```

  

