# quick start tips for xiaoqiang 
### 0. install DSO from https://github.com/JakobEngel/dso.git
### 1. git clone and install
```
cd ~/Documents/ros/src
git clone https://github.com/BlueWhaleRobot/dso_ros.git 
cd ..
export DSO_PATH=[PATH_TO_DSO]/dso　　#egg: export DSO_PATH=/home/xiaoqiang/Documents/dso
catkin_make
```
### 2. launch the node
```
rosrun dso_ros dso_live image:=/camera_node/image_raw calib=/home/xiaoqiang/Documents/ros/src/dso_ros/camera.txt mode=1
```
# Made with :heart: by BlueWhale Tech corp.

##################################################################

# ROS Wrapper around DSO: Direct Sparse Odometry

For more information see
[https://vision.in.tum.de/dso](https://vision.in.tum.de/dso)

This is meant as simple, minimal example of how to integrate DSO from a different project, and run it on real-time input data.
It does not provide a full ROS interface (no reconfigure / pointcloud output / pose output).
To access computed information in real-time, I recommend to implement your own Output3DWrapper; see the DSO code.


### Related Papers

* **Direct Sparse Odometry**, *J. Engel, V. Koltun, D. Cremers*, In arXiv:1607.02565, 2016

* **A Photometrically Calibrated Benchmark For Monocular Visual Odometry**, *J. Engel, V. Usenko, D. Cremers*, In arXiv:1607.02555, 2016



# 1. Installation

1. Install DSO. We need DSO to be compiled with OpenCV (to read the vignette image), and with Pangolin (for 3D visualization).
2. run 

		export DSO_PATH=[PATH_TO_DSO]/dso
		rosmake
	


# 3 Usage
everything as described in the DSO project - only this is for real-time camera input.


		rosrun dso_ros dso_live image:=image_raw \
			calib=XXXXX/camera.txt \
			gamma=XXXXX/pcalib.txt \
			vignette=XXXXX/vignette.png \


## 3.1 Accessing Data.
see the DSO Readme. As of now, there is no default ROS-based `Output3DWrapper` - you will have to write your own.




# 4 Dependencies

## 4.1 Pangolin
removing

	    fullSystem->outputWrapper = new IOWrap::PangolinDSOViewer(
	    		 (int)undistorter->getSize()[0],
	    		 (int)undistorter->getSize()[1]);

will allow you to use DSO compiled without Pangolin. However, then there is no 3D visualization.
You can also implement your own Output3DWrapper to fit your needs.

## 4.2 OpenCV
you can use DSO compiled without OpenCV. 
In that case, the vignette image will not be read, and no photometric calibration can be used. Also, there will not be any image visualizations / image saving.
You can also implement your own version of ImageRW.h / ImageDisplay.h, instead of the dummies.


### 5 License
This ROS wrapper around DSO is licensed under the GNU General Public License
Version 3 (GPLv3).
For commercial purposes, we also offer a professional version, see
[http://vision.in.tum.de/dso](http://vision.in.tum.de/dso) for details.
