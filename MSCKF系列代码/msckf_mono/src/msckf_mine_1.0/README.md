# This is a simple example of MSCKF

## Change Log
2018.02.05 Almost Done, But still many many bugs. The code can only run 10 frame then the update step seems wrong. 
2017.12.16 Finish the feature tracking and ready to realize the Feature Manager test! the test video can be found [here](https://youtu.be/Nr3VIoFZAhs). 

# 1.Description
wait to add..


# 2. Prerequisites
We have tested the library in **16.04**, but it should be easy to compile in other platforms.

## OpenCV
We use [OpenCV](http://opencv.org) to manipulate images and features. Dowload and install instructions can be found at: http://opencv.org. **Required at leat 2.4.3. Tested with OpenCV 2.4.11 and OpenCV 3.2**.

## Eigen3
Download and install instructions can be found at: http://eigen.tuxfamily.org. **Required at least 3.1.0**.

## Ceres
Download and install instructions can be found at: http://www.ceres-solver.org/installation.html.


# 3. Folder format
**wait to add**

# 4. How to build
mkdir build
cd build
cmake ..
make

# 5. How to run
#### Firstly
you need use the python script to translate the csv file to the txt file.
#### Then in the bin folder run:
./frame_with_imu_test

then you will get result in result/

---

# 6. Show the results

# Good Luck!
