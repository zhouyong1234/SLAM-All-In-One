

# Install

#### 1. Install Eigen & OpenCV (if you don't have it):
    sudo apt-get install libeigen3-dev libopencv-dev

#### 2. install ziplib:
    sudo apt-get install zlib1g-dev
    cd thirdparty
    tar -zxvf libzip-1.1.1.tar.gz
    cd libzip-1.1.1/
    ./configure
    make
    sudo make install
    sudo cp lib/zipconf.h /usr/local/include/zipconf.h   # (no idea why that is needed).


#### 3. install aruco marker detection (optional - only required for vignette calibration):
see eg here: 

    http://maztories.blogspot.de/2013/07/installing-aruco-augmented-reality.html 
    
tested with version 1.3.0. which is included in /thirdparty.

#### 4. Build
    cmake . && make





# Usage: C++ code

## playbackDataset: read images, photometric undistortion & rectification.
Shows images of a dataset. Meant as example code regarding how to read the dataset.
Run with (and replace X with the location of the dataset. Mind the trailing slash):

    ./playDataset X/sequence_01/  


optionally, the calibration is used for 
- rectification ( r )
- response function inversion ( g )
- vignette removal ( v )
- removal of over-exposed (white) images. ( o ).

Pressing the respective key toggles the option. See code for details.


# responseCalib: calibrate response function.
Performs photometric calibration from a set of images, showing the exact same scene at different exposures.
Run with (and replace X with the location of the dataset. Mind the trailing slash):

./responseCalib X/CalibrationDatasets/narrow_sweep1/" 

outputs some intermediate results, and pcalib.txt containing the calibrated inverse response function to ./photoCalibResult. 
See code for details.


# vignetteCalib: calibrate vignette.
Performs photometric calibration from a set of images, showing a flat surface with an ARMarker.
Run with  (and replace X with the location of the dataset. Mind the trailing slash):

./vignetteCalib X/CalibrationDatasets/narrow_vignette/" 

outputs some intermediate results, and vignette.png (16-bit png) containing the calibrated vignette function.
vignetteSmoothed.png is a slightly smoothed version, mainly to remove the black borders (pixels at the border are never observed). See code for details.
**WARNING: requires a lot of Memory (16GB ram for 1000 input images)**! Can easily be changed at the cost of slightly slower runtime... you'll have to do that yourself though.



# Usage: Matlab evaluation code
Implements Sim(3) alignment of a tracked trajectory to the ground-truth segments, and subsequent computation of the different error values. See MatlabEvaluationCode/Example.m for an example, and some documentation regarding the computed values. Further, we include example results computed with DSO for all 50 sequences.



#  License
The code provided in this repository is licensed under the BSD license.

