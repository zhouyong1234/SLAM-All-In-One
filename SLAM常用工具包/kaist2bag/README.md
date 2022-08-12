# kaist2bag
A tool to convert KAIST urban dataset to rosbag.

Only tested on Ubuntu 20.04.



## Guide

1. Download desired database files from the [website](https://sites.google.com/view/complex-urban-dataset)

2. Extract `.gz.tar` files into their folders
```
find . -name 'urban28*.tar.gz' -execdir tar -xzvf '{}' \;
```

3. Clone and build this repository
```
cd src
git clone https://github.com/irapkaist/irp_sen_msgs.git
git clone https://github.com/tsyxyz/kaist2bag.git
cd ..
catkin build
```

4. Edit the [config file](config/config.yaml) with path and desired topics


5. Create a rosbag file for each sensor type
```
source devel/setup.bash
roslaunch kaist2bag kaist2bag.launch
```

6. Merge all bags into a single one (if desired)
```
rosrun kaist2bag mergebag.py merged.bag <bag_file_1> ... <bag_file_8>
```




## Acknowledge

File parsing code are referenced from [file_player](https://github.com/irapkaist/file_player).



