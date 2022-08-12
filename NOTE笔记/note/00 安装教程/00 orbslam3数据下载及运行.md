# 1 跑EuRoC数据集（含双目+imu数据）
[EuRoC数据集下载链接](https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets#downloads)
```
# 单目
./Examples/Monocular/mono_euroc  ./Vocabulary/ORBvoc.txt ./Examples/Monocular/EuRoC.yaml ~/Downloads/MH_01_easy ./Examples/Monocular/EuRoC_TimeStamps/MH01.txt

# 单+imu
./Examples/Monocular-Inertial/mono_inertial_euroc ./Vocabulary/ORBvoc.txt ./Examples/Monocular-Inertial/EuRoC.yaml ~/Downloads/MH_01_easy ./Examples/Monocular-Inertial/EuRoC_TimeStamps/MH01.txt MH01_monoi

# 双目
./Examples/Stereo/stereo_euroc ./Vocabulary/ORBvoc.txt ./Examples/Stereo/EuRoC.yaml ~/Downloads/MH_01_easy ./Examples/Stereo/EuRoC_TimeStamps/MH01.txt MH01_stereo

# 双+IMU
./Examples/Stereo-Inertial/stereo_inertial_euroc ./Vocabulary/ORBvoc.txt ./Examples/Stereo-Inertial/EuRoC.yaml ~/Downloads/MH_04_difficult ./Examples/Stereo-Inertial/EuRoC_TimeStamps/MH04.txt MH04_stereo
```

[ros运行参考链接](https://rupingcen.blog.csdn.net/article/details/115690725?spm=1001.2101.3001.6650.10&utm_medium=distribute.pc_relevant.none-task-blog-2%7Edefault%7EBlogCommendFromBaidu%7Edefault-10.essearch_pc_relevant&depth_1-utm_source=distribute.pc_relevant.none-task-blog-2%7Edefault%7EBlogCommendFromBaidu%7Edefault-10.essearch_pc_relevant)
