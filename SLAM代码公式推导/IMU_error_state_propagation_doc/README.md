# IMU_error_state_propagation_doc

这份pdf提供了不考虑地球自转时的IMU误差传播方程详细推导，本身目的是为了推导李明扬的MSCKF 2.0中的误差传播公式，但采用了Hamilton四元数。
为避免歧义，包括了详细的四元数和旋转矩阵映射的定义，以及旋转矩阵物理意义的定义，个人认为写的还算比较严谨，如果有朋友发现错误，烦请不吝指正。

相关的内容被用于开发[LARVIO](https://github.com/PetWorm/LARVIO)，如果您在研究中使用到这份文档，请引用我们的文章：
```txt
@article{qiu2019monocular,
  title={Monocular Visual-Inertial Odometry with an Unbiased Linear System Model and Robust Feature Tracking Front-End},
  author={Qiu, Xiaochen and Zhang, Hai and Fu, Wenxing and Zhao, Chenxu and Jin, Yanqiong},
  journal={Sensors},
  volume={19},
  number={8},
  pages={1941},
  year={2019},
  publisher={Multidisciplinary Digital Publishing Institute}
}
```
