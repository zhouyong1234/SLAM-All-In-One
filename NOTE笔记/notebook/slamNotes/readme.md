# slam notes
## 目录

- [资料](#资料)
- [环境安装](#环境安装)
- [各种库的使用](#各种库的使用)
- [大佬的博客](#大佬的博客)
- [我的主要工作](#我的主要工作)
- [SLAM评估工具](#SLAM评估工具)
- [SLAM找工作](#SLAM找工作)

## 资料
- [常用开源方案](https://github.com/liuqian62/notebook/tree/main/slamNotes/slam%E5%BC%80%E6%BA%90%E6%96%B9%E6%A1%88)
- [视觉SLAM相关研究](https://github.com/wuxiaolang/Visual_SLAM_Related_Research)
- [SLAM十四讲](https://github.com/liuqian62/notebook/blob/main/slamNotes/14%E8%AE%B2.md)
- [多传感器融合定位知乎专栏](https://zhuanlan.zhihu.com/c_1114864226103037952)
- [深蓝学院视觉slam](https://github.com/zhouyong1234/VIO-Course)
- [深蓝学院激光slam](https://github.com/zhouyong1234/Laser-SLAM-Course)
- [古月居](https://www.guyuehome.com/)

<!-- - [如何使用g2o](use_g2o.md)
- [如何使用Ceres](use_ceres.md) -->

<div align="right">
    <b><a href="#目录">↥ Back To Top</a></b>
</div>


## 环境安装
* [ubuntu安装](https://blog.csdn.net/baidu_36602427/article/details/86548203?ops_request_misc=%257B%2522request%255Fid%2522%253A%2522165266749016782395341493%2522%252C%2522scm%2522%253A%252220140713.130102334.pc%255Fall.%2522%257D&request_id=165266749016782395341493&biz_id=0&utm_medium=distribute.pc_search_result.none-task-blog-2~all~first_rank_ecpm_v1~rank_v31_ecpm-1-86548203-null-null.142^v9^pc_search_result_cache,157^v4^control&utm_term=ubuntu18.04%E5%AE%89%E8%A3%85%E6%95%99%E7%A8%8B&spm=1018.2226.3001.4449)
* [ros安装](https://blog.csdn.net/weixin_50060664/article/details/121781535?ops_request_misc=%257B%2522request%255Fid%2522%253A%2522165266766216782350951349%2522%252C%2522scm%2522%253A%252220140713.130102334.pc%255Fall.%2522%257D&request_id=165266766216782350951349&biz_id=0&utm_medium=distribute.pc_search_result.none-task-blog-2~all~first_rank_ecpm_v1~rank_v31_ecpm-3-121781535-null-null.142^v9^pc_search_result_cache,157^v4^control&utm_term=ubuntu%E5%AE%89%E8%A3%85ros&spm=1018.2226.3001.4449)
* [ros常用命令](https://blog.csdn.net/u010585964/article/details/78715130?ops_request_misc=%257B%2522request%255Fid%2522%253A%2522165551422316780366549949%2522%252C%2522scm%2522%253A%252220140713.130102334..%2522%257D&request_id=165551422316780366549949&biz_id=0&utm_medium=distribute.pc_search_result.none-task-blog-2~all~baidu_landing_v2~default-4-78715130-null-null.142^v17^pc_rank_34,157^v15^new_3&utm_term=ros%E5%B8%B8%E7%94%A8%E5%91%BD%E4%BB%A4&spm=1018.2226.3001.4187)
* [各种库的安装](https://blog.csdn.net/Night___Raid/article/details/105113617?ops_request_misc=%257B%2522request%255Fid%2522%253A%2522165266819116782350993650%2522%252C%2522scm%2522%253A%252220140713.130102334.pc%255Fall.%2522%257D&request_id=165266819116782350993650&biz_id=0&utm_medium=distribute.pc_search_result.none-task-blog-2~all~first_rank_ecpm_v1~rank_v31_ecpm-3-105113617-null-null.142^v9^pc_search_result_cache,157^v4^control&utm_term=slam%E7%9A%84%E5%90%84%E7%A7%8D%E5%BA%93%E5%AE%89%E8%A3%85&spm=1018.2226.3001.4449)
* [安装多个版本OpenCV](https://heyijia.blog.csdn.net/article/details/54575245?spm=1001.2014.3001.5502)
* [d435i驱动安装和标定](https://blog.csdn.net/qq_35616298/article/details/116171823?ops_request_misc=%257B%2522request%255Fid%2522%253A%2522162942123216780271562120%2522%252C%2522scm%2522%253A%252220140713.130102334.pc%255Fall.%2522%257D&request_id=162942123216780271562120&biz_id=0&utm_medium=distribute.pc_search_result.none-task-blog-2~all~first_rank_v2~rank_v29-1-116171823.first_rank_v2_pc_rank_v29&utm_term=%E9%94%99%E8%AF%AF%3A+%E6%97%A0%E6%B3%95%E9%AA%8C%E8%AF%81+faculty.cse.tamu.edu+%E7%9A%84%E7%94%B1+%E2%80%9CCN%3DInCommon+RSA+Server+CA%2COU%3DInCommon%2CO%3DInternet2%2CL%3DAnn+Arbor%2CST%3DMI%2CC%3DUS%E2%80%9D+%E9%A2%81%E5%8F%91%E7%9A%84%E8%AF%81%E4%B9%A6%3A&spm=1018.2226.3001.4187)
* [ros2教程](http://fishros.com/#/fish_home)
* [ros2交流社区](https://fishros.org.cn/forum/)

<div align="right">
    <b><a href="#目录">↥ Back To Top</a></b>
</div>


## 各种库的使用

--[代码](https://github.com/liuqian62/lib_use)
* [Eigen使用](https://blog.csdn.net/yxpandjay/article/details/80587916?ops_request_misc=%257B%2522request%255Fid%2522%253A%2522165266842616782248567999%2522%252C%2522scm%2522%253A%252220140713.130102334.pc%255Fall.%2522%257D&request_id=165266842616782248567999&biz_id=0&utm_medium=distribute.pc_search_result.none-task-blog-2~all~first_rank_ecpm_v1~rank_v31_ecpm-1-80587916-null-null.142^v9^pc_search_result_cache,157^v4^control&utm_term=eigen%E4%BD%BF%E7%94%A8&spm=1018.2226.3001.4449)
  * [Eigen平移旋转](https://blog.csdn.net/u011092188/article/details/77430988) 
* [g2o使用](https://blog.csdn.net/He3he3he/article/details/110007973?ops_request_misc=%257B%2522request%255Fid%2522%253A%2522165266856216782246426329%2522%252C%2522scm%2522%253A%252220140713.130102334.pc%255Fall.%2522%257D&request_id=165266856216782246426329&biz_id=0&utm_medium=distribute.pc_search_result.none-task-blog-2~all~first_rank_ecpm_v1~rank_v31_ecpm-9-110007973-null-null.142^v9^pc_search_result_cache,157^v4^control&utm_term=g2o%E4%BD%BF%E7%94%A8&spm=1018.2226.3001.4449)
* [Ceres使用](https://blog.csdn.net/zzyczzyc/article/details/88937558?ops_request_misc=%257B%2522request%255Fid%2522%253A%2522165266905516782395383342%2522%252C%2522scm%2522%253A%252220140713.130102334.pc%255Fall.%2522%257D&request_id=165266905516782395383342&biz_id=0&utm_medium=distribute.pc_search_result.none-task-blog-2~all~first_rank_ecpm_v1~rank_v31_ecpm-5-88937558-null-null.142^v9^pc_search_result_cache,157^v4^control&utm_term=ceres%E4%BD%BF%E7%94%A8&spm=1018.2226.3001.4449)
* [OpenCV使用](https://blog.csdn.net/zzx2016zzx/article/details/108691235?ops_request_misc=%257B%2522request%255Fid%2522%253A%2522165266957316781683948705%2522%252C%2522scm%2522%253A%252220140713.130102334.pc%255Fall.%2522%257D&request_id=165266957316781683948705&biz_id=0&utm_medium=distribute.pc_search_result.none-task-blog-2~all~first_rank_ecpm_v1~rank_v31_ecpm-6-108691235-null-null.142^v9^pc_search_result_cache,157^v4^control&utm_term=opencv%E6%95%99%E7%A8%8Bc%2B%2B&spm=1018.2226.3001.4449)
  * [OpenCV官方文档](https://docs.opencv.org/3.4.4/index.html)
* [Sophus使用](https://blog.csdn.net/u011092188/article/details/77833022?ops_request_misc=%257B%2522request%255Fid%2522%253A%2522165267053516780357263815%2522%252C%2522scm%2522%253A%252220140713.130102334.pc%255Fall.%2522%257D&request_id=165267053516780357263815&biz_id=0&utm_medium=distribute.pc_search_result.none-task-blog-2~all~first_rank_ecpm_v1~rank_v31_ecpm-5-77833022-null-null.142^v9^pc_search_result_cache,157^v4^control&utm_term=Sophus%E4%BD%BF%E7%94%A8&spm=1018.2226.3001.4449)

<div align="right">
    <b><a href="#目录">↥ Back To Top</a></b>
</div>


## 大佬的博客
* [高翔](https://www.cnblogs.com/gaoxiang12/)
* [贺一家](https://blog.csdn.net/heyijia0327?type=blog)
* [紫薯萝卜](https://www.zhihu.com/people/mao-shu-yuan/posts)
* [小吴同学](https://wym.netlify.app/)


<!-- ## slam 后端一般分为两种处理方法
* 扩展卡尔曼滤波（滤波方法）
* 图优化（非线性优化方法）

## 图优化
1. 构建图。机器人位姿作为顶点，位姿间关系作为边。
2. 优化图。调整机器人的位姿（顶点）来尽量满足边的约束，使得误差最小。 -->


<div align="right">
    <b><a href="#目录">↥ Back To Top</a></b>
</div>


## 我的主要工作
* [SLAM学习笔记](slam学习笔记.md)
* [论文阅读](./slam论文阅读/)
* [2021](2021.md)
  * MSCKF加回环检测
  * MSCKF双目改单目
  * LARVIO加回环检测 
* [2022](2022.md)
  * MSCKF改进特征跟踪
  * OpenVINS改进特征跟踪
* [2023](2023.md)
  * do something 

<div align="right">
    <b><a href="#目录">↥ Back To Top</a></b>
</div>


## SLAM评估工具
* [evo](https://github.com/MichaelGrupp/evo)
  * [evo使用教程](https://blog.csdn.net/u011341856/article/details/104594392?spm=1001.2014.3001.5501) 
* [rpg_trajectory_evaluation](https://github.com/uzh-rpg/rpg_trajectory_evaluation) 
* [数据集](https://blog.csdn.net/crp997576280/article/details/103340020?ops_request_misc=%257B%2522request%255Fid%2522%253A%2522165284191416781432971813%2522%252C%2522scm%2522%253A%252220140713.130102334..%2522%257D&request_id=165284191416781432971813&biz_id=0&utm_medium=distribute.pc_search_result.none-task-blog-2~all~sobaiduend~default-1-103340020-null-null.142^v10^pc_search_result_control_group,157^v4^control&utm_term=slam%E6%95%B0%E6%8D%AE%E9%9B%86%E7%99%BE%E5%BA%A6%E7%BD%91%E7%9B%98&spm=1018.2226.3001.4187)
* [tum](https://vision.in.tum.de/data/datasets/rgbd-dataset/download)
* [awesome-slam-datasets](https://github.com/youngguncho/awesome-slam-datasets)

<div align="right">
    <b><a href="#目录">↥ Back To Top</a></b>
</div>


## SLAM找工作
* [面试问题整理](https://github.com/liuqian62/notebook/blob/main/slamNotes/SLAM%E9%9D%A2%E8%AF%95%E9%97%AE%E9%A2%98%E6%95%B4%E7%90%86.md)
* [视觉slam面试题总结](https://blog.csdn.net/weixin_44580210/article/details/91790044)
* [slam常见面试题](https://zhuanlan.zhihu.com/p/46694678)
* [slam求职分享](https://zhuanlan.zhihu.com/p/68858564)
* [面试SLAM算法实习生总结](https://zhuanlan.zhihu.com/p/76280626)

<div align="right">
    <b><a href="#目录">↥ Back To Top</a></b>
</div>

