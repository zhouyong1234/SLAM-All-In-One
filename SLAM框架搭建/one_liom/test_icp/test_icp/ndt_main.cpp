#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>//NDT配准类对应头文件
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/approximate_voxel_grid.h>//滤波类对应头文件
#include <pcl/visualization/pcl_visualizer.h>//可视化头文件
#include <boost/thread/thread.hpp>//多线程相关头文件
int
main (int argc, char** argv)
{
  //加载房间的第一次扫描
  pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  if (pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/touchair/sensor_fusion_ws/src/FAST_LIO/PCD/filtered_scans.pcd", *target_cloud) == -1)
  {
    PCL_ERROR ("Couldn't read file room_scan1.pcd \n");
    return (-1);
  }
  std::cout << "Loaded " << target_cloud->size () << " data points from room_scan1.pcd" << std::endl;
  //加载从新视角得到的房间的第二次扫描
  pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  if (pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/touchair/LIO-SAM/CornerMap.pcd", *input_cloud) == -1)
  {
    PCL_ERROR ("Couldn't read file room_scan2.pcd \n");
    return (-1);
  }
  std::cout << "Loaded " << input_cloud->size () << " data points from room_scan2.pcd" << std::endl;
  //将输入的扫描过滤到原始尺寸的大概10%以提高匹配的速度。
  //对第二次的扫描结果进行体素滤波（下采样）
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::VoxelGrid<pcl::PointXYZ> approximate_voxel_filter;
  // pcl::PassThrough<pcl::PointXYZ> approximate_voxel_filter;

  // pcl::ApproximateVoxelGrid<pcl::PointXYZ> approximate_voxel_filter;
  approximate_voxel_filter.setInputCloud(input_cloud);
  // approximate_voxel_filter.setFilterFieldName("z");
  // approximate_voxel_filter.setFilterLimits(0.0, 2.0);
  approximate_voxel_filter.setLeafSize(0.5f, 0.5f, 0.5f);
  approximate_voxel_filter.filter(*filtered_cloud);//下采样之后的点云保存在filtered_cloud
  std::cout << "Filtered cloud contains " << filtered_cloud->size() << " data points from room_scan2.pcd" << std::endl;
  //初始化正态分布变换（NDT）
  pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
  //设置依赖尺度NDT参数
  //为终止条件设置最大转换差异
  ndt.setTransformationEpsilon (0.01);
  //为More-Thuente线搜索设置最大步长
  ndt.setStepSize (0.1);
  //设置NDT网格结构的分辨率（VoxelGridCovariance）
  ndt.setResolution (1);
  //设置匹配迭代的最大次数
  ndt.setMaximumIterations (35);
  // 设置要配准的点云
  ndt.setInputSource (filtered_cloud);//第二次扫描的点云进行体素滤波的结果.设置输入点云要用setInputSource而不是setInputCloud
  //设置点云配准目标
  ndt.setInputTarget (target_cloud);//第一次扫描的结果
  //设置使用机器人测距法得到的初始对准估计结果
  Eigen::AngleAxisf init_rotation (0, Eigen::Vector3f::UnitZ ());
  Eigen::Translation3f init_translation (0, 0, 0);
  Eigen::Matrix4f init_guess = (init_translation * init_rotation).matrix ();
  //计算需要的刚体变换以便将输入的点云匹配到目标点云
  pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  ndt.align (*output_cloud, init_guess);
  std::cout << "Normal Distributions Transform has converged:" << ndt.hasConverged ()
            << " score: " << ndt.getFitnessScore () << std::endl;
  //使用创建的变换对未过滤的输入点云进行变换
  pcl::transformPointCloud (*input_cloud, *output_cloud, ndt.getFinalTransformation ());
  //保存转换的输入点云
  pcl::io::savePCDFileASCII ("room_scan2_transformed.pcd", *output_cloud);
  // 初始化点云可视化界面
  boost::shared_ptr<pcl::visualization::PCLVisualizer>
  viewer_final (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  //第一个窗口显示input_cloud,第二个窗口显示target_cloud,第三个窗口显示配准之后的结果
  int v1(0),v2(0),v3(0);
  viewer_final->createViewPort(0,0,0.5,1,v1);//第一个窗口位置、颜色
  viewer_final->setBackgroundColor (0,0,0,v1);
  viewer_final->createViewPort(0.5,0,1,1,v2);//第二个窗口位置以及背景颜色
  viewer_final->setBackgroundColor (0,0,0,v2);
 // viewer_final->createViewPort (0.66,0,1,1,v3);//第三个窗口位置、颜色
 // viewer_final->setBackgroundColor (0, 0, 0,v3);
  //对待一个窗口显示
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> input_color(input_cloud,0,255,0);
  viewer_final->addPointCloud<pcl::PointXYZ>(input_cloud,input_color,"input color",v1);

  //对目标点云着色（红色）并可视化
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
  target_color (target_cloud, 255, 0, 0);
  //对第二个视口显示
  viewer_final->addPointCloud<pcl::PointXYZ>(target_cloud, target_color, "target cloud0",v1);
  viewer_final->addPointCloud<pcl::PointXYZ> (target_cloud, target_color, "target cloud",v2 );
  viewer_final->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                                  1, "target cloud");
  //对转换后的目标点云着色（绿色）并可视化
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
  output_color (output_cloud, 0, 255, 0);
  viewer_final->addPointCloud<pcl::PointXYZ> (output_cloud, output_color, "output cloud",v2);
  viewer_final->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                                  1, "output cloud");
  // 启动可视化
  viewer_final->addCoordinateSystem (1.0);
  viewer_final->initCameraParameters ();
  //等待直到可视化窗口关闭。
  while (!viewer_final->wasStopped ())
  {
    viewer_final->spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }
  return (0);
}