/*该类的功能主要是接受实时三维点云帧，进行100hz的三种格式的发布，包括三维点云、
 三维栅格、二维栅格。
 edited by goldqiu -2022-04-9
*/

#include "map_conversion/utility.hpp"

int main(int argc, char** argv)
{
   google::InitGoogleLogging(argv[0]);
   FLAGS_log_dir = WORK_SPACE_PATH + "/Log";
   FLAGS_alsologtostderr = 1;

   ros::init(argc, argv, "test_frame_node");
   ros::NodeHandle nh; //创建ros句柄
   //创建读取参数的对象
   YAML::Node config = YAML::LoadFile(WORK_SPACE_PATH+"/config/params.yaml");

   //创建点云处理对象，传入参数为前面创建的读取参数的对象
   Pointcloud2dProcess *pointcloud_2d_process_ptr = new Pointcloud2dProcess(config);
   Pointcloud3dProcess *pointcloud_3d_process_ptr = new Pointcloud3dProcess(config);

   //创建ros发布和接受话题对象
   std::shared_ptr<CloudSubscriber> cloud_sub_ptr = std::make_shared<CloudSubscriber>(nh, pointcloud_2d_process_ptr->local_cloud_topic, 100000);
   std::shared_ptr<CloudPublisher> local_cloud_pub_ptr = std::make_shared<CloudPublisher>(nh, "pcl_output", "/camera_init", 100);
   std::shared_ptr<CloudPublisher> local_3d_grid_pub_ptr = std::make_shared<CloudPublisher>(nh, "inflation_map", "/camera_init", 100);

   //这里创建发布二维占据栅格地图对象并没有封装为类
   ros::Publisher local_grid_map_pub = nh.advertise<nav_msgs::OccupancyGrid>("/grid_map_realtime", 100);
  
   nav_msgs::OccupancyGrid local_grid_map_msg;
   std::deque<CloudData> cloud_data_buff; //实时点云buffer

   ros::Rate loop_rate(100.0);//局部地图的更新频率为100hz
   while(ros::ok())
   {
     ros::spinOnce();
     cloud_sub_ptr->ParseData(cloud_data_buff);//从封装好的点云话题接受类内取数据
     if(cloud_data_buff.size() > 0)
     {

       CloudData cloud_data = cloud_data_buff.front();  //存到buffer

       //发布实时三维点云
       pointcloud_2d_process_ptr->find_Z_value(cloud_data);
       pointcloud_2d_process_ptr->PassThroughFilter(cloud_data,cloud_data,false);
       local_cloud_pub_ptr->Publish(cloud_data.cloud_ptr);

       //发布实时二维栅格
       pointcloud_2d_process_ptr->Pointcloud_to_2d_grid(cloud_data,local_grid_map_msg);
       local_grid_map_msg.header.frame_id = "camera_init";
       local_grid_map_pub.publish(local_grid_map_msg);

       //发布实时三维栅格
       CloudData grid_cloud_data;
       pointcloud_3d_process_ptr->pointcloud_to_3dGridMap(cloud_data,grid_cloud_data);
       local_3d_grid_pub_ptr->Publish(grid_cloud_data.cloud_ptr);


       cloud_data_buff.pop_front();
     }
     loop_rate.sleep();
   }
   return 0;
}

