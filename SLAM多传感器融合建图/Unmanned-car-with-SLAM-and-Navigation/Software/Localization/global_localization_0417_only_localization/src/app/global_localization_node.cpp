#include <ros/ros.h>
#include "glog/logging.h"
#include "global_localization/global_defination/global_defination.h"
#include "global_localization/matching/matching.hpp"
#include "global_localization/publisher/cloud_publisher.hpp"


#include <pcl/io/pcd_io.h>

using namespace global_localization;

int main(int argc, char *argv[]) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_log_dir = WORK_SPACE_PATH + "/Log";
    FLAGS_alsologtostderr = 1;

    ros::init(argc, argv, "global_localization_node");
    ros::NodeHandle nh; //创建ros句柄

    //创建全局定位matching对象
    std::shared_ptr<Matching> matching_ptr_ = std::make_shared<Matching>(nh);
    //创建ros发布话题对象
    std::shared_ptr<CloudPublisher> global_map_pub_ptr_ = std::make_shared<CloudPublisher>(nh, "/global_map", "/map", 100);
    std::shared_ptr<CloudPublisher> local_map_pub_ptr_ = std::make_shared<CloudPublisher>(nh, "/local_map", "/map", 100);
    std::shared_ptr<CloudPublisher> current_scan_pub_ptr_ = std::make_shared<CloudPublisher>(nh, "/current_scan", "/map", 100);

    ros::Rate rate(100);
    while (ros::ok()) {
        ros::spinOnce();

        //发布全局地图，只发送一次
        if (matching_ptr_->HasNewGlobalMap() && global_map_pub_ptr_->HasSubscribers()) {
        CloudData::CLOUD_PTR global_map_ptr(new CloudData::CLOUD());
        matching_ptr_->GetGlobalMap(global_map_ptr);
        global_map_pub_ptr_->Publish(global_map_ptr);
        }
        //发送全局子地图
        if(matching_ptr_->HasNewLocalMap() && local_map_pub_ptr_->HasSubscribers())
        local_map_pub_ptr_->Publish(matching_ptr_->GetLocalMap());

        //读取数据帧，进行位姿初始化和点云配准，然后发布点云和位姿。
        matching_ptr_->ReadData();
        while(matching_ptr_->HasData()) {
            if (!matching_ptr_->ValidData()) {
                LOG(INFO) << "Invalid data. Skip matching" << std::endl;
                continue;
            }
            
            if (matching_ptr_->UpdateMatching())  //进行位姿初始化，成功后才发布
            {
                matching_ptr_->PublishData();  //发布话题数据
            }
        }
        rate.sleep();
    }

    return 0;
}