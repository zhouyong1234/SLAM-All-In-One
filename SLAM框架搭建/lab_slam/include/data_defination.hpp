#ifndef DATA_CENTER_HPP
#define DATA_CENTER_HPP

#include <utility.hpp>
#include <deque>
#include <unordered_map>
#include <mutex>
#include <sensor_msgs/PointCloud2.h>

struct DataGroup{
    DataGroup():corner_cloud(new PointCloudXYZI), plane_cloud(new PointCloudXYZI),
        less_corner_cloud(new PointCloudXYZI), less_plane_cloud(new PointCloudXYZI){}
    double time_stamp;
    PointCloudXYZIPtr corner_cloud;
    PointCloudXYZIPtr less_corner_cloud;
    PointCloudXYZIPtr plane_cloud;
    PointCloudXYZIPtr less_plane_cloud;
    std_msgs::Header h;
    // debug
    sensor_msgs::PointCloud2::ConstPtr cloud_msg;
    bool is_finished = false;
};
typedef boost::shared_ptr<DataGroup> DataGroupPtr;

class DataCenter{
public:
    static DataCenter* Instance(){
        static DataCenter* p = new DataCenter;
        return p;
    }

    void push_back(const DataGroupPtr& data){
        data_.push_back(data);
    }

    bool pop_front(){
        if(!data_.empty()){
           data_.pop_front();
           return true;
        }
        return false;
    }
private:
    std::deque<DataGroupPtr> data_;
};
static std::mutex data_center_mutex;

#endif