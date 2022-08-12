#include "visualization.h"

sensor_msgs::PointCloud transToPointCloud(const kit::perception::fusion::FusionObjectList& data)
{
    sensor_msgs::PointCloud cloud;
    cloud.header.stamp = ros::Time::now();
    cloud.header.frame_id = "map";
    cloud.points.resize(data.objs.size());
    //we'll also add an intensity channel to the cloud
    cloud.channels.resize(1);
    cloud.channels[0].name = "rgb";
    cloud.channels[0].values.resize(data.objs.size());

    //generate some fake data for our point cloud
    for (unsigned int i = 0; i < data.objs.size(); ++i)
    {
        cloud.points[i].x = data.objs[i]->x;
        cloud.points[i].y = data.objs[i]->y;
        cloud.points[i].z = data.objs[i]->z;
        cloud.channels[0].values[i] = 255;
    }
    return cloud;
}

sensor_msgs::PointCloud transToPointCloud(const kit::perception::fusion::LiDARObjectList& data)
{
    sensor_msgs::PointCloud cloud;
    cloud.header.stamp = ros::Time::now();
    cloud.header.frame_id = "map";
    cloud.points.resize(data.objs.size());
    //we'll also add an intensity channel to the cloud
    cloud.channels.resize(1);
    cloud.channels[0].name = "rgb";
    cloud.channels[0].values.resize(data.objs.size());

    //generate some fake data for our point cloud
    for (unsigned int i = 0; i < data.objs.size(); ++i)
    {
        cloud.points[i].x = data.objs[i]->x;
        cloud.points[i].y = data.objs[i]->y;
        cloud.points[i].z = data.objs[i]->z;
        cloud.channels[0].values[i] = 255;
    }
    return cloud;
}
