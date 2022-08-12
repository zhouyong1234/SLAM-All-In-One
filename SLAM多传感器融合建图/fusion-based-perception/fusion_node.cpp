#include <ros/ros.h>
#include <string>
#include <list>
#include <iostream>
#include "input_data_type.h"
#include <interpreter.hpp>
#include <boost/filesystem.hpp>
#include "fusion/fusion_wrapper.h"
#include "frame_build.h"
#include "visualization.h"

inline void printMregeRes(const std::vector<std::tuple<double, proto_input::FileTag, std::string> >& data)
{
    for (auto& v : data)
        std::cout<<std::get<1>(v)<<" : "<<std::get<2>(v) << std::endl;
}

int main(int argc, char** argv)
{
    //argv[1] config, argv[2] location, argv[3] camera, argv[4] lidar, argv[5] ground truth
    ros::init(argc, argv, "fusion_node");
    if (argc != 5)
    {
        std::cout << "Need 4 args!\n";
        return 0;
    }

    //File name is time stamp, all object detected by same sensor build a input sequence by function buildSensorInputSequence.
    //All input sequence sorted by time stamp and merge into one input sequence by function mergeInputSequence.
    //Next, earse the element so that the each four adjacent elements have different FileTag and sort the four adjacent elements by FileTag.
    std::vector<std::tuple<double, proto_input::FileTag, std::string> >&& data_frame_seq = proto_input::mergeInputSequence({
        proto_input::buildSensorInputSequence(argv[2], "Location", proto_input::FileTag::LOCATION),
        proto_input::buildSensorInputSequence(argv[3], "Lidar", proto_input::FileTag::CAMERA),
        proto_input::buildSensorInputSequence(argv[4], "Lidar", proto_input::FileTag::LIDAR),
        //proto_input::buildSensorInputSequence(argv[5], "Radar", FileTag::RADAR),
    });
    //uncomment follow line to check input Sequence
    //printMregeRes(data_frame_seq);

    ros::NodeHandle nh;
    ros::Publisher lidar_pub = nh.advertise<sensor_msgs::PointCloud>("lidar_det", 50);
    ros::Publisher res_pub = nh.advertise<sensor_msgs::PointCloud>("fusion_result", 50);
    ros::Rate r(2);

    //ezcfg::Interpreter is a protobuf interpreter that no need proto file. It refers to the struct define itself as proto file.
    ezcfg::Interpreter itp;
    kit::perception::fusion::FusionWrapper fusion{ argv[1] };
    for (auto  it = data_frame_seq.begin(); it != data_frame_seq.end();)
    {
        //get vehicle position and orientation
        //Never mind the itp usage. You just know if a name can be found in input file and struct define,
        //the value of the name will automatically assign to the element in the struct, the same below.
        itp.loadFile(std::get<2>(*it));
        proto_input::Location pose;
        itp.lex.matchID("header");
        itp.parse(pose.header);
        itp.lex.matchID("pose");
        itp.parse(pose.pose);
        ++it;

        std::shared_ptr<kit::perception::fusion::Frame> frame = std::make_shared<kit::perception::fusion::Frame>();
        //get camera detection results
        if (std::get<1>(*it) == proto_input::FileTag::CAMERA)
        {
            proto_input::CameraObject raw_co;
            kit::perception::fusion::CameraObjectListPtr co_list = std::make_shared<kit::perception::fusion::CameraObjectList>();
            itp.loadFile(std::get<2>(*it));
            while (itp.lex.getToken() == ezcfg::Token::ID && itp.lex.getTokenText() == "perception_obstacle")
            {
                itp.lex.next();
                itp.parse(raw_co);
                co_list->objs.push_back(makeCameraObjectPtr(raw_co, std::get<0>(*it)));
            }
            co_list->time_ns = std::get<0>(*it);
            frame->camera_objs = std::move(co_list);
            ++it;
        }
        //get lidar detection results
        if (std::get<1>(*it) == proto_input::FileTag::LIDAR)
        {
            proto_input::LidarObject raw_lo;
            kit::perception::fusion::LiDARObjectListPtr lo_list = std::make_shared<kit::perception::fusion::LiDARObjectList>();
            itp.loadFile(std::get<2>(*it));
            while (itp.lex.getToken() == ezcfg::Token::ID && itp.lex.getTokenText() == "perception_obstacle")
            {
                itp.lex.next();
                itp.parse(raw_lo);
                raw_lo.anchor_point.x = raw_lo.anchor_point.x - pose.pose.position.x;
                raw_lo.anchor_point.y = raw_lo.anchor_point.y - pose.pose.position.y;
                raw_lo.anchor_point.z = raw_lo.anchor_point.z - pose.pose.position.z;
                lo_list->objs.push_back(makeLiDARObjectPtr(raw_lo, std::get<0>(*it)));
            }
            lidar_pub.publish(transToPointCloud(*lo_list));
            lo_list->time_ns = std::get<0>(*it);
            frame->lidar_objs = std::move(lo_list);
            ++it;
        }

        //fusion start
        fusion.Update(frame);
        kit::perception::fusion::FusionObjectListPtr fusion_res = std::make_shared<kit::perception::fusion::FusionObjectList>(kit::perception::fusion::FusionObjectList());
        fusion.GetFusionResult(fusion_res);
        
        //result output
        std::cout << "********************************************************************\n";
        for (size_t i = 0; i < fusion_res->objs.size(); i++)
            std::cout << fusion_res->objs[i]->ToString() << std::endl;
        res_pub.publish(transToPointCloud(*fusion_res));
        std::cout << "********************************************************************\n";
        r.sleep();
    }

    ros::shutdown();
    return 0;
}
