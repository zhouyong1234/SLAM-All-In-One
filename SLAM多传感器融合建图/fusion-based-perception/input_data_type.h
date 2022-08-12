#pragma once

namespace proto_input
{
    struct Point3D
    {
        double x;
        double y;
        double z;
    };
    struct BBox2D
    {
        double xmin;
        double ymin;
        double xmax;
        double ymax;
    };
    struct Header
    {
        double timestamp_sec;
    };
    struct Quaternion
    {
        double qx;
        double qy;
        double qz;
        double qw;
    };
    struct Pose
    {
        Point3D position;
        Quaternion orientation;
    };

    struct Location
    {
        Header header;
        Pose pose;
    };

    struct LidarObject
    {
        int id;
        Point3D velocity;
        double length;
        double width;
        double height;
        Point3D anchor_point;
    };
    struct CameraObject
    {
        int id;
        BBox2D bbox2d;
    };
}