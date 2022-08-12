#include "utils.h"

namespace kit {
namespace perception {
namespace fusion {

// Helper function to convert hex int to OpenCV RGB (scalar) color
cv::Scalar Hex2RGB(int hexValue) {
    cv::Scalar rgbColor;
    int r = ((hexValue >> 16) & 0xFF);  // Extract the RR byte
    int g = ((hexValue >> 8) & 0xFF);   // Extract the GG byte
    int b = ((hexValue)&0xFF);          // Extract the BB byte
    return CV_RGB(r, g, b);
}

float DistanceIn3D(const FusionObjectPtr &local,
                   const FusionObjectPtr &global) {
    return std::sqrt(std::pow((local->x - global->x), 2) +
                     std::pow((local->y - global->y), 2));
}

void Project3DBoxTo2DBox(
        const BBox3D &box_3d, const Eigen::Affine3d &extrinsic,
        const Vector6d &intrinsic, BBox2D &box_2d) {
    // std::cout << "box_3d: " << box_3d.x << " " <<  box_3d.y << " " << box_3d.z
    //     << " " << box_3d.length << " " << box_3d.width << " " << box_3d.height
    //     << std::endl;
    std::vector<Eigen::Vector2d> vertices;
    std::vector<std::vector<int>> indices = {
        {-1, -1, -1}, {1, -1, -1}, {1, 1, -1}, {-1, 1, -1},
        {-1, -1, 1}, {1, -1, 1}, {1, 1, 1}, {-1, 1, 1}
    };
    for (size_t i = 0; i < indices.size(); ++i) {
        Point3D pt;
        pt.x = box_3d.x + indices[i][0] * box_3d.length * 0.5;
        pt.y = box_3d.y + indices[i][1] * box_3d.width * 0.5;
        pt.z = box_3d.z + indices[i][2] * box_3d.height * 0.5;
        Eigen::Vector2d verty = ProjectPoint2Image(
                Eigen::Vector3d(pt.x, pt.y, pt.z), extrinsic, intrinsic);
        vertices.push_back(verty);
    }

    double min_x = std::numeric_limits<double>::max();
    double max_x = std::numeric_limits<double>::min();
    double min_y = std::numeric_limits<double>::max();
    double max_y = std::numeric_limits<double>::min();
    for (size_t i = 0; i < vertices.size(); ++i) {
        min_x = std::min(vertices[i](0), min_x);
        max_x = std::max(vertices[i](0), max_x);
        min_y = std::min(vertices[i](1), min_y);
        max_y = std::max(vertices[i](1), max_y);
    }
    min_x = std::max(min_x, 0.0);
    min_y = std::max(min_y, 0.0);
    max_x = std::max(max_x, 0.0);
    max_y = std::max(max_y, 0.0);
    min_x = std::min(min_x, 1920.0);
    min_y = std::min(min_y, 1020.0);
    max_x = std::min(max_x, 1920.0);
    max_y = std::min(max_y, 1020.0);

    box_2d.x = (max_x + min_x) * 0.5;
    box_2d.y = (max_y + min_y) * 0.5;
    box_2d.width = max_x - min_x;
    box_2d.height = max_y - min_y;
}

Eigen::Vector2d ProjectPoint2Image(const Eigen::Vector3d& point,
        const Eigen::Affine3d& extrinsic, const Vector6d& intrinsic) {
    // std::cout << "point: " << point(0) << " " << point(1) << " " << point(2) << std::endl;
    // std::cout << "extrinsic: " << extrinsic.linear() << "\n"
    //     << "intrinsic: " << intrinsic << std::endl;
    Eigen::Vector3d pt_in_cam =
        extrinsic.linear() * point + extrinsic.translation();

    Eigen::Vector2d pt_in_img;
    pt_in_img(0) =
        (intrinsic(0) * pt_in_cam.x() + intrinsic(2)) / pt_in_cam.z() + intrinsic(4);
    pt_in_img(1) =
        (intrinsic(1) * pt_in_cam.y() + intrinsic(3)) / pt_in_cam.z() + intrinsic(5);
    return pt_in_img;
}

float IoUIn2D(const BBox2D &box_2d, const BBox3D &box_3d,
        const Eigen::Affine3d& extrinsic, const Vector6d& intrinsic) {
    BBox2D tgt;
    Project3DBoxTo2DBox(box_3d, extrinsic, intrinsic, tgt);
    // std::cout << "pred: "
    //     << box_2d.x << " " << box_2d.y << " " << box_2d.width << " " << box_2d.height
    //     << "\ntgt:"
    //     << tgt.x << " " << tgt.y << " " << tgt.width << " " << tgt.height
    //     << std::endl;
    return IoUIn2D(box_2d, tgt);
}

float IoUIn2D(const FusionObjectPtr &local, const FusionObjectPtr &global) {
    BBox2D pred, tgt;
    pred.x = local->ux;
    pred.y = local->vy;
    pred.width = local->width_2d;
    pred.height = local->height_2d;
    tgt.x = global->ux;
    tgt.y = global->vy;
    tgt.width = global->width_2d;
    tgt.height = global->height_2d;

    return IoUIn2D(pred, tgt);
}

float IoUIn2D(const BBox2D &pred, const BBox2D &tgt) {
    float x1_min = pred.x - pred.width * 0.5;
    float y1_min = pred.y - pred.height * 0.5;
    float x1_max = pred.x + pred.width * 0.5;
    float y1_max = pred.y + pred.height * 0.5;
    float x2_min = tgt.x - tgt.width * 0.5;
    float y2_min = tgt.y - tgt.height * 0.5;
    float x2_max = tgt.x + tgt.width * 0.5;
    float y2_max = tgt.y + tgt.height * 0.5;

    // std::cout
    //     << "(x1_min, y1_min)= (" << x1_min << ", " << y1_min << "),"
    //     << "(x1_max, y1_max)= (" << x1_max << ", " << y1_max << "),"
    //     << "(x2_min, y2_min)= (" << x2_min << ", " << y2_min << "),"
    //     << "(x2_max, y2_max)= (" << x2_max << ", " << y2_max << "),"
    //     << std::endl;

    if ((x1_max <= x2_min || y1_max <= y2_min) ||
        (x2_max <= x1_min || y2_max <= y1_min)) {
        return 0.0;
    }

    float x_min = std::max(x1_min, x2_min);
    float y_min = std::max(y1_min, y2_min);
    float x_max = std::min(x1_max, x2_max);
    float y_max = std::min(y1_max, y2_max);
    float inter = (y_max - y_min) * (x_max - x_min);
    float uni = (pred.width * pred.height + tgt.width * tgt.height) - inter;

    return inter / uni;
}

float IoUIn3D(const LiDARObjectPtr& local, const FusionObjectPtr& global) {
    // treat 3D box as BEV 2D box
    BBox2D pred, tgt;

    pred.x = local->x - global->x;
    pred.y = local->y - global->y;
    pred.height = local->length;
    pred.width = local->width;

    tgt.x = 0;
    tgt.y = 0;
    tgt.height = global->length;
    tgt.width = global->width;
    return  IoUIn2D(pred, tgt);

    // treat 3D box as box with pose
    // BBox3D pred, tgt;
    // pred.x = local->x;
    // pred.y = local->y;
    // pred.z = local->z;
    // pred.length = local->length;
    // pred.width = local->width;
    // pred.height = local->height;

    // tgt.x = global->x;
    // tgt.y = global->y;
    // tgt.z = global->z;
    // tgt.length = global->length;
    // tgt.width = global->width;
    // tgt.height = global->height;
    // 
    // return IoUIn3D(pred, tgt);
}

float IoUIn3D(const BBox3D &pred, const BBox3D& tgt) {
    Point base(std::min(pred.x, tgt.x), std::min(pred.y, tgt.y));
    Point p0(pred.x - base.x - pred.length * 0.5, pred.y - base.y - pred.width * 0.5);
    Point p1(pred.x - base.x + pred.length * 0.5, pred.y - base.y - pred.width * 0.5);
    Point p2(pred.x - base.x + pred.length * 0.5, pred.y - base.y + pred.width * 0.5);
    Point p3(pred.x - base.x - pred.length * 0.5, pred.y - base.y + pred.width * 0.5);
    Quad pred_q(p0, p1, p2, p3); 
    // std::cout << "(" << p0.x << ", " << p0.y << ")"
    //     << "(" << p1.x << ", " << p1.y << ")"
    //     << "(" << p2.x << ", " << p2.y << ")"
    //     << "(" << p3.x << ", " << p3.y << ")"
    //     << std::endl;

    Point t0(tgt.x - base.x - tgt.length * 0.5, tgt.y - base.y - tgt.width * 0.5);
    Point t1(tgt.x - base.x + tgt.length * 0.5, tgt.y - base.y - tgt.width * 0.5);
    Point t2(tgt.x - base.x + tgt.length * 0.5, tgt.y - base.y + tgt.width * 0.5);
    Point t3(tgt.x - base.x - tgt.length * 0.5, tgt.y - base.y + tgt.width * 0.5);
    Quad tgt_q(t0, t1, t2, t3); 
    // std::cout << "(" << t0.x << ", " << t0.y << ")"
    //     << "(" << t1.x << ", " << t1.y << ")"
    //     << "(" << t2.x << ", " << t2.y << ")"
    //     << "(" << t3.x << ", " << t3.y << ")"
    //     << std::endl;

    return iou(pred_q, tgt_q);
}

}  // namespace fusion
}  // namespace perception
}  // namespace kit
