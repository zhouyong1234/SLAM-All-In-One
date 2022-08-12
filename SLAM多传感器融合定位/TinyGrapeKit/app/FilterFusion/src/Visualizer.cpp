#include <FilterFusion/Visualizer.h>

#include <pangolin/pangolin.h>

namespace FilterFusion {

Visualizer::Visualizer(const Config& config) : config_(config) { 
    // Start viz_thread.
    viz_thread_ = std::make_shared<std::thread>(&Visualizer::Run, this);
}

void Visualizer::DrawCameras(const std::vector<std::pair<Eigen::Matrix3d, Eigen::Vector3d>>& camera_poses) {
    std::lock_guard<std::mutex> lg(data_buffer_mutex_);
    camera_poses_ = camera_poses;
}

void Visualizer::DrawWheelPose(const Eigen::Matrix3d& G_R_O, const Eigen::Vector3d& G_p_O) {
    std::lock_guard<std::mutex> lg(data_buffer_mutex_);
    wheel_traj_.emplace_back(G_R_O, G_p_O);
    if (wheel_traj_.size() > config_.max_traj_length) { wheel_traj_.pop_front(); }
}

void Visualizer::DrawGroundTruth(const Eigen::Matrix3d& G_R_O, const Eigen::Vector3d& G_p_O) {
    std::lock_guard<std::mutex> lg(data_buffer_mutex_);
    gt_wheel_traj_.emplace_back(G_R_O, G_p_O);
    if (gt_wheel_traj_.size() > config_.max_traj_length) { gt_wheel_traj_.pop_front(); }
}

void Visualizer::DrawWheelOdom(const Eigen::Matrix3d& G_R_O, const Eigen::Vector3d& G_p_O) {
    std::lock_guard<std::mutex> lg(data_buffer_mutex_);
    wheel_odom_traj_.emplace_back(G_R_O, G_p_O);
    if (wheel_odom_traj_.size() > config_.max_traj_length) { wheel_odom_traj_.pop_front(); }
}

void Visualizer::DrawFeatures(const std::vector<Eigen::Vector3d>& features) {
    std::lock_guard<std::mutex> lg(data_buffer_mutex_);
    for (const Eigen::Vector3d& ft : features) {
        features_.push_back(ft);
    }
    while (features_.size() > config_.max_num_features) { features_.pop_front(); } 
}

void Visualizer::DrawColorImage(const cv::Mat& image) {
    std::lock_guard<std::mutex> lg(data_buffer_mutex_);
    cv::resize(image, image_, cv::Size(config_.img_width, config_.img_height), 0., 0., cv::INTER_NEAREST);
    cv::flip(image_, image_, 0);
}

void Visualizer::DrawGps(const Eigen::Vector3d& G_p_Gps) {
    std::lock_guard<std::mutex> lg(data_buffer_mutex_);
    gps_points_.push_back(G_p_Gps);
    if (gps_points_.size() > config_.max_gps_length) {
        gps_points_.pop_front();
    }
}

void Visualizer::DrawImage(const cv::Mat& image, 
                           const std::vector<Eigen::Vector2d>& tracked_fts, 
                           const std::vector<Eigen::Vector2d>& new_fts) {
    // Covert gray image to color image.
    cv::Mat color_img;
    cv::cvtColor(image, color_img, cv::COLOR_GRAY2BGR);

    // Draw features on image.
    for (const Eigen::Vector2d& ft : tracked_fts) {
        cv::circle(color_img, cv::Point(ft[0], ft[1]), 5, cv::Scalar(0, 255, 0), -1);
    }
    for (const Eigen::Vector2d& ft : new_fts) {
        cv::circle(color_img, cv::Point(ft[0], ft[1]), 5, cv::Scalar(255, 0, 0), -1);
    }
    
    std::lock_guard<std::mutex> lg(data_buffer_mutex_);
    cv::resize(color_img, image_, cv::Size(config_.img_width, config_.img_height));
    cv::flip(image_, image_, 0);
}

pangolin::OpenGlMatrix SE3ToOpenGlMat(const Eigen::Matrix3d& G_R_C, const Eigen::Vector3d& G_p_C) {
    pangolin::OpenGlMatrix p_mat;
    
    p_mat.m[0] = G_R_C(0, 0);
    p_mat.m[1] = G_R_C(1, 0);
    p_mat.m[2] = G_R_C(2, 0);
    p_mat.m[3] = 0.;

    p_mat.m[4] = G_R_C(0, 1);
    p_mat.m[5] = G_R_C(1, 1);
    p_mat.m[6] = G_R_C(2, 1);
    p_mat.m[7] = 0.;

    p_mat.m[8] = G_R_C(0, 2);
    p_mat.m[9] = G_R_C(1, 2);
    p_mat.m[10] = G_R_C(2, 2);
    p_mat.m[11] = 0.;

    p_mat.m[12] = G_p_C(0);
    p_mat.m[13] = G_p_C(1);
    p_mat.m[14] = G_p_C(2);
    p_mat.m[15] = 1.;

    return p_mat;
}

void Visualizer::DrawOneCamera(const Eigen::Matrix3d& G_R_C, const Eigen::Vector3d& G_p_C) {
    const float w = config_.cam_size;
    const float h = w * 0.75;
    const float z = w * 0.6;

    pangolin::OpenGlMatrix G_T_C = SE3ToOpenGlMat(G_R_C, G_p_C);

    glPushMatrix();

#ifdef HAVE_GLES
    glMultMatrixf(G_T_C.m);
#else
    glMultMatrixd(G_T_C.m);
#endif

    glLineWidth(config_.cam_line_width);
    glBegin(GL_LINES);
    glVertex3f(0, 0, 0);
    glVertex3f(w, h, z);
    glVertex3f(0, 0, 0);
    glVertex3f(w, -h, z);
    glVertex3f(0, 0, 0);
    glVertex3f(-w, -h, z);
    glVertex3f(0, 0, 0);
    glVertex3f(-w, h, z);

    glVertex3f(w, h, z);
    glVertex3f(w, -h, z);

    glVertex3f(-w, h, z);
    glVertex3f(-w, -h, z);

    glVertex3f(-w, h, z);
    glVertex3f(w, h, z);

    glVertex3f(-w, -h, z);
    glVertex3f(w, -h, z);
    glEnd();

    glPopMatrix();

    pangolin::glDrawAxis(G_T_C, 0.2);
}

void Visualizer::DrawCameras() {
    std::lock_guard<std::mutex> lg(data_buffer_mutex_);
    for (const std::pair<Eigen::Matrix3d, Eigen::Vector3d>& cam : camera_poses_) {
        DrawOneCamera(cam.first, cam.second);
    }
}

void Visualizer::DrawTraj(const std::deque<std::pair<Eigen::Matrix3d, Eigen::Vector3d>>& traj_data) {
    glLineWidth(config_.cam_line_width);
    glBegin(GL_LINE_STRIP);

    std::lock_guard<std::mutex> lg(data_buffer_mutex_);
    for (const auto& wheel_pose : traj_data) {
        const Eigen::Vector3d& G_p_O = wheel_pose.second;
        glVertex3f(G_p_O[0], G_p_O[1], G_p_O[2]);
    }

    glEnd();
}

void Visualizer::DrawFeatures() {
    glPointSize(config_.point_size);
    glBegin(GL_POINTS);

    std::lock_guard<std::mutex> lg(data_buffer_mutex_);
    for (const Eigen::Vector3d& pt : features_) {
        glVertex3f(pt[0], pt[1], pt[2]);
    }

    glEnd();
}

void Visualizer::DrawWheeFrame(const Eigen::Matrix3d& G_R_O, const Eigen::Vector3d& G_p_O) {
    pangolin::OpenGlMatrix G_T_O = SE3ToOpenGlMat(G_R_O, G_p_O); 
    pangolin::glDrawAxis(G_T_O, config_.wheel_frame_size);
}

void Visualizer::DrawWheeFrame() {
    std::lock_guard<std::mutex> lg(data_buffer_mutex_);
    if (wheel_traj_.empty()) { return; }
    DrawWheeFrame(wheel_traj_.back().first, wheel_traj_.back().second);
}

void Visualizer::DrawGpsPoints() {
    glPointSize(config_.gps_point_size);
    glBegin(GL_POINTS);

    std::lock_guard<std::mutex> lg(data_buffer_mutex_);
    for (const Eigen::Vector3d& pt : gps_points_) {
        glVertex3f(pt[0], pt[1], pt[2]);
    }

    glEnd();
}

void Visualizer::Run() {
    pangolin::CreateWindowAndBind("TinyGrapeKit-FilterFusion", 1920, 1080);
    glEnable(GL_DEPTH_TEST);

    // Issue specific OpenGl we might need
    glEnable (GL_BLEND);
    glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    // Menu.
    pangolin::CreatePanel("menu").SetBounds(0.0, 1.0, 0.0, pangolin::Attach::Pix(175));

    pangolin::Var<bool> menu_follow_cam("menu.Follow Camera", true, true);
    pangolin::Var<int> grid_scale("menu.Grid Size (m)", 100, 1, 500);
    pangolin::Var<bool> show_grid("menu.Show Grid", true, true);
    pangolin::Var<bool> show_map("menu.Show Map", true, true);
    pangolin::Var<bool> show_cam("menu.Show Camera", true, true);
    pangolin::Var<bool> show_traj("menu.Show Traj", true, true);
    pangolin::Var<bool> show_gt_traj("menu.Show GroundTruth", true, true);
    pangolin::Var<bool> show_raw_odom("menu.Show Raw Odom", config_.show_raw_odom, true);
    pangolin::Var<bool> show_gps_point("menu.Show GPS", config_.show_gps_points, true);

    // Define Camera Render Object (for view / scene browsing)
    pangolin::OpenGlRenderState s_cam(
        pangolin::ProjectionMatrix(1920, 1080, config_.view_point_f, config_.view_point_f, 960, 540, 0.1, 10000),
        pangolin::ModelViewLookAt(config_.view_point_x, config_.view_point_y, config_.view_point_z, 0, 0, 0, 1, 0, 0));
                      
    // Add named OpenGL viewport to window and provide 3D Handler
    pangolin::View& d_cam = pangolin::CreateDisplay()
        .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1920.0f/1080.0f)
        .SetHandler(new pangolin::Handler3D(s_cam));

    // Draw image.
    pangolin::View& d_image = pangolin::Display("image")
      .SetBounds(0.8f, 1.0f, 0.1, 1./3, config_.img_width / config_.img_height)
      .SetLock(pangolin::LockLeft, pangolin::LockTop);
    pangolin::GlTexture image_texture(config_.img_width, config_.img_height, GL_RGB, true, 0, GL_RGB, GL_UNSIGNED_BYTE);

    pangolin::OpenGlMatrix G_T_C;
    G_T_C.SetIdentity();

    running_flag_ = true;
    while (running_flag_) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        
        d_cam.Activate(s_cam);
        glClearColor(0.0f, 0.0f, 0.0f,1.0f);
        {
            std::lock_guard<std::mutex> lg(data_buffer_mutex_);
            if(menu_follow_cam && !camera_poses_.empty()){
                G_T_C = SE3ToOpenGlMat(camera_poses_.back().first, camera_poses_.back().second);
                s_cam.Follow(G_T_C);
            }
        }

        // Draw grid.
        if (show_grid.Get()) {
            glColor3f(0.3f, 0.3f, 0.3f);
            pangolin::glDraw_z0(grid_scale, 1000);
        }

        // Draw wheel traj.
        if (show_traj.Get()) {
            glColor3f(1.0f, 0.0f, 0.0f);
            DrawTraj(wheel_traj_);
            DrawWheeFrame();
        }

        // Draw gt wheel traj.
        if (show_gt_traj.Get()) {
            glColor3f(0.0f, 1.0f, 0.0f);
            DrawTraj(gt_wheel_traj_);
        }

        // Draw raw odometry.
        if (show_raw_odom.Get()) {
            glColor3f(1.0f, 0.0f, 1.0f);
            DrawTraj(wheel_odom_traj_);
        }

        // Draw camera poses.
        if (show_cam.Get()) {
            glColor3f(0.0f, 1.0f, 0.0f);
            DrawCameras();
        }

        // Draw map points.
        if (show_map.Get()) {
            glColor3f(0.0f, 0.0f, 1.0f);
            DrawFeatures();
        }

        // Draw Gps points.
        if (show_gps_point.Get()) {
            glColor3f(0.0f, 1.0f, 1.0f);
            DrawGpsPoints();
        }

        // Draw image
        {
            std::lock_guard<std::mutex> lg(data_buffer_mutex_);
            if (!image_.empty()) {
                image_texture.Upload(image_.data, GL_RGB, GL_UNSIGNED_BYTE);
                d_image.Activate();
                glColor3f(1.0, 1.0, 1.0);
                image_texture.RenderToViewport();
            }
        }
        pangolin::FinishFrame();
    }
}

}  // namespace FilterFusion