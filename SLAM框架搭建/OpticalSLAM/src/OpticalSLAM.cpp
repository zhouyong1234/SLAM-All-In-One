#include "OpticalSLAM.h"

OpticalSLAM::OpticalSLAM()
{
    Rp.resize(WINDOW_SIZE);
    tp.resize(WINDOW_SIZE);
}

OpticalSLAM::~OpticalSLAM()
{

}

void OpticalSLAM::initialize()
{
	is_first_img = false;
    ready_to_display = false;
	nFrames = 0;
}

void OpticalSLAM::trackStereoFrames(const Mat& leftImg, const Mat& rightImg)
{
	// if(nFrames < WINDOW_SIZE)
	// {
		state_estimator.img_manager.trackStereoFrames(leftImg, rightImg);

    if(nFrames >= WINDOW_SIZE)
    {
        m_pose.lock();
        for(int i = 0; i < WINDOW_SIZE; ++i)
        {
            Rp[i] = state_estimator.img_manager.Rp[i];
            tp[i] = state_estimator.img_manager.tp[i];
        }
        trj.push_back(Vector3d(state_estimator.img_manager.tp[WINDOW_SIZE-1][0],
            state_estimator.img_manager.tp[WINDOW_SIZE-1][1],
            state_estimator.img_manager.tp[WINDOW_SIZE-1][2]));
        m_pose.unlock();
        ready_to_display = true;
    }
	//}
    // else if(nFrames == WINDOW_SIZE)
    // {

    //     // for(auto &it:state_estimator.img_manager.landmarks)
    //     // {
    //     //     cout<<it.first<<" "<<it.second->state<<" "<<it.second->depth<<" ";
    //     //     for(auto &ob:it.second->observation)
    //     //     {
    //     //         cout<<ob.first<<ob.second<<" ";
    //     //     }
    //     //     cout<<endl;
    //     // }

        // for(int i = 0; i < WINDOW_SIZE; ++i)
        // {
        //     cout<<state_estimator.img_manager.Rp[i]<<endl;
        //     cout<<state_estimator.img_manager.tp[i].transpose()<<endl<<endl;
        // }

        // ready_to_display = true;
    //}
	m_dis.lock();
	img_to_display = leftImg.clone();
	cvtColor(img_to_display, img_to_display, CV_GRAY2BGR);
	resize(img_to_display, img_to_display, Size(img_to_display.cols/2, img_to_display.rows/2));
	m_dis.unlock();
    ++nFrames;
}

void OpticalSLAM::display()
{
	float fx = 277.34;
    float fy = 291.402;
    float cx = 312.234;
    float cy = 239.777;
    // create pangolin window and plot the trajectory
    pangolin::CreateWindowAndBind("Trajectory Viewer", 1024, 768);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::OpenGlRenderState s_cam(
            pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
            pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0)
            /*pangolin::ModelViewLookAt(0, 10, -10, 0, 0, 0, 0.0, 1.0, 0.0)*/
    );

    pangolin::View &d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
            .SetHandler(new pangolin::Handler3D(s_cam));

   
    while (pangolin::ShouldQuit() == false) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        d_cam.Activate(s_cam);
        glClearColor(0.0f, 0.0f, 0.0f, 0.0f);

        // draw poses
        float sz = 0.3;
        int width = 640, height = 480;
        
        // glLineWidth(2);
        // if(Trajectory2.size() > 1)
        // {
        //     for(int i = 0;i < Trajectory2.size() - 1;i++)
        //     {
        //         glColor3f(0, 1, 0);
        //         glBegin(GL_LINES);
        //         auto p1 = Trajectory2[i], p2 = Trajectory2[i + 1];
        //         glVertex3d(p1[0], p1[1], p1[2]);
        //         glVertex3d(p2[0], p2[1], p2[2]);
        //         glEnd();
        //     }
        // }


        // if(pMap->points.size() > 2)
        // {
        //     glPointSize(1);
        //     glBegin(GL_POINTS);
        //     for (size_t i = 0; i < pMap->points.size(); i++) 
        //     {
        //         glColor3f(0.6, 0.6, 0.6);
        //         glVertex3d(pMap->points[i].x, 
        //             pMap->points[i].y, 
        //             pMap->points[i].z);
        //     }
        //     glEnd();
        // }
        
        // if(laserReg.points.size() > 2)
        // {
        //     glPointSize(1);
        //     glBegin(GL_POINTS);
        //     for (size_t i = 0; i < laserReg.points.size(); i++) 
        //     {
        //         glColor3f(1, 0, 0);
        //         glVertex3d(laserReg.points[i].x, 
        //             laserReg.points[i].y, 
        //             laserReg.points[i].z);
        //     }
        //     glEnd();
        // }

        if(ready_to_display)
        {
            m_pose.lock();
            for(int i = 0; i < WINDOW_SIZE; ++i)
            {
                glPushMatrix();
                Matrix4d current_pos = Matrix4d::Identity();
                current_pos.block<3,3>(0,0) = Rp[i];
                current_pos.block<3,1>(0,3) = tp[i];
                //current_pos = current_pos.inverse();
                glMultMatrixd((GLdouble *) current_pos.data());
                glColor3f(1, i/12.0, 0);
                glLineWidth(2);
                glBegin(GL_LINES);
                glVertex3f(0, 0, 0);
                glVertex3f(sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz);
                glVertex3f(0, 0, 0);
                glVertex3f(sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
                glVertex3f(0, 0, 0);
                glVertex3f(sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
                glVertex3f(0, 0, 0);
                glVertex3f(sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz);
                glVertex3f(sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz);
                glVertex3f(sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
                glVertex3f(sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
                glVertex3f(sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
                glVertex3f(sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
                glVertex3f(sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz);
                glVertex3f(sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz);
                glVertex3f(sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz);
                glEnd();
                glPopMatrix();
            }

            glLineWidth(2);
            if(trj.size() > 1)
            {
                for(int i = 0;i < trj.size() - 1;i++)
                {
                    glColor3f(0, 1, 0);
                    glBegin(GL_LINES);
                    auto p1 = trj[i], p2 = trj[i + 1];
                    glVertex3d(p1[0], p1[1], p1[2]);
                    glVertex3d(p2[0], p2[1], p2[2]);
                    glEnd();
                }
            }
            m_pose.unlock();
        }
        if(!is_first_img)
        {
        	m_dis.lock();
        	imshow("display", img_to_display);
        	m_dis.unlock();
        	waitKey(4);
        }
        pangolin::FinishFrame();
        
        //usleep(5000);   // sleep 5 ms
    }
}
void OpticalSLAM::setFirstFrame(const Mat& leftImg, const Mat& rightImg)
{
	m_dis.lock();
	img_to_display = leftImg.clone();
	cvtColor(img_to_display, img_to_display, CV_GRAY2BGR);
	resize(img_to_display, img_to_display, Size(img_to_display.cols/2, img_to_display.rows/2));
	m_dis.unlock();
	state_estimator.img_manager.setFirstFrame(leftImg, rightImg);
	++nFrames;
}