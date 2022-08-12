#include "msckf.h"
#include "config.h"
#include <Eigen/Dense>


namespace MSCKF_MINE
{


MSCKF::MSCKF()
{
    mpORBextractor = new ORBextractor(orbParam.nFeatures,orbParam.scaleFactor, orbParam.nLevels,orbParam.iniThFAST,orbParam.minThFAST);
    mbReset = true;
}

MSCKF::MSCKF(const VectorXd &state, const MatrixXd &P, const Vector3d &Acc, const Vector3d &Gyro, double &dt)
{
    /*initializing all the variables*/
    /*actually we need to check the num of the variaables*/
    this->mState = state;
    this->mCovariance = P;
    this->mAccPrev = Acc;
    this->mGyroPrev = Gyro;
    this->mdt = dt;

    mGravity = Vector3d(0.0, 0.0, Config::get<double>("g"));

    /*Construct the mpORBextractor*/
    mpORBextractor = new ORBextractor(orbParam.nFeatures,orbParam.scaleFactor, orbParam.nLevels,orbParam.iniThFAST,orbParam.minThFAST);
    mbReset = true;

}

MSCKF::~MSCKF()
{

}


Matrix4d MSCKF::BigOmega( const Vector3d &w)
{
    //constructing big omega matrix
    Matrix4d W= Matrix4d::Zero();
    W.block<3,3>(0,0)  = -1*skewMatrix(w) ;
    W.block<1,3>(3,0)  =  -w.transpose();
    W.block<3,1>(0,3)  =  w;
    return W;

}

Matrix3d MSCKF::skewMatrix( const Vector3d &v)
{

    Matrix3d skewV;
    skewV << 0, -v(2) , v(1),
            v(2), 0 , -v(0),
            -v(1), v(0),  0;

    return skewV;

}


void MSCKF::propagateIMU(Vector3d linear_acceleration, Vector3d angular_velocity)
{
    /*Here we use the formulation in P.48 - P.52*/
    /*c.f. Monocular Visual Inertial Odometry on a Mobile Device*/

//    cout << BOLDBLUE "In MSCKF::propagateIMU function:" << BOLDWHITE << endl;
//    cout << "--acc = \n" << linear_acceleration << "\nw = \n" << angular_velocity << endl;


    Matrix3d d_R, pre_R;
    Vector3d s_hat, y_hat, tmp_vel, tmp_pos;

    /* step1: get the state from the mState.
     * It should be noted that the order of the state
     * quaternion position velocity gyro_bias acc_bias
     *     4         3        3         3        3
     */
    Vector4d spatial_quaternion = mState.segment(0, 4);
    Vector3d spatial_position   = mState.segment(4, 3);
    Vector3d spatial_velocity   = mState.segment(7, 3);
    Vector3d gyro_bias          = mState.segment(10,3);
    Vector3d acce_bias          = mState.segment(13,3);

//    cout << "--spatial_quaternion = \n" << spatial_quaternion << endl;
//    cout << "--spatial_position   = \n" << spatial_position << endl;
//    cout << "--spatial_velocity   = \n" << spatial_velocity << endl;
//    cout << "--gyro_bias          = \n" << gyro_bias        << endl;
//    cout << "--acce_bias          = \n" << acce_bias        << endl;

    Quaterniond spa_q(spatial_quaternion);
//    cout << "--spa_q = \n" << spa_q.coeffs()<< endl;
    Matrix3d spatial_rotation = spa_q.matrix();

    /*step2: get the measurement of actual acc and w*/

    Vector3d curr_w = angular_velocity - gyro_bias;
    Vector3d curr_a = linear_acceleration - acce_bias;

//    cout << "--curr_w          = \n" << curr_w        << endl;
//    cout << "--curr_a          = \n" << curr_a        << endl;

    /* step3: caculate the qB(l+1)B(l) or RB(l+1)B(l) which represents the rotation from Bl to B(l+1)
     * method: the fourth order Runge-Kutta
     */
    d_R = calcDeltaQuaternion(mGyroPrev, curr_w, mdt).matrix();

//    cout << "--d_R          = \n" << d_R        << endl;
//    cout << "--mdt          = \n" << mdt        << endl;


    /* step4: calculate s_hat and y_hat cf. P49*/

    s_hat = 0.5 * mdt * (d_R.transpose() * curr_a + mAccPrev);
    y_hat = 0.5 * mdt * s_hat;

    /* step5: update q v p*/
    pre_R = spatial_rotation;
    spatial_rotation = spatial_rotation * d_R.transpose(); /*from B(l+1) to G*/
    spatial_quaternion = Quaterniond(spatial_rotation).coeffs() /*w x y z*/;

    tmp_vel = spatial_velocity + spatial_rotation * s_hat + mGravity * mdt;
    tmp_pos = spatial_position + spatial_velocity * mdt + spatial_rotation * y_hat + 0.5 * mGravity * mdt * mdt;

    spatial_velocity = tmp_vel;
    spatial_position = tmp_pos;

    mState.segment(0,4) = spatial_quaternion;
    mState.segment(4,3) = spatial_position;
    mState.segment(7,3) = spatial_velocity;

    /*step6: update the mAccPrev and mGyroPrev*/
    this->mAccPrev  = curr_a;
    this->mGyroPrev = curr_w;

    /*step6: covariance P.50 & P.88*/
    /* |theta(l+1) |   |I3      03  03      phi_qbg  phi_qba|
     * |  pos(l+1) |   |phi_pq  I3  I3*mdt  phi_pbg  phi_pba|
     * |  vel(l+1) | = |phi_vq  03  I3      phi_vbg  phi_vba| + Q(noise)
     * |   bg(l+1) |   |03      03  03      I3       03     |
     * |   ba(l+1) |   |03      03  03      03       I3     |
     *
     * for details please cf.P88
     */

    MatrixXd phi = MatrixXd::Identity(15,15);
    phi.block<3,3>(3,0) = -1.0 * skewMatrix(pre_R * y_hat);  /*phi_pq*/
    phi.block<3,3>(6,0) = -1.0 * skewMatrix(pre_R * s_hat);  /*phi_vq*/

    phi.block<3,3>(3,6) = Matrix3d::Identity() * mdt;

    phi.block<3,3>(0,9) = -0.5 * mdt * (pre_R + spatial_rotation); /*phi_qbg*/
    phi.block<3,3>(6,9) = 0.25 * mdt * mdt * (skewMatrix(spatial_rotation * curr_a) * (pre_R + spatial_rotation));/*phi_qvbg*/
    phi.block<3,3>(3,9) = 0.5 * mdt * phi.block<3,3>(6,9);/*phi_qpbg*/

    phi.block<3,3>(0,12) = 0.5 * mdt * (pre_R + spatial_rotation);
    Matrix3d phi_vba = -0.5 * mdt * (pre_R + spatial_rotation) + 0.25 * mdt * mdt * (skewMatrix(spatial_rotation * curr_a) * (pre_R + spatial_rotation));
    phi.block<3,3>(6,12) = phi_vba;
    phi.block<3,3>(3,12) = 0.5 * mdt * phi_vba;

    Matrix<double, 15, 15> Nc;
    Matrix<double, 15, 1> Nc_diag;
    Nc_diag <<
               mIMUParams.sigma_gc*mIMUParams.sigma_gc * Vector3d(1, 1, 1),Vector3d(0, 0, 0),
            mIMUParams.sigma_ac*mIMUParams.sigma_ac * Vector3d(1, 1, 1),
            mIMUParams.sigma_wgc*mIMUParams.sigma_wgc * Vector3d(1, 1, 1),
            mIMUParams.sigma_wac*mIMUParams.sigma_wac * Vector3d(1, 1, 1);

    Nc = Nc_diag.asDiagonal();

    MatrixXd Qd = 0.5 * mdt * phi * Nc * phi.transpose() + Nc;
    mCovariance.block<15,15>(0,0) = phi * mCovariance.block<15,15>(0,0) * phi.transpose() + Qd;

    /* step7: update the PIC and PCI
     * may be some error!
     * we know that the covariance is structed as |PII PIC|
     *                                            |PCI PCC|
     * I -> Imu
     * C -> Camera
     */
    mCovariance.block(0, 15, 15, mCovariance.cols() - 15) = phi * mCovariance.block(0, 15, 15, mCovariance.cols() - 15);
    mCovariance.block(15, 0, mCovariance.rows() - 15, 15) = mCovariance.block(0, 15, 15, mCovariance.cols() - 15).transpose();

}

Matrix4d MSCKF::calcOmegaMatrix(const Vector3d &w)
{
    Matrix4d W= Matrix4d::Zero();
    W.block<3,3>(0,0)  = -1*skewMatrix(w) ;
    W.block<1,3>(3,0)  =  -w.transpose();
    W.block<3,1>(0,3)  =  w;
    return W;
}



Quaterniond MSCKF::calcDeltaQuaternion(const Vector3d &mGyroPrev, const Vector3d curr_w, double &dt)
{
//    cout << "--imput values \n";


    Vector4d q0;
    q0 << 0.0, 0.0, 0.0, 1.0;
    Vector4d k1, k2, k3, k4, d_q;


    k1 = 0.5 * calcOmegaMatrix(mGyroPrev) * q0;
    k2 = 0.5 * calcOmegaMatrix((mGyroPrev + curr_w) / 2) * (q0 + 0.5 * k1 * dt);
    k3 = 0.5 * calcOmegaMatrix((mGyroPrev + curr_w) / 2) * (q0 + 0.5 * k2 * dt);
    k4 = 0.5 * calcOmegaMatrix(curr_w) * (q0 + k3 * dt);

//    cout << "k1,k2,k3,k4" << k1 << endl << endl << k2 << "\n\n" << k3 << "\n\n" << k4 << endl;

    d_q = q0 + ( (k1 + 2*k2 + 2*k3 + k4) * dt ) / 6.0;

    d_q = d_q / d_q.norm();

    return Quaterniond(d_q);

}

void MSCKF::Augmentation()
{
    // when an image is captured, the current body quaternion, position and velocity are added to the
    // state vector and the covariance is augmented accordingly.

    /*step1: augmente the state vector*/



    int stateSize = mState.rows();
    mState.conservativeResize(stateSize + 10);

    mState.segment<10>(stateSize) = mState.head(10);



    /*step2: augmente the covariance
     * It should be noted that in step 1 we have add an image
     */
    int N = (mState.size() - 16) / 10; // N = number of image
    cout << "in Augmentation: " << " N = " << N << endl;

    MatrixXd Jpi = MatrixXd::Zero(9, mState.size()-(N+1));
    MatrixXd I9 = MatrixXd::Identity(9,9);
    Jpi.block<9,9>(0,0) = I9;

    int covSize = mCovariance.rows();
    mCovariance.conservativeResize(covSize+9, covSize+9); /*since we have add an image, so the covariance need to be augmented*/

    mCovariance.block(0, covSize,covSize,9) = mCovariance.block(0,0,covSize,covSize)*Jpi.transpose();

    mCovariance.block(covSize,0,9,covSize) = mCovariance.block(0, covSize,covSize,9).transpose();
    mCovariance.block(covSize,covSize,9,9) = Jpi*mCovariance.block(0, covSize,covSize,9);

}

void MSCKF::Marginalizefilter()
{
    /*step 1: marginal the state vector*/
    mState.conservativeResize(16,1);

    /*step 2: marginal the cavariance*/
    mCovariance.conservativeResize(15,15);
}


/*ORB_parts*/


void MSCKF::imageComing(const Mat &image, const double timestamp)
{
    mImage = image.clone();
    mTimeStamp = timestamp;
}




void MSCKF::unDistortImage()
{
    cv::Mat tmp = mImage.clone();
    cv::undistort(tmp, mImage, mCAMParams.getK(), mCAMParams.getD(), cv::Mat());
    //    cv::imshow("undistort", mImage);
    //    cv::waitKey(0);

}

void MSCKF::extractFeatures()
{
    /*step1: --> undistort the image*/
    unDistortImage();
    /*step2: --> extract the features*/
    ORBextractor *pOrbExtractor = new ORBextractor(orbParam.nFeatures, orbParam.scaleFactor, orbParam.nLevels,
                                                   orbParam.iniThFAST, orbParam.minThFAST);
    (*pOrbExtractor)(mImage,cv::Mat(),mvKeys,mDescriptors);

}


void MSCKF::ConstructFrame(const Mat &im, const double &timeStamp)
{

    frame = Frame(im, timeStamp, mpORBextractor);
    mImage = im.clone();
}

void MSCKF::ConstructFrame(bool reset)
{
    if(!reset)
    {
        mpORBextractor->setFeatureNum(Config::get<int>("ORBextractor.nFeaturesInit"));
        frame = Frame(mImage, mTimeStamp, mpORBextractor);
    }

    else
    {
        mpORBextractor->setFeatureNum(Config::get<int>("ORBextractor.nFeatures"));
        frame = Frame(mImage, mTimeStamp, mpORBextractor);
    }
}


void MSCKF::RunFeatureMatching()
{

    /*step 1: Construct the frame*/
    this->ConstructFrame(mbReset);

    if(mvFeatures.size() < 50 && mvFeatures.size() > 0)
    {
        /*it means that the number of tracked feature is too small */
        mvFeatures.clear();
        mvFeaturesIdx.clear();
        Marginalizefilter();
        mbReset = true;         /*which means we need to rest the filter*/
    }

    mvLostFeatures.clear();
    mvLostFeatureCamIdx.clear();




    /*we need to know if the status is true or not */
    if(!mbReset)
    {
        /*in this case, the last frame is not empty so we can run the feature matching*/
        /*the most inportment part is frame.matchesID
         */

        ORBmatcher orbMatcher(0.7);
        orbMatcher.MatcheTwoFrames(frame, feedframe, false); /*it is important to use the pior to make the robust match*/

        cv::Mat imMatch = orbMatcher.DrawFrameMatch(frame,feedframe);
        cv::imshow("matches", imMatch);
        cv::waitKey(0);


        ManageOldFeatures();

    }
    else
    {
        /* since the mbReset is true, there are two cases
         * case1: the first frame
         * case2: the mvFeatures.size() < 50 which means the tracked features is two small
         */
        feedframe = Frame(frame);

        /*step 2: AugmentNewFeatures*/

        AugmentNewFeatures();
        mbReset = false;
    }
}

/**
 * @brief MSCKF::AugmentNewFeatures-> used in RunFeatureMatching();
 */
void MSCKF::AugmentNewFeatures()
{
    mvFeatures    = vector<MatrixXd>(frame.mvKeys.size(), MatrixXd::Zero(2,1));
    mvFeaturesIdx = vector<Vector2i>(frame.mvKeys.size(), Vector2i::Zero());

    /*since the filter is reseted, we need add all the features to the feature manager vector*/

    for(int i = 0; i < frame.mvKeys.size(); i++)
    {
        mvFeatures[i](0,0) = frame.mvKeys[i].pt.x;
        mvFeatures[i](1,0) = frame.mvKeys[i].pt.y;
        mvFeaturesIdx[i]   = Vector2i(i, int(frame.mnId));
    }
}

void MSCKF::ManageOldFeatures()
{
    map<int,int> matches = Converter::swapMatchesId(frame.matchesId);
    map<int,int>::iterator matches_iter;

    for(int j = 0; j < mvFeaturesIdx.size(); j++)
    {
        int feedId = mvFeaturesIdx[j](0); /*key*/
        matches_iter = matches.find(feedId);
        if(matches_iter!=matches.end())
        {
            /*feature were tracked!*/
            int M = mvFeatures[j].rows();
            int N = mvFeatures[j].cols();
            mvFeatures[j].conservativeResize(M,N+1);
            mvFeatures[j](0,N) = frame.mvKeys[matches_iter->second].pt.x;
            mvFeatures[j](1,N) = frame.mvKeys[matches_iter->second].pt.y;

        }
        else
        {
            /*feature were not tracked, we need remove it and augment the lostfeatures*/
            mvLostFeatures.push_back(mvFeatures[j]);

            mvLostFeatureCamIdx.push_back(mvFeaturesIdx[j](1));

            mvFeatures.erase(mvFeatures.begin() + j);
            mvFeaturesIdx.erase(mvFeaturesIdx.begin() + j);

        }


    }

    /*show the result*/



}


Vector3d MSCKF::TriangulationWorldPoint(vector<Vector2d> &z, VectorOfPose &poses)
{
    ceres::Problem *problem;
    /*build the optimization problem*/
    ceres::LossFunction* loss_function = new ceres::HuberLoss(1.0);
    ceres::LocalParameterization* quaternion_local_parameterization =
            new ceres::EigenQuaternionParameterization;
    unsigned int i = 0;

    Vector3d point3d(1.0,1.0,1.0); /*the initial guess*/
    for(vector<Vector2d>::iterator z_iter = z.begin(); z_iter!=z.end(); z_iter++)
    {
        Vector2d &z = *z_iter;
        ceres::CostFunction *cost_function = TriangulationReprojectionErr::Create(z);

        problem->AddResidualBlock(cost_function, loss_function,
                                  poses[i].q.coeffs().data(),
                                  poses[i].t.data(),
                                  point3d.data());
        problem->SetParameterization(poses[i].q.coeffs().data(),
                                     quaternion_local_parameterization);

        i++;
    }

    /*since we only optimize the 3d point in the word, so we need to set the q and t constant*/
    for(VectorOfPose::iterator pose_iter = poses.begin(); pose_iter!=poses.end(); pose_iter++)
    {
        problem->SetParameterBlockConstant(pose_iter->q.coeffs().data());
        problem->SetParameterBlockConstant(pose_iter->t.data());
    }

    /*optimize the point3d*/
    ceres::Solver::Options options;
    options.max_num_iterations = 100;
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;

    ceres::Solver::Summary summary;
    ceres::Solve(options, problem, &summary);

    //std::cout << BOLDCYAN << summary.FullReport() << '\n';
    return point3d;
}


void MSCKF::CalcResidualsAndStackingIt()
{
    /*TEST AND DEBUG*/

    cout << "mvFeatures.size() = " << mvFeatures.size() << endl;
    cout << "mvLostFeatures.size() = " << mvLostFeatures.size() << endl;



    /*Ho and r is used for filter update step*/

    vector<MatrixXd> vH;
    vector<MatrixXd> vr;

    /*Ho and r is used for filter update step*/

    for(int i = 0; i < mvLostFeatures.size(); i++)
    {
        /*the ith lost feature*/

        vector<Vector2d> z_observation;
        VectorOfPose poses;



        /*num -> the num of observation of ith feature*/
        int num = mvLostFeatures[i].cols();
        for(int j = 0; j < num; j++)
        {
            /*this part we featch the 2d observation and the transition matrix
             * z_observation -> the ith feature's 2d measurement
             * poses -> Tcw which means world to camera
             * aim-> optimize the world pose of ith feature
             */

            z_observation.push_back(Vector2d(mvLostFeatures[i].col(j)));
            /*since the triangulation need the Tcw*/
            /*but we only get the state from the mstate*/
            Pose pose;
            Quaterniond qwb = Quaterniond( mState.segment<4>(16 + 10*j));
            Matrix3d Rwb = qwb.toRotationMatrix();
            Vector3d twb = mState.segment<3>(16 + 10*j + 4);
            Matrix4d Twb = Matrix4d::Identity();
            Twb.block<3,3>(0,0) = Rwb;
            Twb.block<3,1>(0,3) = twb;
            Matrix4d Twc = Twb * Converter::toMatrix4d(mCAMParams.getTBS());
            Matrix4d Tcw = Twc.inverse();
            pose.q = Quaterniond(Tcw.block<3,3>(0,0));
            pose.t = Tcw.block<3,1>(0,3);
            poses.push_back(pose);

        }

        cout << "data for Triangulation:" << endl;
        cout << "poses.size() = " << poses.size() << endl << "z_observation.size() = " << z_observation.size() << endl;


        /*Triangulation Part*/
        Vector3d point_i_3d = TriangulationWorldPoint(z_observation, poses);

        cin.get();



        /*cf.53 6.5.2 Error Representation Monocular Visual Inertial Odometry on a Mobile Device*/

        Matrix<double, 2,9> Hbij;
        MatrixXd Hxij = MatrixXd::Zero(2, 15+9*num);
        Matrix<double, 2,3> Hfij;
        Vector2d rij;
        MatrixXd Hxi = MatrixXd::Zero(2*num, 15+9*num);
        MatrixXd Hfi = MatrixXd::Zero(2*num, 3);
        VectorXd ri;
        MatrixXd roi;
        MatrixXd Hoi;

        for(int j = 0; j < num; j++)
        {
            /*compute the Hxi and Hfi
             * Hxi = Hzjfi * Hfix
             * Hzjfi is simple to calc since we konw pc = Tcw * pw
             */

            Matrix4d Tcw = Matrix4d::Identity();
            Tcw.block<3,3>(0,0) = poses[j].q.toRotationMatrix();
            Tcw.block<3,1>(0,3) = poses[j].t;

            /*step2: calcHx and Hf
             * Hbij -->2*9  -->Hxij
             * Hfij -->2*3  -->Hfi
             */
            Vector2d zij = z_observation[j];
            CalcHxAndHf(Tcw, point_i_3d, Hbij, Hfij, zij, rij);

            Hxij.block<2,9>(0, 15+9*j) = Hbij;
            Hxi.block(2*j,0, 2,Hxi.cols()) = Hxij; /*very important*/
            Hfi.block<2,3>(2*j,0) = Hfij;
            ri.segment<2>(2*j) = rij;




        }

        /*step3: nullspace to calc the H*/
        MatrixXd Ai;
        nullSpace(Hfi, Ai);
        roi = Ai.transpose() * ri;
        Hoi = Ai.transpose() * Hxi;

        /* step4: Outiler Detection
         * method: Chi-square test
         */
        if (ChiSquareTest(Hoi,roi,mCovariance))
        {
            /*inlier*/
            /*keep the roi and Hoi into the vector*/
            vH.push_back(Hoi);
            vr.push_back(roi);
        }



        /*after triangulation we need clear the buff*/
        z_observation.clear();
        poses.clear();
    }

    /*after the lost features loop*/
    /*update step is needed*/

    MsckfUpdate(vH,vr);



}

void MSCKF::CalcHxAndHf(Matrix4d &Tcw, Vector3d &pw, Matrix<double, 2,9> &Hbi,  Matrix<double, 2,3> &Hfi, Vector2d zij, Vector2d rij)
{
    /*step1: translate the point_i_3d to the current j frame cj_p_fi*/
    Matrix3d Rcw = Tcw.block<3,3>(0,0);
    Vector3d tcw = Tcw.block<3,1>(0,3);
    Vector3d pc = Rcw * pw + tcw;



    Matrix<double,2,3> Jh = Matrix<double,2,3>::Zero();
    double fx = mCAMParams.fx;
    double fy = mCAMParams.fy;
    double cx = mCAMParams.cx;
    double cy = mCAMParams.cy;

    Vector2d z_hat = Vector2d(cx + fx * pc(0) / pc(2), cy + fy * pc(1) / pc(2));

    Jh << fx/pc(2),        0,   (-1.0 * fx * pc(0))/(pc(2) * pc(2)),
            0     , fy/pc(1),   (-1.0 * fy * pc(1))/(pc(2) * pc(2));
    Matrix4d Tbc = Converter::toMatrix4d(mCAMParams.getTBS());
    Matrix4d Tbw = Tbc * Tcw;
    Matrix4d Twb = Tbw.inverse();
    Vector3d twb = Twb.block<3,1>(0,3);
    Matrix3d Rbw = Tbw.block<3,3>(0,0);
    Matrix3d Rcb = Tbc.block<3,3>(0,0).transpose();

    /*step2 -> calc the Hfi*/
    Hfi = Jh * Rcb * Rbw;    /* 2*3 */

    /*step1 -> calc the Hxi*/
    Matrix3d I3 = Matrix3d::Identity();
    Matrix3d Z3 = Matrix3d::Zero();

    Matrix<double,3,9> Hfb;
    Hfb.block<3,3>(0,0) = skewMatrix(pw - twb);
    Hfb.block<3,3>(0,3) = -I3;
    Hfb.block<3,3>(0,6) = Z3;
    Hbi = Hfi * Hfb; /* 2 * 9 */

    /*step3 -> calc ri*/
    rij = zij - z_hat;
}


void MSCKF::nullSpace(MatrixXd &H, MatrixXd &A)
{
    int n = H.rows() /2;

    JacobiSVD<MatrixXd> svd(H, ComputeFullU);
    A = svd.matrixU().rightCols(2*n-3).transpose();
}

bool MSCKF::ChiSquareTest(MatrixXd &Hoi, MatrixXd &roi, MatrixXd &covariance)
{
    int k = roi.rows(); /* k/2 ? */
    MatrixXd gamma = roi * (Hoi * covariance * Hoi.transpose()).inverse() * roi.transpose();
    if(gamma(0,0) < chi2Inv(k) ) /*gamma(0)*/
        return true;
    else
        return false;
}

void MSCKF::MsckfUpdate(vector<MatrixXd> &vH, vector<MatrixXd> &vr)
{
    /*stack H and r*/
    int vSize = vH.size();
    int rowH,rowr,colH,colr;
    rowH = rowr = colH = colr = 0;

    colH = vH[0].cols();
    colr = vr[0].cols();
    vector<int> vRowH;
    vector<int> vRowr;


    for(int i = 0; i < vSize; i++)
    {
        rowH += vH[i].rows();
        rowr += vr[i].rows();
        vRowH.push_back(rowH);
        vRowr.push_back(rowr);
    }

    MatrixXd H = MatrixXd::Zero(rowH, colH);
    MatrixXd r = MatrixXd::Zero(rowr, colr);

    int tmpRowH = 0;
    int tmpRowr = 0;

    for(int i = 0; i < vSize; i++)
    {
        H.block(tmpRowH, 0, vRowH[i], colH) = vH[i];
        r.block(tmpRowr, 0, vRowr[i], colr) = vr[i];
        tmpRowH += vH[i].rows();
        tmpRowr += vH[i].rows();
    }


    /*for now we have get H and r
     * r = Hx + n
     */

    /*we need QR decomposition*/
    MatrixXd rq, TH;
    QRdecomposition(H, r, rq, TH);

    /*update step*/
    Update(H,rq,TH);


}


void MSCKF::QRdecomposition(MatrixXd H, MatrixXd r, MatrixXd &rq, MatrixXd &TH)
{
    /*we need to QR decomposition*/
    HouseholderQR<MatrixXd> qr(H);
    MatrixXd Q1;
    Q1 = qr.householderQ() * (MatrixXd::Identity(H.rows(),H.cols()));
    TH = qr.matrixQR().block(0,0,H.cols(),H.cols()).triangularView<Upper>();

    rq = Q1.transpose() * r;
    /*it seems that we need generate some new results*/

}

void MSCKF::Update(MatrixXd &H, MatrixXd &rq, MatrixXd &TH)
{
    int q = H.rows();
    MatrixXd Rq = MatrixXd::Identity(q,q);
    Rq = mCAMParams.sigma_img * mCAMParams.sigma_img * Rq;
    MatrixXd I_belta = MatrixXd::Identity(q,q);

    /*before update we need a check for row and col to ensure the mutiply*/
    MatrixXd K = mCovariance * TH.transpose() *(TH * mCovariance * TH.transpose()).inverse();
    mCovariance = (I_belta - K * TH) * mCovariance * (I_belta - K * TH).transpose() + K * Rq * K.transpose();
    MatrixXd delta = K * rq;
    VectorXd delta_x(delta);

    /*update*/
    Quaterniond Q(mState.segment<4>(0));
    Quaterniond dq(1, 0.5*delta_x(0), 0.5*delta_x(1), 0.5*delta_x(2));
    mState.segment<4>(0) = (Q*dq).normalized().coeffs();
    mState.segment<3>(4) += delta_x.segment<3>(3);
    mState.segment<3>(7) += delta_x.segment<3>(6);
    mState.segment<3>(10) += delta_x.segment<3>(9);
    mState.segment<3>(13) += delta_x.segment<3>(12);

    /*for cameras*/
    for(int i = 16; i < mState.size(); i+=10)
    {
        Quaterniond Q(mState.segment<4>(i));
        Quaterniond dq(1,0.5*delta_x(i-1), 0.5*delta_x(i-1), 0.5*delta_x(i-1));
        mState.segment<4>(i) = (Q * dq).normalized().coeffs();
        mState.segment<3>(i+4) += delta_x.segment<3>(i+3);
        mState.segment<3>(i+7) += delta_x.segment<3>(i+6);
    }


}



}
