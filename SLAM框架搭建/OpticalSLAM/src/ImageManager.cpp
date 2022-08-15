#include "ImageManager.h"

void removeOutliers(vector<cv::Point2f> &v, vector<uchar>& status)
{
    int j = 0;
    for (int i = 0; i < int(v.size()); i++)
        if (status[i])
            v[j++] = v[i];
    v.resize(j);
}

void removeOutliers(vector<long int> &v, vector<uchar>& status)
{
    int j = 0;
    for (int i = 0; i < int(v.size()); i++)
        if (status[i])
            v[j++] = v[i];
    v.resize(j);
}

Point2f pixel2cam(Point2f p, Vec4d intrinsics)
{
	float _fx = intrinsics[0];
	float _cx = intrinsics[1];
	float _fy = intrinsics[2];
	float _cy = intrinsics[3];

	return Point2f((p.x-_cx)/_fx,(p.y-_cy)/_fy);
}

ImageManager::ImageManager()
{
	id_cnt = 1;
	frame_cnt = 0;
	is_first_track = true;

	cam_intrinsics[0] = fx;
	cam_intrinsics[1] = cx;
	cam_intrinsics[2] = fy;
	cam_intrinsics[3] = cy;

	frameID0 = 0;
	Rp.resize(WINDOW_SIZE);
	tp.resize(WINDOW_SIZE);
	//frameIDmid = WINDOW_SIZE/2;
}

ImageManager::~ImageManager()
{

}

void ImageManager::setFirstFrame(const Mat& leftImg, const Mat& rightImg)
{
	cur_left_img = leftImg;
	cur_right_img = rightImg;

	createImagePyramids();

	//vector<Point2f> ptsl, ptsr;
	vector<uchar> status;
    vector<float> err;

    goodFeaturesToTrack(leftImg, prev_left_pts, MAX_CNT, 0.01, 30);

    //stereoMatch(ptsl, ptsr, status);

    prev_left_img = cur_left_img;
    prev_right_img = cur_right_img;

    swap(cur_left_pyramid_, prev_left_pyramid_);
    swap(cur_right_pyramid_, prev_right_pyramid_);

    for(int i = 0; i < prev_left_pts.size(); ++i)
    	ids.push_back(-1);

    frame_cnt++;
    cur_left_pyramid_.clear();
    cur_right_pyramid_.clear();
}

void ImageManager::stereoMatch(std::vector<cv::Point2f>& left_points,
      std::vector<cv::Point2f>& right_points,
      std::vector<unsigned char>& inlier_markers)
{
	if (left_points.size() == 0) return;

	calcOpticalFlowPyrLK(cur_left_pyramid_, cur_right_pyramid_,
      left_points, right_points,
      inlier_markers, noArray(),
      Size(21, 21),
      3);

	for (int i = 0; i < right_points.size(); ++i) 
	{
	    if (inlier_markers[i] == 0) continue;
	    if (right_points[i].y < 0 ||
	        right_points[i].y > cur_right_img.rows-1 ||
	        right_points[i].x < 0 ||
	        right_points[i].x > cur_right_img.cols-1)
	      inlier_markers[i] = 0;
 	}

 	removeOutliers(left_points, inlier_markers);
 	removeOutliers(right_points, inlier_markers);

 	vector<uchar> status2;
	findFundamentalMat(left_points, right_points, cv::FM_RANSAC, 1.0, 0.99, status2);

	removeOutliers(left_points, status2);
 	removeOutliers(right_points, status2);
}
void ImageManager::checkEpipolarConstraint(const vector<Point2f>& pts0, const vector<Point2f>& pts1, vector<uchar>& status, double threshold)
{
	status.resize(pts0.size());
	// for(int i = 0; i < pts0.size(); ++i)
	// {
	// 	status[i] = 1;
	// }

	Mat t_x = (Mat_<double>(3,3)<<0, 0, 0,
								0, 0, 1,
								0, -1, 0);
	Point2f p1,p2;
	Vec4d intrin(fx, cx, fy, cy);
	for(int i = 0; i < pts0.size(); ++i)
	{
		p1 = pixel2cam(pts0[i], intrin);
		p2 = pixel2cam(pts1[i], intrin);

		Mat y1 = (Mat_<double>(3,1)<<p1.x, p1.y, 1);
		Mat y2 = (Mat_<double>(3,1)<<p2.x, p2.y, 1);

		Mat d = y2.t()*t_x*y1;
		double dd = d.at<double>(0,0);

		if(std::abs(dd) > threshold)
		{
			status[i] = 0;
		}
		else
		{
			status[i] = 1;
		}
		// int dddd = status[i];
		// cout<<dd<<" "<<dddd<<endl;
	}
	
}
void ImageManager::checkShiftsOnY(const vector<Point2f>& pts0, const vector<Point2f>& pts1, vector<uchar>& status, double threshold)
{
	status.resize(pts0.size());
	
	for(int i = 0; i < pts0.size(); ++i)
	{
		
		double shift = pts0[i].y - pts1[i].y;
		if(std::abs(shift) > threshold)
		{
			status[i] = 0;
		}
		else
		{
			status[i] = 1;
		}
	}
}
void ImageManager::createImagePyramids()
{
  buildOpticalFlowPyramid(
      cur_left_img, cur_left_pyramid_,
      Size(21, 21),
      3, true, BORDER_REFLECT_101,
      BORDER_CONSTANT, false);//构建3层图像金字塔

  buildOpticalFlowPyramid(
      cur_right_img, cur_right_pyramid_,
      Size(21, 21),
      3, true, BORDER_REFLECT_101,
      BORDER_CONSTANT, false);
}
void ImageManager::setMask()
{
	mask = Mat(cur_left_img.rows, cur_left_img.cols, CV_8UC1, Scalar(255));
	for(int i = 0; i < cur_left_pts.size(); ++i)
	{
		if(mask.at<uchar>(cur_left_pts[i].y, cur_left_pts[i].x) == 255)
		{
			circle(mask, cur_left_pts[i], MIN_DIST, 0, -1);
		}
	}
}
void ImageManager::trackStereoFrames(const Mat& leftImg, const Mat& rightImg)
{
	cur_left_img = leftImg;
	cur_right_img = rightImg;

	createImagePyramids();

	vector<unsigned char> status1;
	calcOpticalFlowPyrLK(prev_left_pyramid_, cur_left_pyramid_,
      prev_left_pts, cur_left_pts,
      status1, noArray(),
      Size(21, 21),
      3);

	for (int i = 0; i < cur_left_pts.size(); ++i) 
	{
	    if (status1[i] == 0) continue;
	    if (cur_left_pts[i].y < 0 ||
	        cur_left_pts[i].y > cur_left_img.rows-1 ||
	        cur_left_pts[i].x < 0 ||
	        cur_left_pts[i].x > cur_left_img.cols-1)
	      status1[i] = 0;
 	}

 	removeOutliers(cur_left_pts, status1);
 	removeOutliers(prev_left_pts, status1);
 	removeOutliers(ids, status1);

 	vector<uchar> status2;
	findFundamentalMat(prev_left_pts, cur_left_pts, cv::FM_RANSAC, 1.0, 0.99, status2);
	removeOutliers(cur_left_pts, status2);
 	removeOutliers(prev_left_pts, status2);
 	removeOutliers(ids, status2);
 	pre.resize(prev_left_pts.size());
 	cur.resize(cur_left_pts.size());

 	for(int i = 0; i < prev_left_pts.size(); ++i)
 	{
 		pre[i] = pixel2cam(prev_left_pts[i], cam_intrinsics);
 		cur[i] = pixel2cam(cur_left_pts[i], cam_intrinsics);
 	}
 	//cout<<averageParallax(prev_left_pts, cur_left_pts, status2)<<endl;
 	if(averageParallax(prev_left_pts, cur_left_pts, status2) < MIN_PARALLAX)
	{
		cur_right_pyramid_.clear();
	    cur_left_pyramid_.clear();
		cur_left_pts.clear();
		return;
	}

 	//vector<Point2f> prevPtsRightS;
 	vector<unsigned char> status3;
 	vector<unsigned char> status4;
 	if(is_first_track)
 	{
 		is_first_track = false;

 		calcOpticalFlowPyrLK(prev_left_pyramid_, prev_right_pyramid_,
	      prev_left_pts, prev_right_pts,
	      status3, noArray(),
	      Size(21, 21),
	      3);

 		for (int i = 0; i < prev_right_pts.size(); ++i) 
		{
		    if (status3[i] == 0) continue;
		    if (prev_right_pts[i].y < 0 ||
		        prev_right_pts[i].y > prev_right_img.rows-1 ||
		        prev_right_pts[i].x < 0 ||
		        prev_right_pts[i].x > prev_right_img.cols-1)
		      status3[i] = 0;
		}

		removeOutliers(cur_left_pts, status3);
	 	removeOutliers(prev_left_pts, status3);
	 	removeOutliers(prev_right_pts, status3);
	 	removeOutliers(ids, status3);

	 	//findFundamentalMat(prev_left_pts, prev_right_pts, cv::FM_RANSAC, 1.0, 0.99, status4);
	 	//checkEpipolarConstraint(prev_left_pts, prev_right_pts, status4, 0.003);
	 	checkShiftsOnY(prev_left_pts, prev_right_pts, status4, ShiftOnYTHS);

		removeOutliers(prev_left_pts, status4);
	 	removeOutliers(prev_right_pts, status4);
	 	removeOutliers(cur_left_pts, status4);
	 	removeOutliers(ids, status4);

	 	// Mat debug = prev_left_img.clone();
	 	// cvtColor(debug, debug, CV_GRAY2BGR);
	 	// for(int i = 0; i < prev_left_pts.size(); ++i)
	 	// {
	 	// 	circle(debug, prev_left_pts[i], 2, Scalar(0,0,255), 2);
	 	// 	line(debug, prev_left_pts[i], prev_right_pts[i], Scalar(0,255,255));
	 	// }
	 	// stringstream sstr;
	 	// sstr<<frame_cnt<<"stereo";
	 	// string filename = "../debug/"+sstr.str()+".png";
	 	// imwrite(filename, debug);

	 	for(int i = 0; i < ids.size(); ++i)
	 	{
	 		if(ids[i] < 0)
	 			ids[i] = id_cnt++;
	 	}

	 	for(int i = 0; i < ids.size(); ++i)
	 	{
	 		if(landmarks.count(ids[i])>0)
	 		{
		 		pair<int,Point2f> p;
	 			p.first = frame_cnt;
	 			p.second = pixel2cam(cur_left_pts[i], cam_intrinsics);
	 			landmarks[ids[i]]->observation.emplace_back(std::move(p));
	 		}
	 		else
	 		{
	 			double disp = prev_left_pts[i].x - prev_right_pts[i].x;
	 			double depth = BASELINE*fx/disp;
	 			// if(depth > 2 && depth < 40)
	 			// {
	 				landmarks[ids[i]] = shared_ptr<LandMark>(new LandMark);
		 			pair<int,Point2f> p0, p1;
		 			p0.first = frame_cnt - 1;
		 			p0.second = pixel2cam(prev_left_pts[i], cam_intrinsics);

		 			p1.first = frame_cnt;
		 			p1.second = pixel2cam(cur_left_pts[i], cam_intrinsics);

		 			landmarks[ids[i]]->observation.emplace_back(std::move(p0));
		 			landmarks[ids[i]]->observation.emplace_back(std::move(p1));

		 			landmarks[ids[i]]->depth = depth;
	 		}
	 	}
 	}
 	else if(frame_cnt < WINDOW_SIZE)
 	{
 		int ii = -1;
	 	for(int i = ids.size() - 1; i >= 0; --i)
	 	{
	 		if(ids[i] >= 0)
	 		{
	 			ii = i+1;
	 			break;
	 		}
	 	}
	 	if(ii != ids.size())
	 	{
	 		vector<Point2f> newFeatPre, newFeatCur;
	 		for(int i = ii; i < ids.size(); ++i)
	 		{
	 			newFeatPre.emplace_back(prev_left_pts[i]);
	 			newFeatCur.emplace_back(cur_left_pts[i]);
	 		}

	 		calcOpticalFlowPyrLK(prev_left_pyramid_, prev_right_pyramid_,
		      newFeatPre, prev_right_pts,
		      status3, noArray(),
		      Size(21, 21),
		      3);

	 		for (int i = 0; i < prev_right_pts.size(); ++i) 
			{
			    if (status3[i] == 0) continue;
			    if (prev_right_pts[i].y < 0 ||
			        prev_right_pts[i].y > prev_right_img.rows-1 ||
			        prev_right_pts[i].x < 0 ||
			        prev_right_pts[i].x > prev_right_img.cols-1)
			      status3[i] = 0;
			}

			removeOutliers(newFeatCur, status3);
		 	removeOutliers(newFeatPre, status3);
		 	removeOutliers(prev_right_pts, status3);
		 	//removeOutliers(ids, status3);

		 	//findFundamentalMat(newFeatPre, prev_right_pts, cv::FM_RANSAC, 1.0, 0.99, status4);
		 	//checkEpipolarConstraint(newFeatPre, prev_right_pts, status4, 0.003);
		 	checkShiftsOnY(newFeatPre, prev_right_pts, status4, ShiftOnYTHS);
			removeOutliers(newFeatPre, status4);
		 	removeOutliers(prev_right_pts, status4);
		 	removeOutliers(newFeatCur, status4);
		 	//removeOutliers(ids, status4);

		 	int newsize = ii + 1 + newFeatCur.size();

		 	for(int i = ii; i < newsize; ++i)
		 	{
		 		cur_left_pts[i] = newFeatCur[i-ii];
		 		prev_left_pts[i] = newFeatPre[i-ii];
		 		ids[i] = id_cnt++;
		 	}
		 	cur_left_pts.resize(newsize);
		 	prev_left_pts.resize(newsize);
		 	ids.resize(newsize);

		 	for(int i = 0; i < ii; ++i)
		 	{
		 		pair<int, Point2f> p;
		 		p.first = frame_cnt;
		 		p.second = pixel2cam(cur_left_pts[i], cam_intrinsics);
		 		
		 		landmarks[ids[i]]->observation.emplace_back(p);
		 	}
		 	for(int i = ii; i < newsize; ++i)
		 	{
		 		double disp = prev_left_pts[i].x - prev_right_pts[i-ii].x;
		 		double depth = BASELINE*fx/disp;
		 		if(depth > 2)
		 		{
		 			landmarks[ids[i]] = shared_ptr<LandMark>(new LandMark);

			 		pair<int,Point2f> p0, p1;
		 			p0.first = frame_cnt - 1;
		 			p0.second = pixel2cam(prev_left_pts[i], cam_intrinsics);
		 			

		 			p1.first = frame_cnt;
		 			p1.second = pixel2cam(cur_left_pts[i], cam_intrinsics);

		 			//cout<<fx<<endl;
		 			landmarks[ids[i]]->observation.emplace_back(std::move(p0));
		 			landmarks[ids[i]]->observation.emplace_back(std::move(p1));
		 			landmarks[ids[i]]->depth = depth;
		 		}
		 	}
	 	}
 	}

 	if(frame_cnt == WINDOW_SIZE - 1)
 	{
 		buildCoarse3DStructure();
 	}

 	if(frame_cnt >= WINDOW_SIZE)
 	{
 		int ii = -1;
	 	for(int i = ids.size() - 1; i >= 0; --i)
	 	{
	 		if(ids[i] >= 0)
	 		{
	 			ii = i+1;
	 			break;
	 		}
	 	}
	 	if(ii != ids.size())
	 	{
	 		vector<Point2f> newFeatPre, newFeatCur;
	 		for(int i = ii; i < ids.size(); ++i)
	 		{
	 			newFeatPre.emplace_back(prev_left_pts[i]);
	 			newFeatCur.emplace_back(cur_left_pts[i]);
	 		}

	 		calcOpticalFlowPyrLK(prev_left_pyramid_, prev_right_pyramid_,
		      newFeatPre, prev_right_pts,
		      status3, noArray(),
		      Size(21, 21),
		      3);

	 		for (int i = 0; i < prev_right_pts.size(); ++i) 
			{
			    if (status3[i] == 0) continue;
			    if (prev_right_pts[i].y < 0 ||
			        prev_right_pts[i].y > prev_right_img.rows-1 ||
			        prev_right_pts[i].x < 0 ||
			        prev_right_pts[i].x > prev_right_img.cols-1)
			      status3[i] = 0;
			}

			removeOutliers(newFeatCur, status3);
		 	removeOutliers(newFeatPre, status3);
		 	removeOutliers(prev_right_pts, status3);
		 	//removeOutliers(ids, status3);

		 	//findFundamentalMat(newFeatPre, prev_right_pts, cv::FM_RANSAC, 1.0, 0.99, status4);
		 	//checkEpipolarConstraint(newFeatPre, prev_right_pts, status4, 0.003);
		 	checkShiftsOnY(newFeatPre, prev_right_pts, status4, ShiftOnYTHS);
			removeOutliers(newFeatPre, status4);
		 	removeOutliers(prev_right_pts, status4);
		 	removeOutliers(newFeatCur, status4);
		 	//removeOutliers(ids, status4);

		 	int newsize = ii + 1 + newFeatCur.size();

		 	for(int i = ii; i < newsize; ++i)
		 	{
		 		cur_left_pts[i] = newFeatCur[i-ii];
		 		prev_left_pts[i] = newFeatPre[i-ii];
		 		ids[i] = id_cnt++;
		 	}
		 	cur_left_pts.resize(newsize);
		 	prev_left_pts.resize(newsize);
		 	ids.resize(newsize);

		 	for(int i = 0; i < ii; ++i)
		 	{
		 		pair<int, Point2f> p;
		 		p.first = frame_cnt;
		 		p.second = pixel2cam(cur_left_pts[i], cam_intrinsics);
		 		
		 		landmarks[ids[i]]->observation.emplace_back(p);
		 	}
		 	for(int i = ii; i < newsize; ++i)
		 	{
		 		double disp = prev_left_pts[i].x - prev_right_pts[i-ii].x;
		 		double depth = BASELINE*fx/disp;
		 		// if(depth > 2 && depth < 40)
		 		// {
		 			landmarks[ids[i]] = shared_ptr<LandMark>(new LandMark);

			 		pair<int,Point2f> p0, p1;
		 			p0.first = frame_cnt - 1;
		 			p0.second = pixel2cam(prev_left_pts[i], cam_intrinsics);
		 			

		 			p1.first = frame_cnt;
		 			p1.second = pixel2cam(cur_left_pts[i], cam_intrinsics);

		 			//cout<<fx<<endl;
		 			landmarks[ids[i]]->observation.emplace_back(std::move(p0));
		 			landmarks[ids[i]]->observation.emplace_back(std::move(p1));
		 			landmarks[ids[i]]->depth = depth;
		 	}

		 	for(int i = 0; i < WINDOW_SIZE-1; ++i)
		 	{
		 		Rp[i] = Rp[i+1];
		 		tp[i] = tp[i+1];
		 	}
		 	++frameID0;

		 	vector<Point2f> tmpR;
		 	vector<Point2f> tmpL(prev_left_pts);
		 	vector<Point2f> tmpCurL(cur_left_pts);
		 	vector<Point3f> tmp3d;
			vector<Point2f> tmp2d;

		 	vector<uchar> status5;
		 	calcOpticalFlowPyrLK(prev_left_pyramid_, prev_right_pyramid_,
		      tmpL, tmpR,
		      status5, noArray(),
		      Size(21, 21),
		      3);

		 	for (int i = 0; i < tmpR.size(); ++i) 
			{
			    if (status5[i] == 0) continue;
			    if (tmpR[i].y < 0 ||
			        tmpR[i].y > prev_right_img.rows-1 ||
			        tmpR[i].x < 0 ||
			        tmpR[i].x > prev_right_img.cols-1)
			      status5[i] = 0;
			}

			removeOutliers(tmpR, status5);
			removeOutliers(tmpL, status5);
			removeOutliers(tmpCurL, status5);

			vector<uchar> status6;
			checkShiftsOnY(tmpL, tmpR, status6, ShiftOnYTHS);
			removeOutliers(tmpR, status6);
		 	removeOutliers(tmpL, status6);
		 	removeOutliers(tmpCurL, status6);

		 	// Mat debug = prev_left_img.clone();
		 	// cvtColor(debug, debug, CV_GRAY2BGR);
		 	// for(int i = 0; i < tmpL.size(); ++i)
		 	// {
		 	// 	circle(debug, tmpL[i], 2, Scalar(0,0,255), 2);
		 	// 	line(debug, tmpL[i], tmpR[i], Scalar(0,255,255));
		 	// }
		 	// stringstream sstr;
		 	// sstr<<frame_cnt;
		 	// string filename = "../debug/"+sstr.str()+".png";
		 	// imwrite(filename, debug);
				 	//tmp3d.reszie(tmpL.szie());

		 	for(int i = 0; i < tmpL.size(); ++i)
		 	{
		 		double disp = tmpL[i].x - tmpR[i].x;
		 		double depth = BASELINE*fx/disp;
		 		if(depth > 0)
		 		{
		 			Point2f p1 = pixel2cam(tmpL[i], cam_intrinsics);

		 			tmp3d.emplace_back(Point3f(p1.x*depth, p1.y*depth, depth));

		 			// Vector3d triangulated_point;
		 			// Vector2d pl(tmpL[i]), pr(tmpR[i]);
		 			// triangulatePoint(Left_Cam_Pose, Right_Cam_Pose, )

		 			Point2f p2 = pixel2cam(tmpCurL[i], cam_intrinsics);
		 			tmp2d.emplace_back(std::move(p2));
		 		}
		 	}

		 	Matrix3d tmpR1 = Matrix3d::Identity();
		 	Vector3d tmpT1 = Vector3d(0, 0, 0);

		 	// tmpR1 = tmpR1.transpose();
		 	// tmpT1 = -tmpR1.transpose()*tmpT1;

		 	solvePnP(tmp3d, tmp2d, tmpR1, tmpT1, false);
		 	//cout<<tmpT1.transpose()<<endl<<endl;

		 	Rp[WINDOW_SIZE-1] = tmpR1.transpose();
		 	tp[WINDOW_SIZE-1] = -tmpR1.transpose()*tmpT1;

		 	Rp[WINDOW_SIZE-1] = Rp[WINDOW_SIZE-2]*Rp[WINDOW_SIZE-1];
		 	tp[WINDOW_SIZE-1] = Rp[WINDOW_SIZE-2]*tp[WINDOW_SIZE-1] + tp[WINDOW_SIZE-2];

		 	tmp3d.clear();
		 	tmp2d.clear();
		 }
	}
 	vector<Point2f> newFeat;
 	setMask();
	goodFeaturesToTrack(cur_left_img, newFeat,MAX_CNT - cur_left_pts.size(),0.01,MIN_DIST,mask);
	for(int i = 0; i < newFeat.size(); ++i)
	{
		cur_left_pts.emplace_back(newFeat[i]);
		ids.push_back(-1);
	}

	swap(cur_left_pts, prev_left_pts);
	swap(cur_left_pyramid_, prev_left_pyramid_);
    swap(cur_right_pyramid_, prev_right_pyramid_);
    prev_left_img = cur_left_img;

    cur_right_pyramid_.clear();
    cur_left_pyramid_.clear();
	cur_left_pts.clear();
	prev_right_pts.clear();
	frame_cnt++;
}
void ImageManager::getPairsForPnP(int id0, int id1, vector<Point3f>& p3d, vector<Point2f>& p2d, int tid)
{
	p3d.clear();
	p2d.clear();
	for(auto &it:landmarks)
	{
		int sz = it.second->observation.size();
		if(it.second->observation[0].first <= id0 && 
			it.second->observation[sz-1].first >= id1 && it.second->state == true && 
			it.second->depth > 2 && it.second->depth < 40)
		{
			if(it.second->state == true)
			{
				vector<pair<int, Point2f>>::iterator iter;
				iter = find_if(it.second->observation.begin(), it.second->observation.end(), 
					[tid](const pair<int, Point2f>& item){
					return item.first == tid;
				});
				if(iter != it.second->observation.end())
				{
					p3d.emplace_back(it.second->pos);
					p2d.emplace_back(iter->second);
				}
			}
		}
	}

	assert(p3d.size() == p2d.size());
}
int ImageManager::getTrackNumBetweenTwoFrame(int i, int j)
{
	int sum = 0;
	for(auto& it:landmarks)
	{
		int lz = it.second->observation.size();
		if(it.second->observation[0].first <= i && 
			it.second->observation[lz-1].first >= j)
			++sum;
	} 
	return sum;
}
void ImageManager::buildCoarse3DStructure()
{
	//投影矩阵
	Matrix3d Rot[WINDOW_SIZE];
	Vector3d Trans[WINDOW_SIZE];

	vector<Point2f> p2d;
	vector<Point3f> p3d;

	int l = -1;
	int n = -1;
	for(int i = 0; i < WINDOW_SIZE-1; ++i)
	{
		n = getTrackNumBetweenTwoFrame(i, WINDOW_SIZE - 1);
		//cout<<i<<" "<<n<<endl;

		if(n > MIN_TRACKED_FEATURES)
		{
			l = i;
			break;
		}
	}
	for(auto &it:landmarks)
	{
		int lz = it.second->observation.size();
		if(it.second->observation[0].first > l || 
			it.second->observation[lz-1].first < l)
			continue;

		vector<pair<int, Point2f>>::iterator iter;
		iter = find_if(it.second->observation.begin(), it.second->observation.end(), 
			[l](const pair<int, Point2f>& item){
				return item.first == l;
			});

		if(iter == it.second->observation.end())
			continue;
		Point2f p = iter->second;
		double dp = it.second->depth;

		it.second->state = true;
		it.second->pos = Point3f(p.x*dp, p.y*dp, dp);
	}
	
	Rot[l] = Matrix3d::Identity();
	Trans[l] = Vector3d(0,0,0);

	Rp[l] = Matrix3d::Identity();
	tp[l] = Vector3d(0,0,0);
	for(int i = l+1; i < frameID0+WINDOW_SIZE; ++i)
	{
		getPairsForPnP(l, i, p3d, p2d, i);
		Matrix3d R_init = Rot[i-1];
		Vector3d t_init = Trans[i-1];
		if(!solvePnP(p3d, p2d, R_init, t_init, true))
			cout<<"failed to solvePnP"<<endl;

		Rp[i] = R_init.transpose();
		tp[i] = -R_init.transpose()*t_init;
		for(auto& item:landmarks)
		{
			if(item.second->observation[0].first == i && 
				item.second->state == false)
			{
				Point2f pp = item.second->observation[0].second;
				double dp = item.second->depth;
				Vector3d pv(pp.x*dp, pp.y*dp, dp);
				Matrix3d Ri = R_init.transpose();
				Vector3d ti = -R_init.transpose()*t_init;

				Vector3d pw = Ri*pv + ti;

				item.second->state = true;
				item.second->pos = Point3f(pw[0], pw[1], pw[2]);
			}

		}
	}
	for(int i = l-1; i >= frameID0; --i)
	{
		getPairsForPnP(i, l, p3d, p2d, i);
		Matrix3d R_init = Rot[i+1];
		Vector3d t_init = Trans[i+1];
		if(!solvePnP(p3d, p2d, R_init, t_init, true))
			cout<<"failed to solvePnP"<<endl;

		Rp[i] = R_init.transpose();
		tp[i] = -R_init.transpose()*t_init;

		for(auto& item:landmarks)
		{
			if(item.second->observation[0].first == i && 
				item.second->state == false)
			{
				Point2f pp = item.second->observation[0].second;
				double dp = item.second->depth;
				Vector3d pv(pp.x*dp, pp.y*dp, dp);
				Matrix3d Ri = R_init.transpose();
				Vector3d ti = -R_init.transpose()*t_init;

				Vector3d pw = Ri*pv + ti;

				item.second->state = true;
				item.second->pos = Point3f(pw[0], pw[1], pw[2]);
			}

		}
	}
}
bool ImageManager::solvePnP(const vector<Point3f>& p3d, const vector<Point2f>& p2d, Matrix3d &R_initial, Vector3d &P_initial, bool use_ransac)
{
	cv::Mat r, rvec, t, D, tmp_r;
	cv::eigen2cv(R_initial, tmp_r);
	cv::Rodrigues(tmp_r, rvec);
	cv::eigen2cv(P_initial, t);
	cv::Mat K = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);

	vector<int> inliers;
	bool pnp_succ;
	if(use_ransac)
		pnp_succ = cv::solvePnPRansac(p3d, p2d, K, D, rvec, t, true, 100, 8.0, 0.99, inliers);
	else
		pnp_succ = cv::solvePnP(p3d, p2d, K, D, rvec, t, 1);

	if(!pnp_succ)
	{
		return false;
	}
	cv::Rodrigues(rvec, r);
	//cout << "r " << endl << r << endl;
	MatrixXd R_pnp;
	cv::cv2eigen(r, R_pnp);
	MatrixXd T_pnp;
	cv::cv2eigen(t, T_pnp);
	R_initial = R_pnp;
	P_initial = T_pnp;
	return true;
}
void ImageManager::triangulatePoint(Eigen::Matrix<double, 3, 4> &Pose0, Eigen::Matrix<double, 3, 4> &Pose1,
                        Eigen::Vector2d &point0, Eigen::Vector2d &point1, Eigen::Vector3d &point_3d)
{
    Eigen::Matrix4d design_matrix = Eigen::Matrix4d::Zero();
    design_matrix.row(0) = point0[0] * Pose0.row(2) - Pose0.row(0);
    design_matrix.row(1) = point0[1] * Pose0.row(2) - Pose0.row(1);
    design_matrix.row(2) = point1[0] * Pose1.row(2) - Pose1.row(0);
    design_matrix.row(3) = point1[1] * Pose1.row(2) - Pose1.row(1);
    Eigen::Vector4d triangulated_point;
    triangulated_point =
              design_matrix.jacobiSvd(Eigen::ComputeFullV).matrixV().rightCols<1>();
    point_3d(0) = triangulated_point(0) / triangulated_point(3);
    point_3d(1) = triangulated_point(1) / triangulated_point(3);
    point_3d(2) = triangulated_point(2) / triangulated_point(3);
}

double ImageManager::averageParallax(const vector<Point2f>& pts0, const vector<Point2f>& pts1, const vector<uchar>& status)
{
	double sum_parallax = 0;
	double sum_pts = 0;
	for(int i = 0; i < status.size(); ++i)
	{
		if(status[i])
		{
			double x2 = (pts0[i].x - pts1[i].x)*(pts0[i].x - pts1[i].x);
			double y2 = (pts0[i].y - pts1[i].y)*(pts0[i].y - pts1[i].y);
			sum_parallax += std::abs(std::sqrt(x2 + y2));
			++sum_pts;
		}
	}
	return sum_parallax/sum_pts;
}
