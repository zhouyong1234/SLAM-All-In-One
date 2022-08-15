#pragma once
#include <Eigen/Core>
#include <Eigen/SVD>
#include <iomanip>
#include <iostream>
#include <fstream>
#include <vector>
#include <utility>
using namespace std;
using namespace Eigen;

	//对尚未配对的轨迹进行配准
	//根据时间戳进行配对
	//若误差较小返回true，否则返回false
	inline bool registration(
		const vector<pair<double, Vector3d>>& t1,
		const vector<pair<double, Vector3d>>& t2,
		Matrix3d& R, Vector3d& t
	);

	//对已配对的轨迹进行配准
	//要求pts1和pts2的元素按顺序一一对应
	//若误差较小返回true，否则返回false
	inline bool pose_estimation_3d3d(
		const vector<Vector3d>& pts1,
		const vector<Vector3d>& pts2,
		Matrix3d& R, Vector3d& t
	);

	//根据时间进行配准的函数
	inline vector<pair<int, int>> GetMatchingTime(
		const vector<pair<double, Vector3d>>& pts1,
		const vector<pair<double, Vector3d>>& pts2);

	//从文件中读取轨迹的函数
	inline vector<pair<double, Vector3d>> ReadTrajetoryFromFile(std::string filename, int index1, int index2, int index3, int index4, bool is_inv, bool is_only_xy);


	//int main()
	//{
	//	//示例
	//    vector<pair<double,Vector3d>> t1=ReadTrajetoryFromFile("/home/zhouyuxuan/data/traj_vio.txt",1,2,3,0,false,false);
	//    vector<pair<double,Vector3d>> t2=ReadTrajetoryFromFile("/home/zhouyuxuan/data/MH03_gps_true.txt",1,2,3,0,false,false);
	//    Matrix3d R;
	//    Vector3d t;
	//    cout<<registration(t1,t2,R,t);
	//    cout<<"R:"<<endl<<R<<endl;
	//    cout<<"t:"<<endl<<t<<endl;
	//    return 0;
	//}

	inline bool registration(
		const vector<pair<double, Vector3d>>& t1,
		const vector<pair<double, Vector3d>>& t2,
		Matrix3d& R, Vector3d& t
	)
	{
		vector<pair<int, int >> v = GetMatchingTime(t1, t2);
		vector<Vector3d> vec1;
		vector<Vector3d> vec2;
		double ratio = 100;
		//cerr<<v.size()<<endl;
		int count = 0, count_Ref = 0;
		for (int i = 0; i < (int)v.size()*(ratio / 100.0); i++)
		{
			vec1.push_back(t1[v[i].first].second);
			vec2.push_back(t2[v[i].second].second);
			count = v[i].first;
			count_Ref = v[i].second;
		}
		return pose_estimation_3d3d(vec2, vec1, R, t);
	}

    inline bool registration_4DOF(
        const vector<pair<double, Vector3d>>& t1,
        const vector<pair<double, Vector3d>>& t2,
            const Matrix3d &Ren, double &yaw , Vector3d &t)
    {
        vector<pair<int, int >> v = GetMatchingTime(t1, t2);
        Vector3d v1=t1[v.back().first].second - t1[v.front().first].second;
        Vector3d v2=t2[v.back().second].second - t2[v.front().second].second;
        //cout<<v2.norm()<<endl;
        v2 = Ren.transpose()*v2;
        ROS_INFO("v1 v2 %f %f",v1.norm(),v2.norm());
        cout<<v1.transpose()<<endl;
        cout<<v2.transpose()<<endl;
        if(v1.norm()/v2.norm()<1.2 && v1.norm()/v2.norm()>0.8 && v1.norm()>1.5)
        {

            double dx = v2(0)-v1(0);
            double dy = v2(1)-v1(1);
            yaw=atan2(v2(1),v2(0))-atan2(v1(1),v1(0));

            Eigen::Matrix3d R;
            R<<cos(yaw),-sin(yaw),0,
               sin(yaw),cos(yaw),0,
               0,0,1;
            t=t2[v.back().second].second-Ren*R*t1[v.back().first].second;
            return true;
        }
        else return false;


    }

	inline vector<pair<double, Vector3d>> ReadTrajetoryFromFile(std::string filename, int index1, int index2, int index3, int index4, bool is_inv, bool is_only_xy)
	{
		std::fstream fs(filename);
		std::string buffer;
		std::stringstream ss;
		vector<pair<double, Vector3d>> vec;
		double x0, y0, z0;
		bool ref = false;
		while (fs.good())
		{
			Vector3d m;
			std::getline(fs, buffer);
			if (buffer[0] == '#' || buffer.length() < 2) continue;
			for (int i = 0; i < buffer.size(); i++)
			{
				if (buffer[i] == ',') buffer[i] = ' ';
			}
			ss.str("");
			ss.clear();
			ss << buffer;
			double x, y, z, t = 0, temp;
			for (int i = 0; ss.good(); i++)
			{
				ss >> temp;
				if (i == index1) x = temp;
				if (i == index2) y = temp;
				if (i == index3) z = temp;
				if (i == index4)
				{
					if (temp > 1e18) temp = temp / 1e9;
					t = temp;
				}
			}

			m << x, y, z;
			if (is_only_xy) m(2) = 0;

			if (is_inv) m = -m;
			vec.push_back(make_pair(t, m));

		}
		fs.close();
		return vec;
	}

	inline bool pose_estimation_3d3d(
		const vector<Vector3d>& pts1,
		const vector<Vector3d>& pts2,
		Matrix3d& R, Vector3d& t
	)
	{
		if (pts1.size() != pts2.size()) return false;
		Vector3d p1 = Vector3d::Zero();
		Vector3d p2 = Vector3d::Zero();     // center of mass
		int N = pts1.size();
		for (int i = 0; i < N; i++)
		{
			p1 += pts1[i];
			p2 += pts2[i];
		}
		p1 = p1 * (1.0 / N);
		p2 = p2 * (1.0 / N);
		vector<Vector3d>     q1(N), q2(N); // remove the center
		for (int i = 0; i < N; i++)
		{
			q1[i] = pts1[i] - p1;
			q2[i] = pts2[i] - p2;
		}


		Matrix3d W = Matrix3d::Zero();
		for (int i = 0; i < N; i++)
		{

			W += q1[i] * (q2[i].transpose());
		}

		Eigen::JacobiSVD<Eigen::Matrix3d> svd(W, Eigen::ComputeFullU | Eigen::ComputeFullV);
		Eigen::Matrix3d U = svd.matrixU();
		Eigen::Matrix3d V = svd.matrixV();
		Eigen::Vector3d singular = svd.singularValues();
		cout << "singular:"<<singular(0)<<" "<<singular(1)<<" "<<singular(2) << endl;
		
		R = U * (V.transpose());
		t = p1 - R * p2;

		//如果奇异值不满足某些条件，得到的结果就不可靠
		if (singular(0) / singular(1) > 5*5) return false;
        if (singular(0) < 2*2) return false;

		//convergence?
		double var = 0;
		double minx = 1e10;
		double maxx = -1e10;
		double miny = 1e10;
		double maxy = -1e10;
		double minz = 1e10;
		double maxz = -1e10;
		for (int i = 0; i < N; i++)
		{
			if (pts1[i](0) < minx) minx = pts1[i](0);
			if (pts1[i](0) > maxx) maxx = pts1[i](0);
			if (pts1[i](1) < miny) miny = pts1[i](1);
			if (pts1[i](1) > maxy) maxy = pts1[i](1);
			if (pts1[i](2) < minz) minz = pts1[i](2);
			if (pts1[i](2) > maxz) maxz = pts1[i](2);
			var += (R*pts2[i] + t - pts1[i]).norm()*(R*pts2[i] + t - pts1[i]).norm();
			//cout << R * pts2[i] + t - pts1[i] << endl;
		}
		var /= N;
		//double dx = maxx - minx;
		//double dy = maxy - miny;
		//double dz = maxz - minz;
		double dist = sqrt((maxx - minx)*(maxx - minx)
			+ (maxy - miny)*(maxy - miny)
			+ (maxz - minz)*(maxz - minz));
        ROS_INFO("sqrt(var): %f    dist: %f" ,sqrt(var),dist);
        if (sqrt(var) < dist / 4) return true;

		else return false;

	}

	inline vector<pair<int, int>> GetMatchingTime(
		const vector<pair<double, Vector3d>>& pts1,
		const vector<pair<double, Vector3d>>& pts2)
	{
		vector<pair<int, int>> vec;
		int i = 0, j = 0;
		double dt = 1000000000;
		while (i < pts1.size() && j < pts2.size())
		{
			//        cerr<<setprecision(20)<<pts1[i].at<double>(3)<<" "
			//           <<setprecision(20)<<pts2[j].at<double>(3)<<endl;
			if (pts1[i].first < pts2[j].first)
			{
				if (pts2[j].first - pts1[i].first < dt + 0.00001)
				{
					dt = pts2[j].first - pts1[i].first;
					i++;
					continue;
				}
				else
				{
					vec.push_back(pair<int, int>(i, j - 1));
					//cerr<<dt<<endl;
					//cerr<<i<<"/"<<pts1.size()<<" "<<j-1<<"/"<<pts2.size()<<endl;
					i++;
					dt = 1000000000;
					continue;
				}
			}
			else
			{
				if (pts1[i].first - pts2[j].first < dt + 0.00001)
				{
					dt = pts1[i].first - pts2[j].first;
					j++;
					continue;
				}
				else
				{
					vec.push_back(pair<int, int>(i - 1, j));
					// cerr<<dt<<endl;
					// cerr<<i-1<<"/"<<pts1.size()<<" "<<j<<"/"<<pts2.size()<<endl;
					j++;
					dt = 1000000;
					continue;
				}
			}
		}
		// cerr<<"!!!"<<endl;
		return vec;
	}
