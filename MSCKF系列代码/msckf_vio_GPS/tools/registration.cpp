#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <iomanip>
#include <iostream>
#include <vector>
#include <utility>
using namespace std;
void pose_estimation_3d3d (
    const vector<cv::Mat>& pts1,
    const vector<cv::Mat>& pts2,
    cv::Mat& R, cv::Mat& t
);
vector<pair<int,int>> GetMatchingTime (
    const vector<cv::Mat>& pts1,
    const vector<cv::Mat>& pts2);
vector<cv::Mat> ReadTrajetoryFromFile(std::string filename,int index1,int index2,int index3,int index4,bool is_inv,bool is_only_xy);

int main()
{
    vector<cv::Mat> t1=ReadTrajetoryFromFile("/home/zhouyuxuan/data/traj.txt",1,2,3,0,false,false);
    vector<cv::Mat> t2=ReadTrajetoryFromFile("/home/zhouyuxuan/data/MH03_gps.txt",1,2,3,0,false,false);
    cv::Mat R;
    cv::Mat t;
    vector<pair<int,int >> v=GetMatchingTime(t1,t2);
    vector<cv::Mat> vec1;
    vector<cv::Mat> vec2;
    double ratio=100;
    //cerr<<v.size()<<endl;
    int count=0,count_Ref=0;
    for(int i=0;i<(int)v.size()*(ratio/100.0);i++)
    {
                vec1.push_back(t1[v[i].first]);
                vec2.push_back(t2[v[i].second]);
                count=v[i].first;
                count_Ref=v[i].second;
    }
    pose_estimation_3d3d(vec2,vec1,R,t);
    ofstream ofs("transform.txt");
    ofs<<"R_e_w:"<<endl<<R<<endl;
    ofs<<"t_w_e:"<<endl<<t<<endl;

    
    return 0;
}

vector<cv::Mat> ReadTrajetoryFromFile(std::string filename,int index1,int index2,int index3,int index4,bool is_inv,bool is_only_xy)
{
    std::fstream fs(filename);
    std::string buffer;
    std::stringstream ss;
    vector<cv::Mat> vec;
    double x0,y0,z0;
    bool ref=false;
    while(fs.good())
    {
        cv::Mat m;
        std::getline(fs,buffer);
        if(buffer[0]=='#'||buffer.length()<2) continue;
        for(int i=0;i<buffer.size();i++)
        {
            if(buffer[i]==',') buffer[i]=' ';
        }
        ss.str("");
        ss.clear();
        ss<<buffer;
        double x,y,z,t=0,temp;
        for(int i=0;ss.good();i++)
        {
            ss>>temp;
            if(i==index1) x=temp;
            if(i==index2) y=temp;
            if(i==index3) z=temp;
            if(i==index4)
            {   if(temp>1e18) temp=temp/1e9;
                t=temp;
            }
        }

        m = (cv::Mat_<double>(4,1) << x,y,z,t);
        if(is_only_xy) m.at<double>(2)=0;

        if(is_inv) m=-m;
        vec.push_back(m);

    }
    fs.close();
    return vec;
}

void pose_estimation_3d3d (
    const vector<cv::Mat>& pts1,
    const vector<cv::Mat>& pts2,
    cv::Mat& R, cv::Mat& t
)
{
        cv::Mat p1=(cv::Mat_<double>(3,1) << 0,0,0);
        cv::Mat p2=(cv::Mat_<double>(3,1) << 0,0,0);     // center of mass
        int N = pts1.size();
        for ( int i=0; i<N; i++ )
        {
            p1 += pts1[i].rowRange(0,3);
            p2 += pts2[i].rowRange(0,3);
        }
        p1 = p1*(1.0/N);
        p2 = p2*(1.0/N);
        vector<cv::Mat>     q1 ( N ), q2 ( N ); // remove the center
        for ( int i=0; i<N; i++ )
        {
            q1[i] = pts1[i].rowRange(0,3) - p1;
            q2[i] = pts2[i].rowRange(0,3) - p2;
        }


        cv::Mat W = cv::Mat::zeros(3,3,CV_64F);
            for ( int i=0; i<N; i++ )
            {

                W += q1[i] * (q2[i].t());
            }
        cv::Mat S,U,VT;
        cv::SVD::compute(W,S,U,VT);
        R=U*VT;
        t=p1-R*p2;
}

vector<pair<int,int>> GetMatchingTime (
    const vector<cv::Mat>& pts1,
    const vector<cv::Mat>& pts2)
{
    vector<pair<int,int>> vec;
    int i=0,j=0;
    double dt=1000000000;
    while(i<pts1.size()&&j<pts2.size())
    {
//        cerr<<setprecision(20)<<pts1[i].at<double>(3)<<" "
//           <<setprecision(20)<<pts2[j].at<double>(3)<<endl;
       if(pts1[i].at<double>(3)<pts2[j].at<double>(3))
       {
          if(pts2[j].at<double>(3)-pts1[i].at<double>(3)<dt+0.00001)
          {
              dt=pts2[j].at<double>(3)-pts1[i].at<double>(3);
              i++;
              continue;
          }
          else
          {
              vec.push_back(pair<int,int>(i,j-1));
              //cerr<<dt<<endl;
              //cerr<<i<<"/"<<pts1.size()<<" "<<j-1<<"/"<<pts2.size()<<endl;
              i++;
              dt=1000000000;
              continue;
          }
       }
       else
       {
           if(pts1[i].at<double>(3)-pts2[j].at<double>(3)<dt+0.00001)
           {
               dt=pts1[i].at<double>(3)-pts2[j].at<double>(3);
               j++;
               continue;
           }
           else
           {
               vec.push_back(pair<int,int>(i-1,j));
              // cerr<<dt<<endl;
              // cerr<<i-1<<"/"<<pts1.size()<<" "<<j<<"/"<<pts2.size()<<endl;
               j++;
               dt=1000000;
               continue;
           }
       }
    }
   // cerr<<"!!!"<<endl;
    return vec;
}
