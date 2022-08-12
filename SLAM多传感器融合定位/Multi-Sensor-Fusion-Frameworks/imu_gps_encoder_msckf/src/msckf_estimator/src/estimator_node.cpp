#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include "parameters.h"
#include <read_kaist_dataset/imu.h>
#include <read_kaist_dataset/encoder.h>
#include <read_kaist_dataset/gps.h>
#include <sensor_msgs/PointCloud.h>
#include <nav_msgs/Path.h>
#include "estimator.h"
using namespace std;

Estimator estimator;

std::condition_variable con;
std::mutex m_buf;

const double kToSecond = 1e-9;
double last_imu_t = 0;
queue<read_kaist_dataset::imu> imu_buf;
double last_encoder_t = 0;
queue<read_kaist_dataset::encoder> encoder_buf;
double last_gps_t = 0;
deque<read_kaist_dataset::gps> gps_buf;
bool init_feature = false;
double last_feature_t = 0;
queue<pair<double,sensor_msgs::PointCloudConstPtr>> feature_buf;

ros::Publisher pub_state;
ros::Publisher pub_gps_state;

void imu_callback(const  read_kaist_dataset::imu& imu_msg){
    if (imu_msg.timeStamp <= last_imu_t){
        ROS_WARN("imu message in disorder!");
        return;
    }
    last_imu_t = imu_msg.timeStamp;
	ROS_DEBUG("imu time: %f",last_imu_t);
    m_buf.lock();
    imu_buf.push(imu_msg);
    m_buf.unlock();
    con.notify_one();
}

void encoder_callback(const  read_kaist_dataset::encoder& encoder_msg){
    if (encoder_msg.timeStamp <= last_encoder_t){
        ROS_WARN("encoder message in disorder!");
        return;
    }
    last_encoder_t = encoder_msg.timeStamp;
	ROS_DEBUG("encoder time: %f",last_encoder_t);
    m_buf.lock();
    encoder_buf.push(encoder_msg);
    m_buf.unlock();
    con.notify_one();
}

void gps_callback(const  read_kaist_dataset::gps& gps_msg){
    if (gps_msg.timeStamp <= last_gps_t){
        ROS_WARN("gps message in disorder!");
        return;
    }
    last_gps_t = gps_msg.timeStamp;
	ROS_DEBUG("gps time: %f",last_gps_t);
    m_buf.lock();
    gps_buf.push_back(gps_msg);
    m_buf.unlock();
    con.notify_one();
}

void feature_callback(const sensor_msgs::PointCloudConstPtr &feature_msg)
{
    if (!init_feature){
        init_feature = true;
        return;
    }
    m_buf.lock();
	feature_buf.push(make_pair(feature_msg->header.stamp.toSec(), feature_msg));
    // feature_buf.push(make_pair(kToSecond*stod(feature_msg->header.frame_id),feature_msg));
	// ROS_DEBUG("feature time: %f",kToSecond*stod(feature_msg->header.frame_id));
    m_buf.unlock();
    con.notify_one();
}

vector<measurement> getMeasurements(){
	vector<measurement> measure_res;
	while(true){
        if (gps_buf.empty() || encoder_buf.empty() || imu_buf.empty() || feature_buf.empty())
            return measure_res;

		std::cout << std::fixed << std::setprecision(10) << "gps front stamp: " << gps_buf.front().timeStamp << std::endl;
		std::cout << "gps back  stamp: " << gps_buf.back().timeStamp << std::endl;
		std::cout << "feature   stamp: " << feature_buf.front().first << std::endl;

		if(gps_buf.back().timeStamp < feature_buf.front().first ||
			encoder_buf.back().timeStamp < feature_buf.front().first ||
			imu_buf.back().timeStamp < feature_buf.front().first)
			return measure_res; // wait for gps / encoder / imu data
		if(gps_buf.front().timeStamp > feature_buf.front().first ||
			encoder_buf.front().timeStamp > feature_buf.front().first ||
			imu_buf.front().timeStamp > feature_buf.front().first){
			ROS_INFO("%f, %f, %f, %f", gps_buf.front().timeStamp, encoder_buf.front().timeStamp, imu_buf.front().timeStamp, feature_buf.front().first);
            feature_buf.pop();
            continue;
		}

		double feature_timeStamp = feature_buf.front().first;
		sensor_msgs::PointCloudConstPtr img_msg = feature_buf.front().second;
        feature_buf.pop();

        vector<read_kaist_dataset::imu> IMUs;
        while (imu_buf.front().timeStamp < feature_timeStamp){
            IMUs.emplace_back(imu_buf.front());
            imu_buf.pop();
        }
		IMUs.emplace_back(imu_buf.front());

        vector<read_kaist_dataset::encoder> Encoders;
        while (encoder_buf.front().timeStamp < feature_timeStamp){
            Encoders.emplace_back(encoder_buf.front());
            encoder_buf.pop();
        }
		Encoders.emplace_back(encoder_buf.front());

        vector<read_kaist_dataset::gps> GPSs;
        while (gps_buf.front().timeStamp < feature_timeStamp){
            GPSs.emplace_back(gps_buf.front());
			gps_buf.pop_front();
        }
		GPSs.emplace_back(gps_buf.front());
		if(GPSs.size()>2){
			for(int i = GPSs.size() -2; i >= 0; i--)
				gps_buf.push_front(GPSs[i]);
			vector<read_kaist_dataset::gps> gps_res;
			gps_res.push_back(GPSs[GPSs.size()-2]);
			gps_res.push_back(GPSs[GPSs.size()-1]);
			GPSs = gps_res;
		}else if(GPSs.size()<2){
			ROS_WARN("GPS data error occur !!!!!");
		}else{
			gps_buf.push_front(GPSs[GPSs.size()-2]);
		}

		measurement meas;
		meas.measurements_feature = make_pair(feature_timeStamp,img_msg);
		meas.measurements_imu = IMUs;
		meas.measurements_encoder = Encoders;
		meas.measurements_gps = GPSs;
		measure_res.emplace_back(meas);
	}
	return measure_res;
}

bool checkMeasure(measurement& m){
	bool imu=false, encoder=false, gps=false;
	double feature_time=m.measurements_feature.first;
	imu=(m.measurements_imu[0].timeStamp<=feature_time)&&(m.measurements_imu.back().timeStamp>=feature_time);
	encoder=(m.measurements_encoder[0].timeStamp<=feature_time)&&(m.measurements_encoder.back().timeStamp>=feature_time);
	gps=(m.measurements_gps[0].timeStamp<=feature_time)&&(m.measurements_gps.back().timeStamp>=feature_time);
	if(!imu||!encoder||!gps){
		std::cout << "synchronize failure" << std::endl;
		ROS_WARN("Synchronize failure!!!!! ");
		ROS_WARN("img time: %f, imu back time: %f, encoder back time: %f , gps back time: %f", 
			m.measurements_feature.first, m.measurements_imu.back().timeStamp, 
			m.measurements_encoder.back().timeStamp, m.measurements_gps.back().timeStamp);
		ROS_WARN("img time: %f, imu begin time: %f, encoder begin time: %f , gps begin time: %f", 
			m.measurements_feature.first, m.measurements_imu[0].timeStamp, 
			m.measurements_encoder[0].timeStamp, m.measurements_gps[0].timeStamp);
		ROS_WARN("img time: %f, imu size: %d, encoder size: %d , gps size: %d", 
			m.measurements_feature.first, (int)m.measurements_imu.size(), 
			(int)m.measurements_encoder.size(), (int)m.measurements_gps.size());
		return false;
	}
	return true;
}

void process(){
	while(true){
		vector<measurement> measure;
        std::unique_lock<std::mutex> lk(m_buf);
        con.wait(lk, [&]
		{
			// std::cout << getMeasurements().size() << std::endl;
			return (measure = getMeasurements()).size() != 0;
        });
        lk.unlock();
		for(auto& m:measure){
			if(!checkMeasure(m))
				ROS_BREAK();
			estimator.processMeasurement(m);
			if(estimator.solver_flag == Estimator::SolverFlag::MSCKF){
				estimator.state2nav_msgs();
				pub_state.publish(estimator.fused_path);
				estimator.gps2nav_msgs();
				pub_gps_state.publish(estimator.gps_path);
				estimator.last_state = estimator.curr_state;
			}
		}
	}
}

int main(int argc, char **argv){
    ros::init(argc, argv, "msckf_estimator");
    ros::NodeHandle n("~");
	readParameters(n);
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);

	ros::Subscriber sub_imu = n.subscribe(IMU_TOPIC, 2000, imu_callback, ros::TransportHints().tcpNoDelay());
	ros::Subscriber sub_encoder = n.subscribe(ENCODER_TOPIC, 2000, encoder_callback, ros::TransportHints().tcpNoDelay());
	ros::Subscriber sub_gps = n.subscribe(GPS_TOPIC, 2000, gps_callback, ros::TransportHints().tcpNoDelay());
	ros::Subscriber sub_image = n.subscribe("/feature_tracker/feature", 2000, feature_callback);
	pub_state = n.advertise<nav_msgs::Path>("fused_path", 10);
	pub_gps_state = n.advertise<nav_msgs::Path>("gps_path", 10);

	std::thread measurement_process{process};
	ros::spin();
	return 0;
}