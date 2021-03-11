#include <ros/ros.h>
#include <lidar_attach/lidar_attach.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <message_filters/subscriber.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <functional>
#include <opencv2/core/eigen.hpp>
#include <cv_bridge/cv_bridge.h>
#include <pcl_conversions/pcl_conversions.h>

lidar_attach *lidarAttch = nullptr;
ros::Publisher pub;

void callback(sensor_msgs::ImageConstPtr msg, sensor_msgs::PointCloud2ConstPtr lidar_msg){

	cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg);
	pcl::PointCloud<pcl::PointXYZ> cloud;

	pcl::fromROSMsg(*lidar_msg,cloud);
	pcl::PointCloud<pcl::PointXYZRGB> cloud_rgb;
	
	lidarAttch->attach(cloud, cv_ptr->image,cloud_rgb);
	sensor_msgs::PointCloud2 cloud2;
	pcl::toROSMsg(cloud_rgb,cloud2);
	cloud2.header.frame_id = lidar_msg->header.frame_id;
	cloud2.header.stamp = lidar_msg->header.stamp;
	pub.publish(cloud2);
}


int main(int argc, char** argv){
	
	ros::init(argc,argv,"lidar_attach");
	ros::NodeHandle nh;
	ros::NodeHandle private_nh("~");
	
	message_filters::Subscriber<sensor_msgs::Image> image_sub(nh,"/rgb_image",10);
	message_filters::Subscriber<sensor_msgs::PointCloud2> lidar_sub(nh,"/points_raw",10);
	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::PointCloud2>  MySyncPolicy;
	pub = nh.advertise<sensor_msgs::PointCloud2>("points_rgb_raw",1);

	message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(20),image_sub,lidar_sub);
	std::string path;
	private_nh.getParam("config_path",path);
	std::cout << "path is: "<< path <<std::endl;
	cv::FileStorage fs(path,cv::FileStorage::READ);
	std::cout << "parse file" << std::endl;
	cv::Mat T_C_L;
	cv::Mat K;
	fs["T_C_L"] >> T_C_L;
	std::cout << T_C_L <<std::endl;
	std::cout << "parse T_C_L" << std::endl;
	fs["K"] >> K;
	std::cout << "parse K" << std::endl;
	Eigen::MatrixXf T_cl_eigen(3,4);
	cv::cv2eigen(T_C_L,T_cl_eigen);
	std::cout << T_cl_eigen <<std::endl;
	std::cout << K << std::endl;
	cameraIntrix ci;
	ci.fx = K.at<double>(0,0);
	ci.fy = K.at<double>(1,0);
	ci.cx = K.at<double>(2,0);
	ci.cy = K.at<double>(3,0);
	std::cout << ci.fx << " " << ci.fy << " "<<ci.cx << " " << ci.cy <<std::endl;
	//return 0;
	
	lidar_attach la(T_cl_eigen,ci);
	lidarAttch = &la;
	
	sync.registerCallback(boost::bind(&callback,_1,_2));
	ros::spin();
	return 0;
}