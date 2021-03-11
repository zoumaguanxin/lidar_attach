/*
 * Copyright (c) 2020, <copyright holder> <email>
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *     * Neither the name of the <organization> nor the
 *     names of its contributors may be used to endorse or promote products
 *     derived from this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY <copyright holder> <email> ''AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL <copyright holder> <email> BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 */

#include "lidar_attach/lidar_attach.h"
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>


void projectPointToImage(Eigen::Vector3f& point, const cameraIntrix& camera_intrix, cv::Point& cv_point)
{
	Eigen::Matrix3f K;
	K << camera_intrix.fx, 0, camera_intrix.cx, 0, camera_intrix.fy, camera_intrix.cy, 0,0,1;
	//std::cout << K <<std::endl;
	Eigen::Vector3f normalize_point = point/point[2];
	Eigen::Vector3f cv_point3 = K*normalize_point;
	cv_point.x = std::floor(cv_point3[0]+0.5);
	cv_point.y = std::floor(cv_point3[1]+0.5);
}


lidar_attach::lidar_attach(std::__cxx11::string config_path)
{
	cv::FileStorage fs(config_path,cv::FileStorage::READ);
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
	lidar_attach(T_cl_eigen,ci);
}


lidar_attach::lidar_attach(const Eigen::Matrix3f& rotation, const Eigen::Vector3f& trans, const cameraIntrix& camera_intrix)
{
	_rotation = rotation;
	_trans = trans;
	_camera_intrix =camera_intrix;

}


lidar_attach::lidar_attach(const Eigen::Matrix< float, int(3), int(4) > T, const cameraIntrix& camer_intrix)
{
	_rotation = T.block(0,0,3,3);
	std::cout << "rotation" <<_rotation <<std::endl;
	_trans = T.block(0,3,3,1);
	std::cout << "trans" <<_trans <<std::endl;
	_camera_intrix = camer_intrix;
}


void lidar_attach::attach(const pcl::PointCloud< pcl::PointXYZ >& cloud_in, const cv::Mat& image, pcl::PointCloud< pcl::PointXYZRGB >& cloud_out)
{
	for(const auto & point: cloud_in.points){
		Eigen::Vector3f v_point(point.x, point.y, point.z);
		Eigen::Vector3f moved_point;
		moved_point = _rotation * v_point + _trans;
		pcl::PointXYZRGB tem;
		if(moved_point[2]>0.5){
			cv::Point cv_point;
			projectPointToImage(moved_point, _camera_intrix,cv_point);
			if(cv_point.x >0 && cv_point.x <image.cols && cv_point.y > 0 && cv_point.y < image.rows){
				cv::Vec3b b = image.at<cv::Vec3b>(cv_point);
				tem.r = b[2];
				tem.g = b[1];
				tem.b = b[0];
				tem.x = point.x;
				tem.y = point.y;
				tem.z = point.z;
				cloud_out.push_back(tem);
			}
		}

	}
}

