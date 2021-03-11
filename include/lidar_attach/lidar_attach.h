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

#ifndef LIDAR_ATTACH_H
#define LIDAR_ATTACH_H

#include <Eigen/Dense>
#include <opencv2/core/core.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

struct cameraIntrix{
	double fx;
	double fy;
	double cx;
	double cy;
};



void projectPointToImage(Eigen::Vector3f& point, const cameraIntrix& camera_intrix, cv::Point& cv_point);


class lidar_attach
{
public:
lidar_attach() = delete;
lidar_attach(std::string config_path);
lidar_attach(const Eigen::Matrix3f& rotation, const Eigen::Vector3f & trans,const cameraIntrix& camera_intrix);
lidar_attach(const Eigen::Matrix<float,3,4> T, const cameraIntrix& camer_intrix);

void attach(const pcl::PointCloud<pcl::PointXYZ>& cloud_in, const cv::Mat& image , pcl::PointCloud<pcl::PointXYZRGB> & cloud_out);

private:
	Eigen::Matrix3f _rotation;
	Eigen::Vector3f _trans;
	cameraIntrix _camera_intrix;
	

};

#endif // LIDAR_ATTACH_H
