/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *
 * Author: Qin Tong (qintonguav@gmail.com)
 *******************************************************/

#pragma once
#include <vector>
#include <map>
#include <iostream>
#include <mutex>
#include <thread>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <ceres/ceres.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include "LocalCartesian.hpp"
#include "tic_toc.h"
#include <fstream>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

//#include "lidarFactor.hpp"
#include "common.h"


using namespace std;

class GlobalOptimization
{
public:
	GlobalOptimization();
	~GlobalOptimization();
	void inputGPS(double t, double latitude, double longitude, double altitude, double posAccuracy);
     void inputGPSPR(double t, double latitude, double longitude, double altitude, double posAccuracy);
        void inputPPKviz(double t, double latitude, double longitude, double altitude, double posAccuracy);
     void inputFRLviz(double t, double latitude, double longitude, double altitude, double w, double x, double y, double z);
	void inputOdom(double t, Eigen::Vector3d OdomP, Eigen::Quaterniond OdomQ);
	void getGlobalOdom(Eigen::Vector3d &odomP, Eigen::Quaterniond &odomQ);
        void GPS2XYZ(double latitude, double longitude, double altitude, double* xyz);
     void inputRot(double t, double q_w, double q_x, double q_y, double q_z, double rotAccuracy);
     void inputMag(double t, double mag_x, double mag_y, double mag_z, double magAccuracy);
     //void inputCloudFullRes(double t,pcl::PointCloud<PointType>::Ptr& laserCloudFullRes);
	nav_msgs::Path global_path;
     nav_msgs::Path gps_path; 
     nav_msgs::Path ppk_path;
     nav_msgs::Path frl_path;
     //pcl::PointCloud<PointType>::Ptr laserCloudFullRes2;
     //double timeLaserCloudFullResLast; //used to check if theres any new cloud to process
	double last_update_time;
     std::mutex mPoseMap;
	//double last_GPS=0; //rav

private:
	
	void optimize();
	void updateGlobalPath();

	// format t, tx,ty,tz,qw,qx,qy,qz
	map<double, vector<double>> localPoseMap;
	map<double, vector<double>> globalPoseMap;
     map<double, vector<double>> globalRotMap;
	map<double, vector<double>> GPSPositionMap;
     map<double, vector<double>> GPSPRPositionMap;
     map<double, vector<double>> PPKPositionMap;
     map<double, vector<double>> FRLPoseMap;
     map<double, vector<double>> magMap;
	bool initGPS;
	bool newGPS;
	bool newGPSPR;
     bool newRot;
     bool newMag;
     bool newCloudFullRes;
	
	GeographicLib::LocalCartesian geoConverter;
	Eigen::Matrix4d WGPS_T_WVIO;
        Eigen::Matrix4d WGPS_T_WVIO_viz;
        int update_count;
        int GTframeCount;
        std::ofstream outfileOdom;
        std::ofstream outfileGt;
	   std::ofstream outfileVINS;   //rav
	   std::ofstream outfileGPS;    //rav
	   std::ofstream outfileFusion; //rav
	Eigen::Vector3d lastP;
	Eigen::Quaterniond lastQ;
	std::thread threadOpt;
};
