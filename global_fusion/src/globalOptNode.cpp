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

/* Updates MUN ISL
   * plot the raw GPS path using custom sources
	*
   * draw the magnetic eading vector using imu message
   * align the  vins path to magnetic north after bias correction
*/

//TODO  : the lidar message should be deskewed and transformed to the closest vio sample before sending to the mapper

#include "ros/ros.h"
#include "globalOpt.h"
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <iostream>
#include <stdio.h>
#include <queue>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include "nmea_msgs/Sentence.h"
#include <fstream>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/date_time/posix_time/posix_time_io.hpp>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/range_image/range_image.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/crop_box.h> 
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/search/impl/search.hpp>
#include <pcl/common/copy_point.h>
#include <pcl/io/pcd_io.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/octree/octree_pointcloud_pointvector.h>
#include <pcl/octree/octree_search.h>


#include "lidarFactor.hpp"
//#include "common.h"
#include "tic_toc.h"
#include "parameters.h"

typedef vector< tuple<size_t, size_t> > TupleList;

GlobalOptimization globalEstimator;
ros::Publisher pub_global_odometry, pub_global_path, pub_gps_path, pub_ppk_path, pub_frl_path, pub_car;
ros::Publisher pubLaserCloudSurround, pubLaserCloudMap, pubLaserCloudFullRes, pubOdomAftMapped, pubOdomAftMappedHighFrec, pubLaserAfterMappedPath, pubLaserCloudMapSurround;
nav_msgs::Path *global_path;
nav_msgs::Path *gps_path; // this is used to plot the gps_message path
nav_msgs::Path *ppk_path; // this is used to plot the gps_message path
nav_msgs::Path *frl_path;
nav_msgs::Path laserAfterMappedPath;
map<double, vector<double>> GPSPositionMap;
double last_vio_t = -1;
std::queue<nav_msgs::OdometryConstPtr> vioQueue;
std::queue<sensor_msgs::NavSatFixConstPtr> gpsQueue;
std::queue<sensor_msgs::NavSatFixConstPtr> prgpsQueue;
std::queue<geometry_msgs::QuaternionStampedConstPtr> rotQueue;
std::queue<geometry_msgs::Vector3StampedConstPtr> magQueue;
std::mutex m_buf;
bool rtk_unreliable = true;  //TODO  : send to config file // to cehck status of the GPS
bool use_gps = false;  // uses the raw rtk gps from fix
bool use_ppk = false;  //TODO  : send to config file
bool use_frl = false;  //TODO  : send to config file
bool use_frl_rot = false; // this one makes it use only the rotation
bool use_mag_head = true;// this one performs a heading only update
bool use_vio_atti = false;// this one performs a ref vector update (good for roll pitch global update using vio)
bool viz_ppk = false;  //TODO  : send to config file
bool viz_frl = true;  //TODO  : send to config file
bool use_lidar = true; // performs lidar mapping
bool use_pr_gps = false; // loop closure updates
bool use_lz_seg =true;
std::string ppk_pos_file = "/storage_ssd/bell412Dataset1/bell412_dataset1_ppk.pos"; //TODO  : send to config file
std::string frl_pos_file = "/storage_ssd/bell412_dataset1/bell412_dataset1_frl.pos"; //TODO  : send to config file
//std::string frl_pos_file = "/storage_ssd/bell412_dataset6/bell412_dataset6_frl.pos";
//std::string frl_pos_file = "/storage_ssd/bell412_dataset1/bell412_dataset1_frl.pos";
//std::string frl_pos_file = "/media/EC3C-97F9/bell412_data_capture_NRC/bell412_dataset5_frl.pos"; //TODO  : send to config file
std::string nmeaSentence = {};
std::string nmea_time;
boost::posix_time::time_duration nmea_time_pval;
boost::posix_time::time_duration ppk_time_pval;
boost::posix_time::ptime my_posix_time;
boost::gregorian::date my_posix_date;
boost::posix_time::time_duration my_time_of_day;
bool cloud_init= false;


std::ifstream myfile1 (ppk_pos_file);
std::ifstream myfile2 (frl_pos_file);

std::string line;
bool skip_read = false;
sensor_msgs::NavSatFix fix_ppk_msg;
geometry_msgs::QuaternionStamped rot_msg;

//input & output: points in one frame. local --> global
pcl::PointCloud<PointType>::Ptr laserCloudFullRes(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType> laserCloudFullMap;
std::queue<sensor_msgs::PointCloud2ConstPtr> fullResBuf;
double timeLaserCloudFullResLast = 0; //used to check if theres any new cloud to process
double timeLaserCloudFullRes = 0;
int path_start_index = 0;
int frameCount = 0;
bool map_init = false;
pcl::VoxelGrid<PointType> downSizeFilterFull;


//Octree
std::deque<pcl::PointCloud<PointType>> cloudBuf;
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudRGB;
float resolution_octree = 8.0f;
float slope_thresh = 0.5;
float z_th = 1.5;
//color values -> original
uint8_t r = 255;
uint8_t g = 0;
uint8_t b = 0;
int32_t rgb_or = (r << 16) | (g << 8) | b;

uint8_t r2 = 0;
uint8_t g2 = 255;
int32_t rgb_nw = (r2 << 16) | (g2 << 8) | b;


/// LZ segmentation
bool classicalSegmentFunction(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudIn){

            if(cloudIn->size() < 30){
                //PCL_ERROR("Could not estimate a planar model for the given data\n");
                //ROS_INFO("Less points");
                return false;
           }
             
 
            // SACSegmentation
            pcl::ModelCoefficients::Ptr coefficents(new pcl::ModelCoefficients);
            pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

            //create segmentation object
            pcl::SACSegmentation<pcl::PointXYZRGB> seg;

            // set the parameters
            seg.setOptimizeCoefficients(true);
            seg.setModelType(pcl::SACMODEL_PLANE);
            seg.setMethodType(pcl::SAC_RANSAC);
            seg.setDistanceThreshold(0.5);

            // input cloud and segment

            seg.setInputCloud(cloudIn);
            seg.segment(*inliers, *coefficents);

            //min max
            float zmin = 10000, zmax=-10000;
            float x_mean=0.0, y_mean=0.0, z_mean=0.0; 
            float xcrr, ycrr, zcrr, count=0.0;
		  for(unsigned int i = 0; i < cloudIn->size(); i++)
			{
				xcrr = cloudIn->at(i)._PointXYZRGB::x;
				ycrr = cloudIn->at(i)._PointXYZRGB::y;
    				zcrr = cloudIn->at(i)._PointXYZRGB::z;

				if(!isnan(x_mean) &&  !isnan(x_mean) && !isnan(x_mean) )
				{
					if(zcrr > zmax)
	        				zmax = zcrr;
	    				if(zcrr < zmin)
	        				zmin = zcrr;
				
					x_mean = x_mean + xcrr;
					y_mean = y_mean + ycrr;
					z_mean = z_mean + zcrr;
					count++;
				}	
			}
			if(count >0){
				x_mean = x_mean/count;
				y_mean = y_mean/count;
				z_mean = z_mean/count;}
			else {return false;}

            // TRANSFORM TO CENTER
            Eigen::Matrix4f tf_adj;
		  tf_adj << 1, 0, 0, -x_mean,
     		  	  0, 1, 0, -y_mean,
     	            0, 0, 1, -z_mean,
                 	  0, 0, 0, 1;
     	 // pcl::transformPointCloud(*cloudIn, *cloudIn, tf_adj);	
  		  //pcl::getMinMax3D (cloudIn, minPt, maxPt);

            float inlierSize = float(inliers->indices.size());
            float cloudSize = float(cloudIn->size());

            //ROS_INFO("test function %ld", inliers->indices.size());
            if(inliers->indices.size() == 0){
                //PCL_ERROR("Could not estimate a planar model for the given data\n");
                //ROS_INFO("Error");
                return false;
            }else{

                float inlier_percentage = ((inlierSize/cloudSize) * 100);
                //ROS_INFO("inlier per %f, %f,  %f", inlierSize, cloudSize, inlier_percentage);
                
                float coeffA = coefficents->values[0];
                float coeffB = coefficents->values[1];
                float coeffC = coefficents->values[2];
                float coeffD = coefficents->values[3];
                coeffA = coeffA/coeffC;
                coeffB = coeffB/coeffC;
                coeffD = coeffD/coeffC;

                //ROS_INFO("inlier per %f, %f, %f,  %f", coeffA, coeffB, coeffD, inlier_percentage);

                //posesFile << inlier_percentage <<" A: " << coeffA << " , B: " << coeffB <<  " , C: " << coeffC << "\n";

                if(inlier_percentage > 90.0){


                    if((! isnan(coeffA)) && (! isnan(coeffB)) && (coeffA > -slope_thresh) && (coeffA < slope_thresh) && (coeffB > -slope_thresh) && (coeffB < slope_thresh)  && ((zmax-zmin) < z_th)){
                        ROS_INFO("inlier per %f, %f, %f, %f, %f. %f, %f", coeffA, coeffB, coeffD, zmax, zmin, cloudSize,inlier_percentage);
                        return true;
                    }else{
                        return false;
                    }
                }else{
                    return false;
                }              
            }
        }


pcl::PointCloud<pcl::PointXYZRGB>::Ptr classicalSegmentCloud(pcl::PointCloud<PointType>::Ptr cloudIn){

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudInterim(new pcl::PointCloud<pcl::PointXYZRGB>());

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudOut(new pcl::PointCloud<pcl::PointXYZRGB>());

	cloudRGB->points.resize(cloudIn->size());

            //ROS_INFO("cloudIn size %ld", cloudIn->size());

     for (size_t i=0; i<cloudIn->points.size(); i++){
                cloudRGB->points[i].x = cloudIn->points[i].x;
                cloudRGB->points[i].y = cloudIn->points[i].y;
                cloudRGB->points[i].z = cloudIn->points[i].z;

                cloudRGB->points[i].rgb = rgb_or;
     }

            pcl::octree::OctreePointCloudSearch<PointType> octreeT (resolution_octree);
            
            octreeT.setInputCloud(cloudIn);
            octreeT.addPointsFromInputCloud();

            
            pcl::PointCloud<PointType>::VectorType voxelCenters;
            octreeT.getOccupiedVoxelCenters(voxelCenters);

            for (size_t r=0; r<voxelCenters.size(); ++r){
                Eigen::Vector3f centerVal= voxelCenters[r].getVector3fMap();
                
                std::vector<int> pointIdxVec;

                if(octreeT.voxelSearch(voxelCenters[r],pointIdxVec)){
                    //ROS_INFO("pointIdVec size %ld", pointIdxVec.size());

                    if(pointIdxVec.size()>15){

                        cloudInterim->resize(pointIdxVec.size());

                        TupleList tl;

                        for(size_t i=0;i<pointIdxVec.size(); ++i){


                            cloudInterim->points[i].x = cloudRGB->points[pointIdxVec[i]].x;
                            cloudInterim->points[i].y = cloudRGB->points[pointIdxVec[i]].y;
                            cloudInterim->points[i].z = cloudRGB->points[pointIdxVec[i]].z;
                            cloudInterim->points[i].rgb = rgb_or;

                            tl.push_back(tuple<size_t,size_t>(i,pointIdxVec[i]));

                        }

                        bool isFlat = classicalSegmentFunction(cloudInterim);

                        if(isFlat){
                            for (size_t j=0;j<pointIdxVec.size(); ++j){
                                //ROS_INFO("idx %d", idx);
                                cloudRGB->points[pointIdxVec[j]].r = 0;
                                cloudRGB->points[pointIdxVec[j]].g = 255;
                                cloudRGB->points[pointIdxVec[j]].b = 121;
                            }
                        }else{
                           // ROS_INFO("Could not estimate a planar model for the given data\n");                            
                        }
                        
                    }
                                       
                }
            }

            return cloudRGB;
        }


// LZ segmentation

void publish_car_model(double t, Eigen::Vector3d t_w_car, Eigen::Quaterniond q_w_car)
{
    visualization_msgs::MarkerArray markerArray_msg;
    visualization_msgs::Marker car_mesh;
    car_mesh.header.stamp = ros::Time(t);
    car_mesh.header.frame_id = "worldGPS";
    car_mesh.type = visualization_msgs::Marker::MESH_RESOURCE;
    car_mesh.action = visualization_msgs::Marker::ADD;
    car_mesh.id = 0;

    car_mesh.mesh_resource = "package://global_fusion/models/car.dae";

    Eigen::Matrix3d rot;
    rot << 0, 0, -1, 0, -1, 0, -1, 0, 0;
    rot << -1, 0, 0, 0, 0, -1, 0, -1, 0; //our dataset
    
    
    Eigen::Quaterniond Q;
    Q = q_w_car * rot; 
    car_mesh.pose.position.x    = t_w_car.x();
    car_mesh.pose.position.y    = t_w_car.y();
    car_mesh.pose.position.z    = t_w_car.z();
    car_mesh.pose.orientation.w = Q.w();
    car_mesh.pose.orientation.x = Q.x();
    car_mesh.pose.orientation.y = Q.y();
    car_mesh.pose.orientation.z = Q.z();

    car_mesh.color.a = 1.0;
    car_mesh.color.r = 1.0;
    car_mesh.color.g = 0.0;
    car_mesh.color.b = 0.0;

    float major_scale = 2.0;

    car_mesh.scale.x = major_scale;
    car_mesh.scale.y = major_scale;
    car_mesh.scale.z = major_scale;
    markerArray_msg.markers.push_back(car_mesh);
    pub_car.publish(markerArray_msg);
}	

void PR_GPS_callback(const sensor_msgs::NavSatFixConstPtr &GPS_msg)
{
    if(!use_pr_gps) return;
    m_buf.lock();
    prgpsQueue.push(GPS_msg);
    m_buf.unlock();

}

void GPS_callback(const sensor_msgs::NavSatFixConstPtr &GPS_msg)
{
    if(!use_gps) return;

    m_buf.lock();
    gpsQueue.push(GPS_msg);
    m_buf.unlock();

    /*
    //1. get the lat long altitude
    double gps_t = GPS_msg->header.stamp.toSec();
    double latitude = GPS_msg->latitude;
    double longitude = GPS_msg->longitude;
    double altitude = GPS_msg->altitude;
    int fixstatus = GPS_msg->status.status;
    //2. convert to xyz
    double xyz[3];
    globalEstimator.GPS2XYZ(latitude, longitude, altitude, xyz);
    if (fixstatus==1){
    	printf("received xyz: %f,%f,%f\n", xyz[0], xyz[1], xyz[2]);
    }
    //vector<double> tmp{xyz[0], xyz[1], xyz[2], posAccuracy};
    //3. append to the gps_path
    vector<double> tmp{xyz[0], xyz[1], xyz[2], 0.0};
    GPSPositionMap[gps_t] = tmp;
    
    
    //4. publish the path
    gps_path.poses.clear();
    map<double, vector<double>>::iterator iter;
    for (iter = GPSPositionMap.begin(); iter != GPSPositionMap.end(); iter++)
    {
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header.stamp = ros::Time(iter->first);
        pose_stamped.header.frame_id = "worldGPS";
        pose_stamped.pose.position.x = iter->second[0];
        pose_stamped.pose.position.y = iter->second[1];
        pose_stamped.pose.position.z = iter->second[2];
        pose_stamped.pose.orientation.w = 1.0;
        pose_stamped.pose.orientation.x = 0.0;
        pose_stamped.pose.orientation.y = 0.0;
        pose_stamped.pose.orientation.z = 0.0;
        gps_path.poses.push_back(pose_stamped);
    }*/


    /*
    //printf("GPS_callback! \n");
    double t = GPS_msg->header.stamp.toSec();
    //printf("receive GPS with timestamp %f\n", GPS_msg->header.stamp.toSec());
    double latitude = GPS_msg->latitude;
    double longitude = GPS_msg->longitude;
    double altitude = GPS_msg->altitude;
    //int numSats = GPS_msg->status.service;
    double pos_accuracy = GPS_msg->position_covariance[0];
    //printf("receive covariance %lf \n", pos_accuracy);
    globalEstimator.inputGPS(t, latitude, longitude, altitude, pos_accuracy);*/

    
}

void mag_callback(const geometry_msgs::Vector3Stamped::ConstPtr &mag_msg)
{
    if (use_mag_head){
	    m_buf.lock();
	    magQueue.push(mag_msg);
	    m_buf.unlock();
    }
}


void vio_callback(const nav_msgs::Odometry::ConstPtr &pose_msg)
{ 
	// keep vio times in a que finite to help with lidar processing
    m_buf.lock();
    vioQueue.push(pose_msg);
    m_buf.unlock();
    // if vio que has older messages than 1s pop the buffer
   
    double t = pose_msg->header.stamp.toSec();
    /*nav_msgs::Odometry::ConstPtr pose_msg_first = vioQueue.front();
    double vio_first_t = pose_msg_first->header.stamp.toSec();
    
    if (t - vio_first_t > 1.0){
      vioQueue.pop();
      //printf("vio queue length: %lu \n", vioQueue.size());
    }*/
    

    //printf("vio_callback! \n");
    
    Eigen::Vector3d vio_t(pose_msg->pose.pose.position.x, pose_msg->pose.pose.position.y, pose_msg->pose.pose.position.z);
    Eigen::Quaterniond vio_q;
    vio_q.w() = pose_msg->pose.pose.orientation.w;
    vio_q.x() = pose_msg->pose.pose.orientation.x;
    vio_q.y() = pose_msg->pose.pose.orientation.y;
    vio_q.z() = pose_msg->pose.pose.orientation.z;
    globalEstimator.inputOdom(t, vio_t, vio_q);
    
    
    // add GPS factors to the graph
    m_buf.lock();
    while(!gpsQueue.empty())
    {
        sensor_msgs::NavSatFixConstPtr GPS_msg = gpsQueue.front();
        double gps_t = GPS_msg->header.stamp.toSec();
        printf("vio t: %f, gps t: %f \n", t, gps_t);
        // 10ms sync tolerance
        //if(gps_t >= t - 0.01 && gps_t <= t + 0.01)
        if(gps_t >= t - 0.1 && gps_t <= t + 0.1) //TODO: get from config ( this can be larger for unsynced data)  
        {
            //printf("receive GPS with timestamp %f\n", GPS_msg->header.stamp.toSec());
            double latitude = GPS_msg->latitude;
            double longitude = GPS_msg->longitude;
            double altitude = GPS_msg->altitude;
            int fixstatus = GPS_msg->status.status;  //this is 2 for rtk and 
            double pos_accuracy = GPS_msg->position_covariance[0];
            if(pos_accuracy > 0 && fixstatus >= 0){
 		if(rtk_unreliable ){
 			if(fixstatus == 2 ){
                	//printf("synced| receive covariance %lf | fix status(2:RTK) %i \n", pos_accuracy, fixstatus); // use this to check gps sync errors
                	globalEstimator.inputGPS(t, latitude, longitude, altitude, pos_accuracy); }}
		else {
			globalEstimator.inputGPS(t, latitude, longitude, altitude, pos_accuracy);
		}
            }
            gpsQueue.pop();
            break;
        }
        //else if(gps_t < t - 0.01)
        else if(gps_t < t - 0.1)
            gpsQueue.pop();
        //else if(gps_t > t + 0.01)
        else if(gps_t > t + 0.1)
            break;
    }


    // add Place recognition factors to the graph
    while(!prgpsQueue.empty())
    {
        sensor_msgs::NavSatFixConstPtr GPS_msg = prgpsQueue.front();
        double gps_t = GPS_msg->header.stamp.toSec();
        printf("vio t: %f, prgps t: %f \n", t, gps_t);
        // 10ms sync tolerance
        //if(gps_t >= t - 0.01 && gps_t <= t + 0.01)
        if(gps_t >= t - 0.1 && gps_t <= t + 0.1) //TODO: get from config ( this can be larger for unsynced data)  
        {
            //printf("receive GPS with timestamp %f\n", GPS_msg->header.stamp.toSec());
            double latitude = GPS_msg->latitude;
            double longitude = GPS_msg->longitude;
            double altitude = GPS_msg->altitude;
            int fixstatus = GPS_msg->status.status;  //this is 2 for rtk and 
            double pos_accuracy = GPS_msg->position_covariance[0];
 		  if(fixstatus == 2 ){
                	//printf("synced| receive covariance %lf | fix status(2:RTK) %i \n", pos_accuracy, fixstatus); // use this to check gps sync errors
                	globalEstimator.inputGPSPR(t, latitude, longitude, altitude, pos_accuracy); }
		  //if(fixstatus == 1 ){
                	//printf("synced| receive covariance %lf | fix status(2:RTK) %i \n", pos_accuracy, fixstatus); // use this to check gps sync errors
                	//globalEstimator.inputGPSLC(t, latitude, longitude, altitude, pos_accuracy); }
            prgpsQueue.pop();
            break;
        }
        //else if(gps_t < t - 0.01)
        else if(gps_t < t - 0.1)
            gpsQueue.pop();
        //else if(gps_t > t + 0.01)
        else if(gps_t > t + 0.1)
            break;
    }



    // process the rot buffer
    while(!rotQueue.empty())
    {
        geometry_msgs::QuaternionStampedConstPtr rot_msg = rotQueue.front();
        double rot_t = rot_msg->header.stamp.toSec();
        //printf("vio t: %f, rot t: %f \n", t, rot_t);
        // 10ms sync tolerance
        //if(gps_t >= t - 0.01 && gps_t <= t + 0.01)
        if(rot_t >= t - 0.1 && rot_t <= t + 0.1) //TODO: get from config ( this can be larger for unsynced data)  
        {
            //printf("receive GPS with timestamp %f\n", GPS_msg->header.stamp.toSec());
            double quat_w = rot_msg->quaternion.w;
            double quat_x = rot_msg->quaternion.x;
            double quat_y = rot_msg->quaternion.y;
            double quat_z = rot_msg->quaternion.z;
            double quat_accuracy = 0.01;
            globalEstimator.inputRot(t, quat_w, quat_x, quat_y, quat_z, quat_accuracy);
            rotQueue.pop();
            break;
        }
        //else if(gps_t < t - 0.01)
        else if(rot_t < t - 0.1)
            rotQueue.pop();
        else if(rot_t > t + 0.1)
            break;
    }

    // process the mag buffer
    while(!magQueue.empty())
    {
        geometry_msgs::Vector3StampedConstPtr mag_msg = magQueue.front();
        double mag_t = mag_msg->header.stamp.toSec();
        
        // 10ms sync tolerance
        //if(gps_t >= t - 0.01 && gps_t <= t + 0.01)
        if(mag_t >= t - 0.05 && mag_t <= t + 0.05) //TODO: get from config ( this can be larger for unsynced data)  
        {
            //printf("receive GPS with timestamp %f\n", GPS_msg->header.stamp.toSec());
            printf("vio t: %f, mag t: %f \n", t, mag_t);
            double mag_x = mag_msg->vector.x;
            double mag_y = mag_msg->vector.y;
            double mag_z = mag_msg->vector.z;
            double mag_accuracy = 0.1;
            globalEstimator.inputMag(t, mag_x, mag_y, mag_z, mag_accuracy);
            magQueue.pop();
            break;
        }
        //else if(gps_t < t - 0.01)
        else if(mag_t < t - 0.05)
            magQueue.pop();
        else if(mag_t > t + 0.05)
            break;
    }

    
    
    m_buf.unlock();

    Eigen::Vector3d global_t;
    Eigen:: Quaterniond global_q;
    globalEstimator.getGlobalOdom(global_t, global_q);

    nav_msgs::Odometry odometry;
    odometry.header = pose_msg->header;
    odometry.header.frame_id = "world";
    odometry.child_frame_id = "world";
    odometry.pose.pose.position.x = global_t.x();
    odometry.pose.pose.position.y = global_t.y();
    odometry.pose.pose.position.z = global_t.z();
    odometry.pose.pose.orientation.x = global_q.x();
    odometry.pose.pose.orientation.y = global_q.y();
    odometry.pose.pose.orientation.z = global_q.z();
    odometry.pose.pose.orientation.w = global_q.w();
    pub_global_odometry.publish(odometry);
    pub_global_path.publish(*global_path);
    pub_gps_path.publish(*gps_path);
    pub_ppk_path.publish(*ppk_path);
    pub_frl_path.publish(*frl_path);
    publish_car_model(t, global_t, global_q);
    /*0if(globalEstimator.timeLaserCloudFullRes>0.0){
	 sensor_msgs::PointCloud2 laserCloudFullRes3;
	 pcl::toROSMsg(*laserCloudFullRes2, laserCloudFullRes3);
	 laserCloudFullRes3.header.stamp = ros::Time().fromSec(timeLaserCloudFullRes);
	 laserCloudFullRes3.header.frame_id = "/world";
	 pubLaserCloudFullRes.publish(laserCloudFullRes3);
	}*/
    
}

void split(const std::string &s, char delim, std::vector<std::string> &elems) {
    std::stringstream ss;
    ss.str(s);
    std::string item;
    while (std::getline(ss, item, delim)) {
        elems.push_back(item);
    }
}

float GpsToDecimalDegrees(const char* nmeaPos, char quadrant)
{
  float v= 0;
  if(strlen(nmeaPos)>5)
  {
    char integerPart[3+1];
    int digitCount= (nmeaPos[4]=='.' ? 2 : 3);
    memcpy(integerPart, nmeaPos, digitCount);
    integerPart[digitCount]= 0;
    nmeaPos+= digitCount;
    v= atoi(integerPart) + atof(nmeaPos)/60.;
    if(quadrant=='W' || quadrant=='S')
      v= -v;
  }
  return v;
}

ros::Time findRosTimeFromNmea(ros::Time stamp, std::string substr){
				my_posix_date   =    stamp.toBoost().date();
				//my_posix_date   =    boost::gregorian::date(2022,1,16);
	  			my_time_of_day  =    boost::posix_time::hours(std::stod(substr.substr(0,2))) +
						     boost::posix_time::minutes(std::stod(substr.substr(2,2))) + 
						     boost::posix_time::seconds(std::stod(substr.substr(4,2))) +
						     boost::posix_time::millisec(std::stod(substr.substr(7,2))*10); 
				
				my_posix_time   =    boost::posix_time::ptime(my_posix_date, my_time_of_day);
				
				

				//UTC 00:00 handling
				// get the difference in calculated clock and header clock
				boost::posix_time::time_duration time_diff = my_posix_time - stamp.toBoost();
				//ROS_INFO_STREAM("diff: "<< time_diff ) ; //ok
				//if the clock is off more than 24 hrs throw an error -this is not currently handled
				if (time_diff >= boost::posix_time::hours(24) || time_diff <= -boost::posix_time::hours(24)){
					ROS_INFO_STREAM("Clocks are off more than 24 Hrs (not handled in code) :"<< time_diff) ; //ok
					ROS_ERROR("Update code");
					ros::shutdown();
				}
				// if it is larger than 12hrs substract a day
				else if (time_diff >= boost::posix_time::hours(12)){
					my_posix_time = my_posix_time - boost::posix_time::hours(24);
					ROS_INFO_STREAM("UTC 00:00 corrected :"<< my_posix_time ) ; //ok
				}
				else if (time_diff <= -boost::posix_time::hours(12)){
					my_posix_time = my_posix_time + boost::posix_time::hours(24);
					ROS_INFO_STREAM("UTC 00:00 corrected :"<< my_posix_time ) ; //ok
					// if it is lees than -12hrs add a day
				}
			
				//perform clock correction
				return ros::Time::fromBoost(my_posix_time);

}

void nmeaCallback(const nmea_msgs::Sentence::ConstPtr& msg)
{  
    nmeaSentence = msg->sentence;
    
    //cout << nmeaSentence << endl;
    std::istringstream ss5(nmeaSentence);
    string token;
    std::getline(ss5,token, ','); string nmea_type = token;
    if (nmea_type.compare("$GNGGA") != 0)
	return;
    std::getline(ss5,token, ','); nmea_time = token; if(token.length()<1) return;
    std::getline(ss5,token, ','); string nmea_lat_str = token;  if(token.length()<1) return;
    std::getline(ss5,token, ','); double nmea_lat = GpsToDecimalDegrees(nmea_lat_str.c_str(), token.at(0));
    std::getline(ss5,token, ','); string nmea_lon_str = token; 
    std::getline(ss5,token, ','); double nmea_lon = GpsToDecimalDegrees(nmea_lon_str.c_str(), token.at(0));
    std::getline(ss5,token, ','); //int nmea_fix = stoi(token);
    std::getline(ss5,token, ',');
    std::getline(ss5,token, ',');
    std::getline(ss5,token, ','); double nmea_alt = stod(token);


    std::stringstream nmea_time_concat;
    nmea_time_concat << nmea_time.substr(0,2) << ":"  << nmea_time.substr(2,2) << ":" << nmea_time.substr(4,5) << "0";
    std::string nmea_time_formatted_to_string = nmea_time_concat.str();

    
  
    //cout << "nmea received time:" << nmea_time << endl; //k
    
    //define the variables 
    std::string tempppk,templat,templon,tempalt,fix_ppk_time;

    if(use_ppk || viz_ppk){
    //open file
    if (myfile1.is_open())
    {
     	
 	bool ppk_synced = false;
	while (!ppk_synced)
	{
		//0. read the next line
		if (!skip_read){
		std::getline(myfile1, line);} 
		
    		//1. remove the header info in pos file
		if(line.at(0) == '%') continue;
		//cout<<line<<endl;
 
          //2. extract the position message line
          //2022/05/25 20:43:10.200   45.325064906  -75.664896317    86.7785   1  28   0.0035   0.0025   0.0064  -0.0005  -0.0015  -0.0021   0.01    7.9
		std::string tempdate,temptime,templat,templon,tempalt,tempQ,tempnumsat,tempsdn,tempsde,tempsdu,tempsdne,tempsdeu,tempsdun,tempage,tempratio;
          std::istringstream iss1(line);
    		iss1 >> tempdate >> temptime >> templat >> templon >> tempalt >> tempQ >> tempnumsat >> tempsdn >> tempsde >> tempsdu >> tempsdne >> tempsdeu >> tempsdun >> tempage >>  tempratio;
		
		//cout<<"ppk time string:"<<temptime<< " Nmea time: "<<nmea_time_formatted_to_string <<endl;//k
		// TODO:quick check of the UTC date to warn possible errors
		// convert ros time of nmea stamp to UTC and and check with PPK time
		
		// 3.1 Convert the two times to seconds
		//time value conversions
  		nmea_time_pval  =    boost::posix_time::hours(std::stod(nmea_time_formatted_to_string.substr(0,2))) +
				           boost::posix_time::minutes(std::stod(nmea_time_formatted_to_string.substr(3,2))) + 
			                boost::posix_time::seconds(std::stod(nmea_time_formatted_to_string.substr(6,2))) +
		                     boost::posix_time::millisec(std::stod(nmea_time_formatted_to_string.substr(9,2))*10);

		ppk_time_pval  =     boost::posix_time::hours(std::stod(temptime.substr(0,2))) +
				           boost::posix_time::minutes(std::stod(temptime.substr(3,2))) + 
			                boost::posix_time::seconds(std::stod(temptime.substr(6,2))-18) + //TODO:GPST to UTC correction
		                     boost::posix_time::millisec(std::stod(temptime.substr(9,2))*10);
		boost::posix_time::time_duration delta = ppk_time_pval-nmea_time_pval;
		double diff = delta.total_microseconds();
		//cout<<"ppk time val:"<< ppk_time_pval << " Nmea time val: "<< nmea_time_pval << "|" << diff << endl;//k

		//3.keep reading the file if the nmea time is ahead
          if (diff<0){
		     skip_read = false;
		     ppk_synced = false;
                     //cout<<"NMEA ahead:"<< ppk_time_pval << " Nmea time val: "<< nmea_time_pval << "|" << diff << endl;
		     continue;
		}

		//4.do not read the next line if the ppk time is ahead
		if (diff>0){
		     skip_read = true;
		     ppk_synced = true; 
		     continue;
		}

          //5.if they match input the GPS data to the optimizer
          if (diff==0){
		     skip_read = false;
		     ppk_synced = true; 


               ros::Time nmea_ros_time=findRosTimeFromNmea(msg->header.stamp,nmea_time);
		     //cout<<"** SYNCED: ppk time val:"<< ppk_time_pval << " Nmea time val: "<< nmea_time_pval << "|" << diff << "|" << nmea_lat << "|" << nmea_lon << "|" << nmea_alt << "|" <<  stof(templat) << "|" <<  stof(templon) << "|" <<  stof(tempalt) << endl;
		     //cout << nmea_ros_time << "|" << msg->header.stamp << endl;
  		     // Create a ppk GPS message
	          fix_ppk_msg.header = msg->header;
		     fix_ppk_msg.status.status = 2;
		     fix_ppk_msg.status.service = 1;
	          fix_ppk_msg.latitude = stof(templat); // deg
		     fix_ppk_msg.longitude = stof(templon); // deg
		     fix_ppk_msg.altitude = stof(tempalt); // m
		     float sdn = std::stof(tempsdn)*10; // ppk covariances are too tight
	    	     float sde = std::stof(tempsde)*10;
	          float sdu = std::stof(tempsdu)*10;
               float sdne = std::stof(tempsdne)*10;
	    	     float sdeu = std::stof(tempsdeu)*10;
	          float sdun = std::stof(tempsdun)*10;
		     fix_ppk_msg.position_covariance = {sdn, sdne, sdun, sdne, sde, sdeu, sdun, sdeu, sdu};
		     fix_ppk_msg.position_covariance_type = 3;

		     

               // append to the ppk path and publish
               // visualization only
               globalEstimator.inputPPKviz(fix_ppk_msg.header.stamp.toSec(), stof(templat), stof(templon), stof(tempalt), sdn);

               //if(nmea_fix==5){   //5 is RTK
               //globalEstimator.inputPPKviz(fix_ppk_msg.header.stamp.toSec(), nmea_lat, nmea_lon, nmea_alt, sdn);}
		     
               if(use_ppk) {
			// include in the GPS q
               sensor_msgs::NavSatFixConstPtr fix_ppk_msg_const_pointer( new sensor_msgs::NavSatFix(fix_ppk_msg) );
               m_buf.lock();
    		     gpsQueue.push(fix_ppk_msg_const_pointer);
               m_buf.unlock();
		     }	
		} //synced routine		 		
	  } // ppk read loop
    }//check open file
    else std::cout << "Unable to open file1";
    cout<<"---------------------------"<<endl;
    }//check ppk processing flags

   if(use_frl || viz_frl || use_frl_rot){
    //open file
    if (myfile2.is_open())
    {
     	
 	bool ppk_synced = false;
	while (!ppk_synced)
	{
		//0. read the next line
		if (!skip_read){
		std::getline(myfile2, line);} 
		
    		//1. remove the header info in pos file
		if(line.at(0) == '%') continue;
		//cout<<line<<endl;
 
          //2. extract the position message line
          //2022/05/25 20:43:10.200   45.325064906  -75.664896317    86.7785   1  28   0.0035   0.0025   0.0064  -0.0005  -0.0015  -0.0021   0.01    7.9
          //2022/05/25 20:43:10.200   45.325070885095 	-75.664890525260  117.3329 68 33 0.1869  0.2636 	0.2647   0.0000  0.0000 0.0000 4.00  0.0 0.23  2.03 63.80 0.05 -0.10 -0.05	-0.01 0.02 0.00
		std::string tempdate,temptime,templat,templon,tempalt,tempQ,tempnumsat,tempsdn,tempsde,tempsdu,tempsdne,tempsdeu,tempsdun,tempage,tempratio;
          std::string temproll,temppitch,tempyaw,tempomgP,tempomgQ,tempomgR,tempVe,tempVn,tempVu;
          std::istringstream iss1(line);
    		iss1 >> tempdate >> temptime >> templat >> templon >> tempalt >> tempQ >> tempnumsat >> tempsdn >> tempsde >> tempsdu >> tempsdne >> tempsdeu >> tempsdun >> tempage >>  tempratio >> temproll >> temppitch >> tempyaw >> tempomgP >> tempomgQ >> tempomgR >> tempVe >> tempVn >> tempVu;
		
		//cout<<"ppk time string:"<<temptime<< " Nmea time: "<<nmea_time_formatted_to_string <<endl;//k
		// TODO:quick check of the UTC date to warn possible errors
		// convert ros time of nmea stamp to UTC and and check with PPK time
		
		// 3.1 Convert the two times to seconds
		//time value conversions
  		nmea_time_pval  =    boost::posix_time::hours(std::stod(nmea_time_formatted_to_string.substr(0,2))) +
				           boost::posix_time::minutes(std::stod(nmea_time_formatted_to_string.substr(3,2))) + 
			                boost::posix_time::seconds(std::stod(nmea_time_formatted_to_string.substr(6,2))) +
		                     boost::posix_time::millisec(std::stod(nmea_time_formatted_to_string.substr(9,2))*10);

		ppk_time_pval  =     boost::posix_time::hours(std::stod(temptime.substr(0,2))) +
				           boost::posix_time::minutes(std::stod(temptime.substr(3,2))) + 
			                boost::posix_time::seconds(std::stod(temptime.substr(6,2))-18-2.3) + //TODO:GPST to UTC correction: manualy found delay using matlab (1.25s)
		                     boost::posix_time::millisec(std::stod(temptime.substr(9,2))*10);
		boost::posix_time::time_duration delta = ppk_time_pval-nmea_time_pval;
		double diff = delta.total_microseconds();
		//cout<<"ppk time val:"<< ppk_time_pval << " Nmea time val: "<< nmea_time_pval << "|" << diff << endl;//k

		//3.keep reading the file if the nmea time is ahead
          if (diff<0){
		     skip_read = false;
		     ppk_synced = false;
                     //cout<<"NMEA ahead:"<< ppk_time_pval << " Nmea time val: "<< nmea_time_pval << "|" << diff << endl;
		     continue;
		}

		//4.do not read the next line if the ppk time is ahead
		if (diff>0){
		     skip_read = true;
		     ppk_synced = true; 
		     continue;
		}

          //5.if they match input the GPS data to the optimizer
          if (diff==0){
		     skip_read = false;
		     ppk_synced = true; 


               ros::Time nmea_ros_time=findRosTimeFromNmea(msg->header.stamp,nmea_time);
		     //cout<<"** SYNCED: frl time val:"<< ppk_time_pval << " Nmea time val: "<< nmea_time_pval << "|" << diff << "|" << nmea_lat << "|" << nmea_lon << "|" << nmea_alt << "|" <<  stof(templat) << "|" <<  stof(templon) << "|" <<  stof(tempalt) << endl;
		     //cout << nmea_ros_time << "|" << msg->header.stamp << endl;
  		     // Create a ppk GPS message
	          fix_ppk_msg.header = msg->header;
		     fix_ppk_msg.status.status = 2;
		     fix_ppk_msg.status.service = 1;
	          fix_ppk_msg.latitude = stof(templat); // deg
		     fix_ppk_msg.longitude = stof(templon); // deg
		     fix_ppk_msg.altitude = stof(tempalt); // m
		     float sdn = std::stof(tempsdn)*10; // ppk covariances are too tight
	    	     float sde = std::stof(tempsde)*10;
	          float sdu = std::stof(tempsdu)*10;
               float sdne = std::stof(tempsdne)*10;
	    	     float sdeu = std::stof(tempsdeu)*10;
	          float sdun = std::stof(tempsdun)*10;
		     fix_ppk_msg.position_covariance = {sdn, sdne, sdun, sdne, sde, sdeu, sdun, sdeu, sdu};
		     fix_ppk_msg.position_covariance_type = 3;

		     //create the quaternion msg
               rot_msg.header = msg->header;
			// convert euler to quaternion yusing eigen
			//Roll pitch and yaw in Radians
			//double roll = 1.5707, pitch = 0, yaw = 0.707; // k verified   
               //cout<<temproll<<temppitch<<tempyaw<<endl;
               double roll =stod(temproll)*M_PI / 180, pitch = stod(temppitch)*M_PI / 180, yaw = stod(tempyaw)*M_PI / 180; 
			Eigen::Quaterniond quat;
			quat = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(-M_PI/2.0, Eigen::Vector3d::UnitZ())  //to ENU
			* Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ())
    			* Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())
    			* Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
               rot_msg.quaternion.w = quat.w(); 
			rot_msg.quaternion.x = quat.x();
			rot_msg.quaternion.y = quat.y();
			rot_msg.quaternion.z = quat.z();
               //cout << quat.w()<<quat.x()<<quat.y()<<quat.z() << endl;
               //cin.get();


               // append to the ppk path and publish
               // visualization only
               //globalEstimator.inputPPKviz(fix_ppk_msg.header.stamp.toSec(), stof(templat), stof(templon), stof(tempalt), sdn);
			globalEstimator.inputFRLviz(fix_ppk_msg.header.stamp.toSec(), stof(templat), stof(templon), stof(tempalt),quat.w(),quat.x(),quat.y(),quat.z());

               //if(nmea_fix==5){   //5 is RTK
               //globalEstimator.inputPPKviz(fix_ppk_msg.header.stamp.toSec(), nmea_lat, nmea_lon, nmea_alt, sdn);}
		     
               if(use_frl) {
			// include in the GPS q
               sensor_msgs::NavSatFixConstPtr fix_ppk_msg_const_pointer( new sensor_msgs::NavSatFix(fix_ppk_msg) );
               m_buf.lock();
    		     gpsQueue.push(fix_ppk_msg_const_pointer);
               m_buf.unlock();
		     }

               if(use_frl_rot){
				geometry_msgs::QuaternionStampedConstPtr rot_msg_const_pointer(new geometry_msgs::QuaternionStamped(rot_msg));
				m_buf.lock();
	    		     rotQueue.push(rot_msg_const_pointer);
		          m_buf.unlock();			
			}	
		} //synced routine		 		
	  } // frl read loop
    }//check open file
    else std::cout << "Unable to open file2";
    cout<<"---------------------------"<<endl;
    }//check frl processing flags
   
}

void pointAssociateToMap(PointType const *const pi, PointType *const po,Eigen::Quaterniond q_w_curr,Eigen::Vector3d t_w_curr)
{
	Eigen::Vector3d point_curr(pi->x, pi->y, pi->z);
	Eigen::Vector3d point_w = q_w_curr * point_curr + t_w_curr;
	po->x = point_w.x();
	po->y = point_w.y();
	po->z = point_w.z();
	po->intensity = pi->intensity;
	//po->intensity = 1.0;
}

void laserCloudFullResHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudFullRes2)
{
  	if(!use_lidar){ return;}
	m_buf.lock();
	fullResBuf.push(laserCloudFullRes2);
	m_buf.unlock();


     // TODO:possible multiple access to resolve here
     //1.check the length of the global path
     //globalEstimator.mPoseMap.lock();
     timeLaserCloudFullRes = fullResBuf.front()->header.stamp.toSec();
     double last_update_time = globalEstimator.last_update_time;
     if (last_update_time == 0.0) return;
     //cout << global_path->poses.size() << endl; //global_path.header.stamp.toSec()

     
     //globalEstimator.mPoseMap.unlock();
     //cout<< global_path.header.stamp.toSec() <<endl;
     //2.check the latest time of global path -k-
     if (last_update_time-timeLaserCloudFullRes < 0.0) return;  // we have to wait
     //printf("*******vio t: %f, cloud t: %f diff: %f ***********\n", last_update_time , timeLaserCloudFullRes, last_update_time-timeLaserCloudFullRes);
	//3.iterate over global path 
     double synced_time;
     geometry_msgs::PoseStamped synced_pose;
     bool sync_found = false;
     for (int i =path_start_index; i<global_path->poses.size();i++){
		double vio_t = global_path->poses[i].header.stamp.toSec();
          //cout << vio_t - timeLaserCloudFullRes << endl;
          if(vio_t >= timeLaserCloudFullRes - 0.07 && vio_t <= timeLaserCloudFullRes + 0.07) 
          {
       
            printf("*******vio t: %f, cloud t: %f diff: %f ***********\n", vio_t, timeLaserCloudFullRes, vio_t-timeLaserCloudFullRes);        
            synced_time = vio_t;
            synced_pose = global_path->poses[i];
            sync_found = true;
            frameCount++;
            laserCloudFullRes->clear();
		  pcl::fromROSMsg(*fullResBuf.front(), *laserCloudFullRes);
            if (fullResBuf.size()>5){
            fullResBuf.pop();fullResBuf.pop(); //catch up
		  }
		  else{ fullResBuf.pop();}
            cout<< "pcl buff size"<<fullResBuf.size() << endl;
            path_start_index = i;
            break;
          }
		// if end is reached pop the cloud buffer
		if(i == global_path->poses.size()-1){
			fullResBuf.pop();fullResBuf.pop(); //catch up
			cout<< "scan deleted for time %f - no sync" << timeLaserCloudFullRes << endl;
		}
	}
     //4.find the matching pose
     Eigen::Quaterniond q_w_curr;
	Eigen::Vector3d t_w_curr;
     if (sync_found){
     	q_w_curr.x() = synced_pose.pose.orientation.x;
		q_w_curr.y() = synced_pose.pose.orientation.y;
		q_w_curr.z() = synced_pose.pose.orientation.z;
		q_w_curr.w() = synced_pose.pose.orientation.w;
		t_w_curr.x() = synced_pose.pose.position.x;
		t_w_curr.y() = synced_pose.pose.position.y;
		t_w_curr.z() = synced_pose.pose.position.z;
	
     //5.associate to map
     /*int laserCloudFullResNum = laserCloudFullRes->points.size();
     for (int i = 0; i < laserCloudFullResNum; i++)
	{
	   pointAssociateToMap(&laserCloudFullRes->points[i], &laserCloudFullRes->points[i], q_w_curr, t_w_curr);
	}*/
     Eigen::Matrix4d Tmat_IC;
     Tmat_IC.setIdentity();   // Set to Identity to make bottom row of Matrix 0,0,0,1
     Tmat_IC.block<3,3>(0,0) = RIC[0];
     Tmat_IC.block<3,1>(0,3) = TIC[0];

     Eigen::Matrix4d Tmat_CL;
     Tmat_CL.setIdentity();   // Set to Identity to make bottom row of Matrix 0,0,0,1
     Tmat_CL.block<3,3>(0,0) = RCL[0];
     Tmat_CL.block<3,1>(0,3) = TCL[0];
   

     Eigen::Affine3f transadjust = pcl::getTransformation(L_C_tx, L_C_ty, L_C_tz, L_C_rx, L_C_ry, L_C_rz);
     Eigen::Matrix4f Tmat_LLadjf = transadjust.matrix();
     Eigen::Matrix4d Tmat_LLadj = Tmat_LLadjf.cast<double>();
     
     Eigen::Affine3d aff = Eigen::Affine3d::Identity();
     aff.translation() = t_w_curr;
     aff.linear() = q_w_curr.normalized().toRotationMatrix();
     Eigen::Matrix4d Tmat_WB = aff.matrix();
   
     Eigen::Matrix4d Tmat_IL = Tmat_IC * Tmat_CL;
    
     Eigen::Matrix4d Tmat_WL = Tmat_WB * Tmat_IL;   // in LVI code this is done by keeping a body ros frame in tf tree

     Eigen::Matrix4d Tmat_WLadj = Tmat_WL * Tmat_LLadj;
      

     pcl::transformPointCloud(*laserCloudFullRes, *laserCloudFullRes, Tmat_WLadj.cast<float>());

     

  
	//6.down sample and add to map 
     pcl::PointCloud<PointType>::Ptr laserCloudFullDS(new pcl::PointCloud<PointType>());
	downSizeFilterFull.setInputCloud(laserCloudFullRes);
	downSizeFilterFull.filter(*laserCloudFullDS);

     if (!map_init){	
          laserCloudFullMap = *laserCloudFullDS;
          cloudBuf.push_back(*laserCloudFullDS);
          map_init =true;
     }
	else{
		laserCloudFullMap += *laserCloudFullDS;
          cloudBuf.push_back(*laserCloudFullDS);
          if (cloudBuf.size()>20){
            cloudBuf.pop_front(); //catch up
		}
	}
     // keep a buffer of the map  - can be optimized
     // pop the bufffer it exceeds length  
     
    
	//7.publish
     sensor_msgs::PointCloud2 laserCloudFullRes3;
	pcl::toROSMsg(*laserCloudFullRes, laserCloudFullRes3);
	laserCloudFullRes3.header.stamp = ros::Time().fromSec(synced_time);
	laserCloudFullRes3.header.frame_id = "/worldGPS";
     pubLaserCloudFullRes.publish(laserCloudFullRes3);
     
      if(use_lz_seg && frameCount % 3 == 0){
          //combine the scans in buffer
          pcl::PointCloud<pcl::PointXYZI>::Ptr cloudPTR(new pcl::PointCloud<pcl::PointXYZI>);
          for (int i=0; i < cloudBuf.size(); i++) {
    			*cloudPTR += cloudBuf[i];
		}
	     
   
          // segmentation of current scan
          cloudRGB =  classicalSegmentCloud(cloudPTR);
          cout << "Segmented map size : " << cloudRGB->points.size() << "| Frame: " << frameCount << endl ; 


		//append the map - TODO: for efficiency use a channel of the same published map
          
          sensor_msgs::PointCloud2 surrooundCloudMsg;
		pcl::toROSMsg(*cloudRGB, surrooundCloudMsg);
		surrooundCloudMsg.header.stamp = ros::Time().fromSec(synced_time);
		surrooundCloudMsg.header.frame_id = "/worldGPS";
     	pubLaserCloudMapSurround.publish(surrooundCloudMsg);}
          //publish    
     }


	//8. publish map
     if (frameCount % 10 == 0)
		{		sensor_msgs::PointCloud2 laserCloudMsg;
				pcl::toROSMsg(laserCloudFullMap, laserCloudMsg);
				laserCloudMsg.header.stamp = ros::Time().fromSec(synced_time);
				laserCloudMsg.header.frame_id = "/worldGPS";
     			pubLaserCloudMap.publish(laserCloudMsg);
               	cout << "Published map size : " << laserCloudFullMap.points.size() << "| Frame: " << frameCount << endl ;
				//publish aft mapped path
                    
                    
		  


	//9. Publish path
     nav_msgs::Odometry odomAftMapped;
     odomAftMapped.header.frame_id = "/worldGPS";
     odomAftMapped.child_frame_id = "/aft_mapped";
	odomAftMapped.header.stamp = ros::Time().fromSec(synced_time);
	odomAftMapped.pose.pose.orientation.x = q_w_curr.x();
	odomAftMapped.pose.pose.orientation.y = q_w_curr.y();
	odomAftMapped.pose.pose.orientation.z = q_w_curr.z();
	odomAftMapped.pose.pose.orientation.w = q_w_curr.w();
	odomAftMapped.pose.pose.position.x = t_w_curr.x();
	odomAftMapped.pose.pose.position.y = t_w_curr.y();
	odomAftMapped.pose.pose.position.z = t_w_curr.z();
	pubOdomAftMapped.publish(odomAftMapped);

	geometry_msgs::PoseStamped laserAfterMappedPose;
	laserAfterMappedPose.header = odomAftMapped.header;
	laserAfterMappedPose.pose = odomAftMapped.pose.pose;
	laserAfterMappedPath.header.stamp = odomAftMapped.header.stamp;
	laserAfterMappedPath.header.frame_id = "/worldGPS";
	laserAfterMappedPath.poses.push_back(laserAfterMappedPose);
	pubLaserAfterMappedPath.publish(laserAfterMappedPath);

            
        


	}



	//m_buf.lock();
	//fullResBuf.push(laserCloudFullRes2);
	//m_buf.unlock();

     // no need to buffer
     // find the time in the vio que
    /* double cloud_t = laserCloudFullRes2->header.stamp.toSec();
     while(!vioQueue.empty())
     {
        nav_msgs::Odometry::ConstPtr pose_msg_first = vioQueue.front();
        double vio_t = pose_msg_first->header.stamp.toSec();
        
        
        if(vio_t >= cloud_t - 0.05 && vio_t <= cloud_t + 0.05) 
        {
       
            printf("*******vio t: %f, cloud t: %f diff: %f ***********\n", vio_t, cloud_t, vio_t-cloud_t);        
            timeLaserCloudFullRes = vio_t;
            laserCloudFullRes->clear();
		  pcl::fromROSMsg(*laserCloudFullRes2, *laserCloudFullRes);
            globalEstimator.inputCloudFullRes(vio_t, laserCloudFullRes);
            vioQueue.pop();
            break;
        }
        else if(vio_t < cloud_t - 0.05)
            vioQueue.pop();
        else if(vio_t > cloud_t + 0.05){
            printf("*******NOT SYNCED vio t: %f, cloud t: %f \n", vio_t, cloud_t);
            break;}
    }*/
     // and clear the que 
     


}




        



int main(int argc, char **argv)
{
    ros::init(argc, argv, "globalEstimator");
    ros::NodeHandle n("~");

    if(use_lidar){
    if(argc != 2)
    {
        printf("please intput: rosrun vins vins_node [config file] \n"
               "for example: rosrun vins vins_node "
               "~/catkin_ws/src/VINS-Fusion/config/euroc/euroc_stereo_imu_config.yaml \n");
        return 1;
    }

    string config_file = argv[1];
    printf("config_file: %s\n", argv[1]);

    readParameters(config_file);}
    cloudRGB.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
    

    global_path = &globalEstimator.global_path;
    gps_path = &globalEstimator.gps_path;
    ppk_path = &globalEstimator.ppk_path;
    frl_path = &globalEstimator.frl_path;
    //laserCloudFullRes2->clear();
    //*laserCloudFullRes2= *globalEstimator.laserCloudFullRes2;

   
    ros::Subscriber sub_nmea = n.subscribe("/nmea_sentence", 100, nmeaCallback);
    ros::Subscriber sub_GPS = n.subscribe("/fix", 100, GPS_callback);
    ros::Subscriber sub_pr_fix = n.subscribe("/pr_fix", 100, PR_GPS_callback);
    ros::Subscriber sub_vio = n.subscribe("/vins_estimator/odometry", 100, vio_callback);
    ros::Subscriber sub_imu = n.subscribe("/imu/mag", 100, mag_callback);
    
    pub_global_path = n.advertise<nav_msgs::Path>("global_path", 100);
    pub_gps_path = n.advertise<nav_msgs::Path>("gps_path", 100);
    pub_ppk_path = n.advertise<nav_msgs::Path>("ppk_path", 100);
    pub_frl_path = n.advertise<nav_msgs::Path>("frl_path", 100);
    //if(use_lidar){
    		ros::Subscriber subLaserCloudFullRes = n.subscribe("/velodyne_cloud_3", 100, laserCloudFullResHandler);
    		pubLaserCloudFullRes = n.advertise<sensor_msgs::PointCloud2>("/velodyne_cloud_registered", 100);
    		pubLaserCloudMap = n.advertise<sensor_msgs::PointCloud2>("/laser_cloud_map", 100);
          pubLaserCloudMapSurround = n.advertise<sensor_msgs::PointCloud2>("/laser_cloud_map_surround", 100);
          pubLaserAfterMappedPath = n.advertise<nav_msgs::Path>("/aft_mapped_path", 100);
          pubOdomAftMapped = n.advertise<nav_msgs::Odometry>("/aft_mapped_to_init", 100);
		downSizeFilterFull.setLeafSize(0.1, 0.1,0.1);
	//}
    pub_global_odometry = n.advertise<nav_msgs::Odometry>("global_odometry", 100);
    pub_car = n.advertise<visualization_msgs::MarkerArray>("car_model", 100);
    ros::spin();
    return 0;
}
