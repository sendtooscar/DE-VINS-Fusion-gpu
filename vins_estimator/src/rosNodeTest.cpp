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

#include <stdio.h>
#include <queue>
#include <map>
#include <thread>
#include <mutex>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include "estimator/estimator.h"
#include "estimator/parameters.h"
#include "utility/visualization.h"


//this class is initialized later
Estimator *estimator;


queue<sensor_msgs::ImuConstPtr> imu_buf;
queue<sensor_msgs::PointCloudConstPtr> feature_buf;
queue<sensor_msgs::ImageConstPtr> img0_buf;
queue<sensor_msgs::ImageConstPtr> img1_buf;
std::queue<sensor_msgs::PointCloud2ConstPtr> fullPointsBuf;
std::mutex m_buf;// visual inertial processing mutex lock

// mtx lock for two areas
std::mutex mtx_lidar; //mtx_lidar is for lidar processing

// global variable for saving the depthCloud shared between two threads
pcl::PointCloud<PointType>::Ptr depthCloud(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr depthCloudLocal(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr laserCloudFullRes(new pcl::PointCloud<PointType>());
double timeLaserCloudFullRes = 0;
int skipFrameNum = 5;
bool systemInited = false;

// global variables saving the lidar point cloud
deque<pcl::PointCloud<PointType>> cloudQueue;
deque<double> timeQueue;
ros::Publisher pub_pcl;
ros::Publisher pubLaserCloudFullRes; 

// global depth register for obtaining depth of a feature
DepthRegister *depthRegister;

void img0_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
    m_buf.lock();
    img0_buf.push(img_msg);
    m_buf.unlock();
}

void img1_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
    m_buf.lock();
    img1_buf.push(img_msg);
    m_buf.unlock();
}


void lidar_callback(const sensor_msgs::PointCloud2ConstPtr& laser_msg)
{
 if(USE_LIDAR){
 static int lidar_count = -1;
    if (++lidar_count % (LIDAR_SKIP+1) != 0)
        return;
  
   // 0. listen to transform
    static tf::TransformListener listener;
    static tf::StampedTransform transform;
    try{
        listener.waitForTransform("world", "body_fast", laser_msg->header.stamp, ros::Duration(0.1));
        listener.lookupTransform("world", "body_fast", ros::Time(0), transform);
        //listener.lookupTransform("world", "body_fast", laser_msg->header.stamp, transform);
        std::cout << "sync_diff"  << laser_msg->header.stamp - transform.stamp_ << std::endl; //k low sync errors
    } 
    catch (tf::TransformException ex){
        /*std::cout << std::endl;
        std::cout << "lidar no tf" << laser_msg->header.stamp  << std::endl;
        std::cout << "lidar no tf" << laser_msg->header.stamp - ros::Time(0)  << std::endl;*/
        ROS_ERROR("lidar no tf");
        return;
    }


    double xCur, yCur, zCur, rollCur, pitchCur, yawCur;
    xCur = transform.getOrigin().x();
    yCur = transform.getOrigin().y();
    zCur = transform.getOrigin().z();
    tf::Matrix3x3 m(transform.getRotation());
    m.getRPY(rollCur, pitchCur, yawCur);
    Eigen::Affine3f transNow = pcl::getTransformation(xCur, yCur, zCur, rollCur, pitchCur, yawCur);
    
   


    // 1. convert laser cloud message to pcl
    pcl::PointCloud<PointType>::Ptr laser_cloud_in(new pcl::PointCloud<PointType>());
    pcl::fromROSMsg(*laser_msg, *laser_cloud_in);

    // 2. downsample new cloud (save memory)
    pcl::PointCloud<PointType>::Ptr laser_cloud_in_ds(new pcl::PointCloud<PointType>());
    static pcl::VoxelGrid<PointType> downSizeFilter;
    downSizeFilter.setLeafSize(0.2, 0.2, 0.2);
    downSizeFilter.setInputCloud(laser_cloud_in);
    downSizeFilter.filter(*laser_cloud_in_ds);
    *laser_cloud_in = *laser_cloud_in_ds;

    // 3. filter lidar points (only keep points in camera view)
    /*pcl::PointCloud<PointType>::Ptr laser_cloud_in_filter(new pcl::PointCloud<PointType>());
    for (int i = 0; i < (int)laser_cloud_in->size(); ++i)
    {
        PointType p = laser_cloud_in->points[i];
        if (p.x >= 0 && abs(p.y / p.x) <= 10 && abs(p.z / p.x) <= 10)
            laser_cloud_in_filter->push_back(p);
    }
    *laser_cloud_in = *laser_cloud_in_filter;*/

    // TODO: transform to IMU body frame
    // 4. offset T_lidar -> T_camera 
    pcl::PointCloud<PointType>::Ptr laser_cloud_offset(new pcl::PointCloud<PointType>());
    Eigen::Affine3f transOffset = pcl::getTransformation(L_C_tx, L_C_ty, L_C_tz, L_C_rx, L_C_ry, L_C_rz); // this is only fine tuning adjustment 
    cout << transOffset.matrix() <<endl;
    pcl::transformPointCloud(*laser_cloud_in, *laser_cloud_offset, transOffset);
    *laser_cloud_in = *laser_cloud_offset;

    

    // 5. transform new cloud into global odom frame
    pcl::PointCloud<PointType>::Ptr laser_cloud_global(new pcl::PointCloud<PointType>());
    //trans now is VW_T_B of vins we want VW_T_L = VW_T_B * B_T_C * C_T_L
    // then the point cloud is correctly transformed to the VW frame
    Eigen::Matrix4d Tmat_IC;
    Tmat_IC.setIdentity();   // Set to Identity to make bottom row of Matrix 0,0,0,1
    Tmat_IC.block<3,3>(0,0) = RIC[0];
    Tmat_IC.block<3,1>(0,3) = TIC[0];

    Eigen::Matrix4d Tmat_CL;
    Tmat_CL.setIdentity();   // Set to Identity to make bottom row of Matrix 0,0,0,1
    Tmat_CL.block<3,3>(0,0) = RCL[0];
    Tmat_CL.block<3,1>(0,3) = TCL[0];
   
    
    Eigen::Matrix4f Tmat_WBf = transNow.matrix();

    Eigen::Matrix4d Tmat_WB = Tmat_WBf.cast<double>();
   
    Eigen::Matrix4d Tmat_IL = Tmat_IC * Tmat_CL;
    
    Eigen::Matrix4d Tmat_WL = Tmat_WB * Tmat_IL;   // in LVI code this is done by keeping a body ros frame in tf tree
    
    transNow = Tmat_WL.cast<float>(); //converts to affine 3f

    pcl::transformPointCloud(*laser_cloud_in, *laser_cloud_global, transNow);

    // 6. save new cloud
    double timeScanCur = laser_msg->header.stamp.toSec();
    cloudQueue.push_back(*laser_cloud_global);
    timeQueue.push_back(timeScanCur);
    *depthCloudLocal = *laser_cloud_in;

    // 7. pop old cloud
    while (!timeQueue.empty())
    {
        if (timeScanCur - timeQueue.front() > 5.0)
        {
            cloudQueue.pop_front();
            timeQueue.pop_front();
        } else {
            break;
        }
    }
    
    std::lock_guard<std::mutex> lock(mtx_lidar);
    
    // 8. fuse global cloud
    depthCloud->clear();
    if (USE_DENSE_CLOUD == 0){
    	*depthCloud += cloudQueue.back();
    }else {
    	for (int i = 0; i < (int)cloudQueue.size(); ++i)
        	*depthCloud += cloudQueue[i];
    }

    // 9. downsample global cloud
    pcl::PointCloud<PointType>::Ptr depthCloudDS(new pcl::PointCloud<PointType>());
    downSizeFilter.setLeafSize(0.2, 0.2, 0.2);
    downSizeFilter.setInputCloud(depthCloud);
    downSizeFilter.filter(*depthCloudDS);
    *depthCloud = *depthCloudDS;

   //10. visualization for debugging
    if(true){
    if (pub_pcl.getNumSubscribers() == 0)
        return;
    sensor_msgs::PointCloud2 tempCloud;
    pcl::toROSMsg(*depthCloud, tempCloud);
    tempCloud.header.frame_id = "world";
    pub_pcl.publish(tempCloud);
    
    }
 }
}


cv::Mat getImageFromMsg(const sensor_msgs::ImageConstPtr &img_msg)
{
    cv_bridge::CvImageConstPtr ptr;
    if (img_msg->encoding == "8UC1")
    {
        sensor_msgs::Image img;
        img.header = img_msg->header;
        img.height = img_msg->height;
        img.width = img_msg->width;
        img.is_bigendian = img_msg->is_bigendian;
        img.step = img_msg->step;
        img.data = img_msg->data;
        img.encoding = "mono8";
        ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
    }
    else
        ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);

    cv::Mat img = ptr->image.clone();
    return img;
}

// extract images with same timestamp from two topics
void sync_process()
{
    while(1)
    {
        if(STEREO)
        {
            cv::Mat image0, image1;
            std_msgs::Header header;
            double time = 0;
            m_buf.lock();
            if (!img0_buf.empty() && !img1_buf.empty())
            {
                double time0 = img0_buf.front()->header.stamp.toSec();
                double time1 = img1_buf.front()->header.stamp.toSec();
                if(time0 < time1)
                {
                    img0_buf.pop();
                    printf("throw img0\n");
                }
                else if(time0 > time1)
                {
                    img1_buf.pop();
                    printf("throw img1\n");
                }
                else
                {
                    time = img0_buf.front()->header.stamp.toSec();
                    header = img0_buf.front()->header;
                    image0 = getImageFromMsg(img0_buf.front());
                    img0_buf.pop();
                    image1 = getImageFromMsg(img1_buf.front());
                    img1_buf.pop();
                    //printf("find img0 and img1\n");
                }
            }
            m_buf.unlock();
            if(!image0.empty())
                estimator->inputImage(time, image0, image1);
        }
        else
        {
            cv::Mat image;
            std_msgs::Header header;
            double time = 0;
            double cloud_time = 0;
            m_buf.lock();
            if(!img0_buf.empty())
            {
                time = img0_buf.front()->header.stamp.toSec();
                header = img0_buf.front()->header;
                image = getImageFromMsg(img0_buf.front());
                img0_buf.pop();
            }
            m_buf.unlock();

            // get the lidar cloud from the thread
            pcl::PointCloud<PointType>::Ptr depth_cloud_temp(new pcl::PointCloud<PointType>());
            mtx_lidar.lock();
	    *depth_cloud_temp = *depthCloud;
            //std::cout << "No depth cloud" <<depth_cloud_temp->size()<< std::endl;//ok
            if (depth_cloud_temp->size()>0){
       	    	cloud_time = timeQueue.back();
            }
            //std::cout << "Depth cloud sync time" << time << std::endl;
	       mtx_lidar.unlock();

            // check time difference between lidar and image (0.5s limit)
           // std::cout << "No depth cloud  :" <<!depth_cloud_temp<< std::endl;
                    
           if (depth_cloud_temp->size()==0){
 	        //std::cout << "No depth cloud"<<depth_cloud_temp->size() << std::endl;
		}
	   else {
                //std::cout << "cloud time" <<  cloud_time << std::endl;
               // std::cout << "image time" <<  time << std::endl;
		//std::cout << "Depth cloud sync time" << time - cloud_time << std::endl;
            }

            // call the feature tracker mono or depth enhanced       
            if(!image.empty())
                estimator->inputImage(time, image);
        }

        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
    }
}


void imu_callback(const sensor_msgs::ImuConstPtr &imu_msg)
{
    double t = imu_msg->header.stamp.toSec();
    double dx = imu_msg->linear_acceleration.x;
    double dy = imu_msg->linear_acceleration.y;
    double dz = imu_msg->linear_acceleration.z;
    double rx = imu_msg->angular_velocity.x;
    double ry = imu_msg->angular_velocity.y;
    double rz = imu_msg->angular_velocity.z;
    Vector3d acc(dx, dy, dz);
    Vector3d gyr(rx, ry, rz);
    estimator->inputIMU(t, acc, gyr);
    return;
}


void feature_callback(const sensor_msgs::PointCloudConstPtr &feature_msg)
{
    map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> featureFrame;
    for (unsigned int i = 0; i < feature_msg->points.size(); i++)
    {
        int feature_id = feature_msg->channels[0].values[i];
        int camera_id = feature_msg->channels[1].values[i];
        double x = feature_msg->points[i].x;
        double y = feature_msg->points[i].y;
        double z = feature_msg->points[i].z;
        double p_u = feature_msg->channels[2].values[i];
        double p_v = feature_msg->channels[3].values[i];
        double velocity_x = feature_msg->channels[4].values[i];
        double velocity_y = feature_msg->channels[5].values[i];
        if(feature_msg->channels.size() > 5)
        {
            double gx = feature_msg->channels[6].values[i];
            double gy = feature_msg->channels[7].values[i];
            double gz = feature_msg->channels[8].values[i];
            pts_gt[feature_id] = Eigen::Vector3d(gx, gy, gz);
            //printf("receive pts gt %d %f %f %f\n", feature_id, gx, gy, gz);
        }
        ROS_ASSERT(z == 1);
        Eigen::Matrix<double, 7, 1> xyz_uv_velocity;
        xyz_uv_velocity << x, y, z, p_u, p_v, velocity_x, velocity_y;
        featureFrame[feature_id].emplace_back(camera_id,  xyz_uv_velocity);
    }
    //inject depth
    
    double t = feature_msg->header.stamp.toSec();
    estimator->inputFeature(t, featureFrame);
    return;
}

void restart_callback(const std_msgs::BoolConstPtr &restart_msg)
{
    if (restart_msg->data == true)
    {
        ROS_WARN("restart the estimator!");
        m_buf.lock();
        while(!feature_buf.empty())
            feature_buf.pop();
        while(!imu_buf.empty())
            imu_buf.pop();
        m_buf.unlock();
        estimator->clearState();
        estimator->setParameter();
    }
    return;
}


void sync_process_lidar(){
     int frameCount = 0;
	while(1){
         
		
		if (!fullPointsBuf.empty()){
          TicToc t_process;
                mtx_lidar.lock();
          	timeLaserCloudFullRes = fullPointsBuf.front()->header.stamp.toSec();
          	laserCloudFullRes->clear();
          	pcl::fromROSMsg(*fullPointsBuf.front(), *laserCloudFullRes);
          	fullPointsBuf.pop();
          	mtx_lidar.unlock();
          
          if (frameCount % skipFrameNum == 0)
            {
			 std::cout<< "pub lidar: " << frameCount << endl;
                frameCount = 0;
			 sensor_msgs::PointCloud2 laserCloudFullRes3;
                pcl::toROSMsg(*laserCloudFullRes, laserCloudFullRes3);
                laserCloudFullRes3.header.stamp = ros::Time().fromSec(timeLaserCloudFullRes);
                laserCloudFullRes3.header.frame_id = "/velodyne";
                pubLaserCloudFullRes.publish(laserCloudFullRes3);
                
		  }
           frameCount++;
		 printf("process measurement time lidar: %f\n", t_process.toc());
          }
          //std::cout << "sync_diff"  << laserCloudFullRes->header.stamp.toSec() - estimator->latest_time  << std::endl;
		std::chrono::milliseconds dura(2);
          std::this_thread::sleep_for(dura);
    }
}

//receive all point cloud
void laserCloudFullResHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudFullRes2)
{
    mtx_lidar.lock();
    fullPointsBuf.push(laserCloudFullRes2);
    mtx_lidar.unlock();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vins_estimator");
    ros::NodeHandle n("~");
 
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);

    if(argc != 2)
    {
        printf("please intput: rosrun vins vins_node [config file] \n"
               "for example: rosrun vins vins_node "
               "~/catkin_ws/src/VINS-Fusion/config/euroc/euroc_stereo_imu_config.yaml \n");
        return 1;
    }

    string config_file = argv[1];
    printf("config_file: %s\n", argv[1]);

    readParameters(config_file);


    estimator = new Estimator(&n);
    estimator->setParameter();

#ifdef EIGEN_DONT_PARALLELIZE
    ROS_DEBUG("EIGEN_DONT_PARALLELIZE");
#endif

    ROS_WARN("waiting for image and imu...");

    registerPub(n);

    ros::Subscriber sub_imu = n.subscribe(IMU_TOPIC, 2000, imu_callback, ros::TransportHints().tcpNoDelay());
    ros::Subscriber sub_lidar = n.subscribe(POINT_CLOUD_TOPIC, 5,    lidar_callback);
    ros::Subscriber subLaserCloudFullRes = n.subscribe<sensor_msgs::PointCloud2>("/velodyne_cloud_2", 100, laserCloudFullResHandler);
    ros::Subscriber sub_feature = n.subscribe("/feature_tracker/feature", 2000, feature_callback);
    ros::Subscriber sub_img0 = n.subscribe(IMAGE0_TOPIC, 100, img0_callback);
    ros::Subscriber sub_img1 = n.subscribe(IMAGE1_TOPIC, 100, img1_callback);
    pub_pcl = n.advertise<sensor_msgs::PointCloud2> ("debug_cloud", 1);
    pubLaserCloudFullRes = n.advertise<sensor_msgs::PointCloud2>("/velodyne_cloud_3", 100);

    std::thread sync_thread{sync_process}; // sync_process() runs in a seperate thread 
    std::thread sync_thread2{sync_process_lidar}; // sync_process_lidar() runs in a seperate thread 
    
    ros::spin();

    return 0;
}
