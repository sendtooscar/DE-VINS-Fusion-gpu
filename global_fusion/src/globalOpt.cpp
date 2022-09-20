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

#include "globalOpt.h"
#include "Factors.h"
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
# define PI  3.141592653589793238462643383279502884L /* pi */

GlobalOptimization::GlobalOptimization():
outfileOdom("resultsOdom.txt", std::ios_base::trunc),
outfileGt("resultsGt.txt", std::ios_base::trunc)
{
    initGPS = false;
    newGPS = false;
    WGPS_T_WVIO = Eigen::Matrix4d::Identity();
    WGPS_T_WVIO_viz = Eigen::Matrix4d::Identity();
    update_count =0;
    GTframeCount = 0;
    threadOpt = std::thread(&GlobalOptimization::optimize, this);
    
}

GlobalOptimization::~GlobalOptimization()
{
    threadOpt.detach();
}

void GlobalOptimization::GPS2XYZ(double latitude, double longitude, double altitude, double* xyz)
{
    if(!initGPS)
    {
        geoConverter.Reset(latitude, longitude, altitude);
        initGPS = true;
    }
    geoConverter.Forward(latitude, longitude, altitude, xyz[0], xyz[1], xyz[2]);
    //printf("la: %f lo: %f al: %f\n", latitude, longitude, altitude);
    //printf("gps x: %f y: %f z: %f\n", xyz[0], xyz[1], xyz[2]);
}

void GlobalOptimization::inputOdom(double t, Eigen::Vector3d OdomP, Eigen::Quaterniond OdomQ)
{
	mPoseMap.lock();
    vector<double> localPose{OdomP.x(), OdomP.y(), OdomP.z(), 
    					     OdomQ.w(), OdomQ.x(), OdomQ.y(), OdomQ.z()};
    localPoseMap[t] = localPose;


    Eigen::Quaterniond globalQ;
    globalQ = WGPS_T_WVIO.block<3, 3>(0, 0) * OdomQ;
    Eigen::Vector3d globalP = WGPS_T_WVIO.block<3, 3>(0, 0) * OdomP + WGPS_T_WVIO.block<3, 1>(0, 3);
    vector<double> globalPose{globalP.x(), globalP.y(), globalP.z(),
                              globalQ.w(), globalQ.x(), globalQ.y(), globalQ.z()};
    globalPoseMap[t] = globalPose;
    lastP = globalP;
    lastQ = globalQ;

    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.stamp = ros::Time(t);
    pose_stamped.header.frame_id = "worldGPS";
    pose_stamped.pose.position.x = lastP.x();
    pose_stamped.pose.position.y = lastP.y();
    pose_stamped.pose.position.z = lastP.z();
    pose_stamped.pose.orientation.x = lastQ.x();
    pose_stamped.pose.orientation.y = lastQ.y();
    pose_stamped.pose.orientation.z = lastQ.z();
    pose_stamped.pose.orientation.w = lastQ.w();
    global_path.header = pose_stamped.header;
    global_path.poses.push_back(pose_stamped);
    gps_path.header = pose_stamped.header; 
    ppk_path.header = pose_stamped.header;


    //Publish the worldGPS frame (only perform 100 updates and stop)
    /*if (update_count <100){
        WGPS_T_WVIO_viz = WGPS_T_WVIO; 
    	update_count++;
        if (update_count ==100){
          printf("*********************WGPS_T_WVIO_viz fixed*********************\n");
        }     
     }*/

    // manual overide of the orientation 
    double angle = 63.8;// from mag for bell dataset 1
    WGPS_T_WVIO_viz << cos(angle*PI/180), -sin(angle*PI/180), 0, 0,
                       sin(angle*PI/180), cos(angle*PI/180), 0, 0,
                       0, 0, 1, 0,
                       0, 0, 0, 1;

    //WGPS_T_WVIO_viz = WGPS_T_WVIO;

    // initialize using compass
    // wait for compass
    //get ref mag heading
    //get current mag heading 
    // set the heading

 
    static tf2_ros::TransformBroadcaster brOpGPS;
    geometry_msgs::TransformStamped transformStampedG;
    transformStampedG.header.stamp = ros::Time(t);
    transformStampedG.header.frame_id = "worldGPS";    //reference frame
    transformStampedG.child_frame_id = "world";
    transformStampedG.transform.translation.x = WGPS_T_WVIO_viz(0,3); //read & send the pos
    transformStampedG.transform.translation.y = WGPS_T_WVIO_viz(1,3);
    transformStampedG.transform.translation.z = WGPS_T_WVIO_viz(2,3);

    Eigen::Quaterniond q_upTemp;
    q_upTemp = Eigen::Quaterniond(WGPS_T_WVIO_viz.block<3, 3>(0, 0));
    transformStampedG.transform.rotation.x = q_upTemp.x();
    transformStampedG.transform.rotation.y = q_upTemp.y();
    transformStampedG.transform.rotation.z = q_upTemp.z();
    transformStampedG.transform.rotation.w = q_upTemp.w();

    //static_broadcaster.sendTransform(static_transformStamped);
    brOpGPS.sendTransform(transformStampedG);


    

    mPoseMap.unlock();
}

void GlobalOptimization::getGlobalOdom(Eigen::Vector3d &odomP, Eigen::Quaterniond &odomQ)
{
    odomP = lastP;
    odomQ = lastQ;
}

void GlobalOptimization::inputGPS(double t, double latitude, double longitude, double altitude, double posAccuracy)
{
	double xyz[3];
	GPS2XYZ(latitude, longitude, altitude, xyz);
	vector<double> tmp{xyz[0], xyz[1], xyz[2], posAccuracy};
	GPSPositionMap[t] = tmp;
    newGPS = true;

}

void GlobalOptimization::inputRot(double t, double q_w, double q_x, double q_y, double q_z, double rotAccuracy)
{
	vector<double> tmp{q_w, q_x, q_y, q_z, rotAccuracy};
	globalRotMap[t] = tmp;
     newRot = true;

}

void GlobalOptimization::inputMag(double t, double mag_x, double mag_y, double mag_z, double magAccuracy)
{
	vector<double> tmp{mag_x, mag_y, mag_z, magAccuracy};
	magMap[t] = tmp;
     newMag = true;

}

void GlobalOptimization::inputPPKviz(double t, double latitude, double longitude, double altitude, double posAccuracy)
{
	double xyz[3];
	GPS2XYZ(latitude, longitude, altitude, xyz);
	vector<double> tmp{xyz[0], xyz[1], xyz[2], posAccuracy};
	PPKPositionMap[t] = tmp;

}

void GlobalOptimization::optimize()
{
    while(true)
    {
        if(newGPS || newRot)
        {
            newGPS = false;
            newRot = false;
            printf("global optimization\n");
            TicToc globalOptimizationTime;

            ceres::Problem problem;
            ceres::Solver::Options options;
            options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
            //options.minimizer_progress_to_stdout = true;
            //options.max_solver_time_in_seconds = SOLVER_TIME * 3;
            options.max_num_iterations = 5;
            ceres::Solver::Summary summary;
            ceres::LossFunction *loss_function;
            loss_function = new ceres::HuberLoss(1.0);
            ceres::LocalParameterization* local_parameterization = new ceres::QuaternionParameterization();

            //add param
            mPoseMap.lock();
            int length = localPoseMap.size();
            // w^t_i   w^q_i
            double t_array[length][3];
            double q_array[length][4];
            map<double, vector<double>>::iterator iter;
            iter = globalPoseMap.begin();
            for (int i = 0; i < length; i++, iter++)
            {
                t_array[i][0] = iter->second[0];
                t_array[i][1] = iter->second[1];
                t_array[i][2] = iter->second[2];
                q_array[i][0] = iter->second[3];
                q_array[i][1] = iter->second[4];
                q_array[i][2] = iter->second[5];
                q_array[i][3] = iter->second[6];
                problem.AddParameterBlock(q_array[i], 4, local_parameterization);
                problem.AddParameterBlock(t_array[i], 3);
            }

            map<double, vector<double>>::iterator iterVIO, iterVIONext, iterGPS, iterRot, iterMag;
            int i = 0;
            for (iterVIO = localPoseMap.begin(); iterVIO != localPoseMap.end(); iterVIO++, i++)
            {
                //vio factor
                iterVIONext = iterVIO;
                iterVIONext++;
                if(iterVIONext != localPoseMap.end())
                {
                    Eigen::Matrix4d wTi = Eigen::Matrix4d::Identity();
                    Eigen::Matrix4d wTj = Eigen::Matrix4d::Identity();
                    wTi.block<3, 3>(0, 0) = Eigen::Quaterniond(iterVIO->second[3], iterVIO->second[4], 
                                                               iterVIO->second[5], iterVIO->second[6]).toRotationMatrix();
                    wTi.block<3, 1>(0, 3) = Eigen::Vector3d(iterVIO->second[0], iterVIO->second[1], iterVIO->second[2]);
                    wTj.block<3, 3>(0, 0) = Eigen::Quaterniond(iterVIONext->second[3], iterVIONext->second[4], 
                                                               iterVIONext->second[5], iterVIONext->second[6]).toRotationMatrix();
                    wTj.block<3, 1>(0, 3) = Eigen::Vector3d(iterVIONext->second[0], iterVIONext->second[1], iterVIONext->second[2]);
                    Eigen::Matrix4d iTj = wTi.inverse() * wTj;
                    Eigen::Quaterniond iQj;
                    iQj = iTj.block<3, 3>(0, 0);
                    Eigen::Vector3d iPj = iTj.block<3, 1>(0, 3);

                    ceres::CostFunction* vio_function = RelativeRTError::Create(iPj.x(), iPj.y(), iPj.z(),
                                                                                iQj.w(), iQj.x(), iQj.y(), iQj.z(),
                                                                                0.1, 0.01);
                    problem.AddResidualBlock(vio_function, NULL, q_array[i], t_array[i], q_array[i+1], t_array[i+1]);
                }
                //gps factor
                double t = iterVIO->first;// TODO: check if this should be iterVIONext  
                iterGPS = GPSPositionMap.find(t);
                if (iterGPS != GPSPositionMap.end())
                {
                    ceres::CostFunction* gps_function = TError::Create(iterGPS->second[0], iterGPS->second[1], 
                                                                       iterGPS->second[2], iterGPS->second[3]);
                    //printf("inverse weight %f \n", iterGPS->second[3]);
                    problem.AddResidualBlock(gps_function, loss_function, t_array[i]);

                }
                // O- Rot factor (FULL AHRS INPUT) --k
                /*iterRot = globalRotMap.find(t);
                if (iterRot != globalRotMap.end())
                {
                    ceres::CostFunction* rot_function = RError::Create(iterRot->second[0], iterRot->second[1], 
                                                                       iterRot->second[2], iterRot->second[3],0.01);
                    //printf("inverse weight %f \n", iterGPS->second[3]);
                    problem.AddResidualBlock(rot_function, loss_function, q_array[i]);

                }*/

			// O- Rot factor (Yaw INPUT) - k possible lag ~1-2 second
               /*iterRot = globalRotMap.find(t);
               if (iterRot != globalRotMap.end())
               {
				// take the quaternion
				double w_q_i[4] = {iterRot->second[0], iterRot->second[1], iterRot->second[2], iterRot->second[3]};
				// convert to yaw
                    double siny_cosp = 2 * (w_q_i[0] * w_q_i[3] + w_q_i[1] * w_q_i[2]);
    				double cosy_cosp = 1 - 2 * (w_q_i[2] * w_q_i[2] + w_q_i[3] * w_q_i[3]);
    				double yaw_meas = atan2(siny_cosp, cosy_cosp);
                    //cout << "FRL yaw | " << yaw_meas*180.0/M_PI;
		
				//add factor
                    ceres::CostFunction* rot_function = YError::Create(yaw_meas,0.01);
                    problem.AddResidualBlock(rot_function, loss_function, q_array[i]);	

               }*/

			//O- mag factor as heading
               iterMag = magMap.find(t);
               if (iterMag != magMap.end())
               {
				double mag_meas[3] = {iterMag->second[0], iterMag->second[1], iterMag->second[2]};
				//cout << "| Xsense Mag | " << mag_meas[0] <<  "," << mag_meas[1] <<  ","<< mag_meas[2] ;
				
				// this has the vio attitude info
				Eigen::Quaterniond q_vio = Eigen::Quaterniond(iterVIO->second[3], iterVIO->second[4], iterVIO->second[5], iterVIO->second[6]);
                    Eigen::Vector3d euler = q_vio.toRotationMatrix().eulerAngles(2, 1, 0);
				
				//adjust mag reading                    
				Eigen::Quaterniond q_vio_no_yaw =  Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitZ())
    				* Eigen::AngleAxisd(euler[1], Eigen::Vector3d::UnitY())
    				* Eigen::AngleAxisd(euler[2], Eigen::Vector3d::UnitX());

				Eigen::Vector3d mag_meas_flat = q_vio_no_yaw * Eigen::Vector3d((mag_meas[0]-0.1829)/1.2073,(mag_meas[1]+0.1630)/1.1773,(mag_meas[2]+0.3197)/1.2761);


				double mag_ref[3]= {0.3633,0.0639,-0.4980}; 
				
                    double yaw_mag = atan2(mag_ref[1],mag_ref[0]) - atan2(mag_meas_flat[1],mag_meas_flat[0]);



                    //cout << "| Vio q | " << q_vio.w() << "," << q_vio.x() << "," << q_vio.y() << "," << q_vio.z()  <<"| Vio eul | "<< atan2(sin(euler[0]),cos(euler[0])) <<  "," << atan2(sin(euler[1]),cos(euler[1])) <<  ","<< atan2(sin(euler[2]),cos(euler[2]))  << "," << -90.0 + yaw_mag/M_PI*180 << endl;
				
				// remove attitude of the mag vector

				// use the magnetic reference to find the yaw - true heading

				// add factor -error
                    //ceres::CostFunction* rot_function = YError::Create(-3*M_PI/2+yaw_mag,0.05);
                    //problem.AddResidualBlock(rot_function, loss_function, q_array[i]);


                    Eigen::Quaterniond q_meas =  Eigen::AngleAxisd(-3*M_PI/2+yaw_mag, Eigen::Vector3d::UnitZ())
    				* Eigen::AngleAxisd(euler[1], Eigen::Vector3d::UnitY())
    				* Eigen::AngleAxisd(euler[2], Eigen::Vector3d::UnitX());
                    ceres::CostFunction* rot_function = RError::Create(q_meas.w(), q_meas.x(), 
                                                                       q_meas.y(), q_meas.z(),0.01);
                    //printf("inverse weight %f \n", iterGPS->second[3]);
                    problem.AddResidualBlock(rot_function, loss_function, q_array[i]);
				//char test;
				//cin >> test;
               }

            }
            //mPoseMap.unlock();
            ceres::Solve(options, &problem, &summary);
            //std::cout << summary.BriefReport() << "\n";

            // update global pose
            //mPoseMap.lock();
            iter = globalPoseMap.begin();
            for (int i = 0; i < length; i++, iter++)
            {
            	vector<double> globalPose{t_array[i][0], t_array[i][1], t_array[i][2],
            							  q_array[i][0], q_array[i][1], q_array[i][2], q_array[i][3]};
            	iter->second = globalPose;
            	if(i == length - 1)
            	{
            	    Eigen::Matrix4d WVIO_T_body = Eigen::Matrix4d::Identity(); 
            	    Eigen::Matrix4d WGPS_T_body = Eigen::Matrix4d::Identity();
            	    double t = iter->first;
            	    WVIO_T_body.block<3, 3>(0, 0) = Eigen::Quaterniond(localPoseMap[t][3], localPoseMap[t][4], 
            	                                                       localPoseMap[t][5], localPoseMap[t][6]).toRotationMatrix();
            	    WVIO_T_body.block<3, 1>(0, 3) = Eigen::Vector3d(localPoseMap[t][0], localPoseMap[t][1], localPoseMap[t][2]);
            	    WGPS_T_body.block<3, 3>(0, 0) = Eigen::Quaterniond(globalPose[3], globalPose[4], 
            	                                                        globalPose[5], globalPose[6]).toRotationMatrix();
            	    WGPS_T_body.block<3, 1>(0, 3) = Eigen::Vector3d(globalPose[0], globalPose[1], globalPose[2]);
            	    WGPS_T_WVIO = WGPS_T_body * WVIO_T_body.inverse();
            	}
            }
            updateGlobalPath();
            //printf("global time %f \n", globalOptimizationTime.toc());
            mPoseMap.unlock();
        }
        std::chrono::milliseconds dura(2000);
        std::this_thread::sleep_for(dura);
    }
	return;
}


void GlobalOptimization::updateGlobalPath()
{
    global_path.poses.clear();
    map<double, vector<double>>::iterator iter;
    for (iter = globalPoseMap.begin(); iter != globalPoseMap.end(); iter++)
    {
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header.stamp = ros::Time(iter->first);
        pose_stamped.header.frame_id = "worldGPS";
        pose_stamped.pose.position.x = iter->second[0];
        pose_stamped.pose.position.y = iter->second[1];
        pose_stamped.pose.position.z = iter->second[2];
        pose_stamped.pose.orientation.w = iter->second[3];
        pose_stamped.pose.orientation.x = iter->second[4];
        pose_stamped.pose.orientation.y = iter->second[5];
        pose_stamped.pose.orientation.z = iter->second[6];
        global_path.poses.push_back(pose_stamped);
    }

    gps_path.poses.clear();
    map<double, vector<double>>::iterator iter3;
    //cout << "GPS Map size: "<<GPSPositionMap.size() <<endl;//k
    for (iter3 = GPSPositionMap.begin(); iter3 != GPSPositionMap.end(); iter3++)
    {
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header.stamp = ros::Time(iter3->first);
        pose_stamped.header.frame_id = "worldGPS";
        pose_stamped.pose.position.x = iter3->second[0];
        pose_stamped.pose.position.y = iter3->second[1];
        pose_stamped.pose.position.z = iter3->second[2];
        pose_stamped.pose.orientation.w = 1.0;
        pose_stamped.pose.orientation.x = 0.0;
        pose_stamped.pose.orientation.y = 0.0;
        pose_stamped.pose.orientation.z = 0.0;
        gps_path.poses.push_back(pose_stamped);
    }

    ppk_path.poses.clear();
    map<double, vector<double>>::iterator iter4;
    //cout << "GPS Map size: "<<GPSPositionMap.size() <<endl;//k
    for (iter4 = PPKPositionMap.begin(); iter4 != PPKPositionMap.end(); iter4++)
    {
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header.stamp = ros::Time(iter4->first);
        pose_stamped.header.frame_id = "worldGPS";
        pose_stamped.pose.position.x = iter4->second[0];
        pose_stamped.pose.position.y = iter4->second[1];
        pose_stamped.pose.position.z = iter4->second[2];
        pose_stamped.pose.orientation.w = 1.0;
        pose_stamped.pose.orientation.x = 0.0;
        pose_stamped.pose.orientation.y = 0.0;
        pose_stamped.pose.orientation.z = 0.0;
        ppk_path.poses.push_back(pose_stamped);
    }

    //save results for KITTI evaluation tool
    int length = globalPoseMap.size();
    Eigen::Quaterniond odomQ;
    Eigen::Vector3d odomP;
    Eigen::Quaterniond gtQ;
    Eigen::Vector3d gtP;
    map<double, vector<double>>::iterator iter2;
    iter = localPoseMap.begin();
    iter2 = globalPoseMap.begin();
     // time sequence check-k  
    //double time_first = iter->first;
    for(int j = 0;j < GTframeCount; j++, iter++, iter2++){ // go to the current frame
    }
    std::ofstream foutC("resultsOdom.txt", std::ios::app);  
    std::ofstream foutD("resultsGt.txt", std::ios::app);           
    for (int i = GTframeCount; i < length; i++, iter++, iter2++)
    {
                
		GTframeCount++;                

                
                odomP.x() = iter->second[0];
                odomP.y() = iter->second[1];
                odomP.z() = iter->second[2];
                odomQ.w() = iter->second[3];
                odomQ.x() = iter->second[4];
                odomQ.y() = iter->second[5];
                odomQ.z() = iter->second[6];
                
                //time sequence check-k 
		//std::cout <<  iter->first - time_first << "," << odomP.x() <<  "|" ;  // ok correct time squence saved
   
		Eigen::Quaterniond globalQ;
    		globalQ = WGPS_T_WVIO_viz.block<3, 3>(0, 0) * odomQ;
    		Eigen::Vector3d globalP = WGPS_T_WVIO_viz.block<3, 3>(0, 0) * odomP + WGPS_T_WVIO_viz.block<3, 1>(0, 3);
   		
     		if(GTframeCount>0)
    		{
		Eigen::Matrix3d globalR = globalQ.normalized().toRotationMatrix();   
	    	foutC.setf(std::ios::fixed, std::ios::floatfield);
	    	foutC.precision(0);
		//foutC << header.stamp.toSec() * 1e9 << ",";
                foutC << GTframeCount << " ";
		foutC.precision(6);
		              foutC << globalR(0,0) << " "
				    << globalR(0,1) << " "
				    << globalR(0,2) << " "
				    << globalP.x()  << " "
				    << globalR(1,0) << " "
				    << globalR(1,1) << " "
				    << globalR(1,2) << " "
				    << globalP.y()  << " "
				    << globalR(2,0) << " "
				    << globalR(2,1) << " "
				    << globalR(2,2) << " "
				    << globalP.z()  << std::endl;
    		}

		gtP.x() = iter2->second[0];
                gtP.y() = iter2->second[1];
                gtP.z() = iter2->second[2];
                gtQ.w() = iter2->second[3];
                gtQ.x() = iter2->second[4];
                gtQ.y() = iter2->second[5];
                gtQ.z() = iter2->second[6];
    	
   		
     		if(GTframeCount>0)
    		{
		Eigen::Matrix3d gtR = gtQ.normalized().toRotationMatrix();   
	    	foutD.setf(std::ios::fixed, std::ios::floatfield);
	    	foutD.precision(0);
		//foutC << header.stamp.toSec() * 1e9 << ",";
                foutD << GTframeCount << " ";
		foutD.precision(6);
		              foutD << gtR(0,0) << " "
				    << gtR(0,1) << " "
				    << gtR(0,2) << " "
				    << gtP.x()  << " "
				    << gtR(1,0) << " "
				    << gtR(1,1) << " "
				    << gtR(1,2) << " "
				    << gtP.y()  << " "
				    << gtR(2,0) << " "
				    << gtR(2,1) << " "
				    << gtR(2,2) << " "
				    << gtP.z()  << std::endl;
    		}
    }
     // time sequence check -k
    //std::cout <<  std::endl;
    //std::cout <<  localPoseMap.end()->first <<std::endl;
    foutC.close();
    foutD.close();
}
