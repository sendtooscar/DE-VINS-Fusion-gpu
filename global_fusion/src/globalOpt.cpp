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

//R------------------------------R//
bool writeToFile = false; //save text files for evaluation
int updateGlobalPathCount = 0;
int optcounter = 0;
int ppkPosCounter = 0;
int vinsPosCounter = 0;
double last_GPS=0;
//R------------------------------R//

GlobalOptimization::GlobalOptimization():
outfileOdom("resultsOdom.txt", std::ios_base::trunc),
outfileGt("resultsGt.txt", std::ios_base::trunc),
outfileVINS("VINS_bell412_dataset1.txt", std::ios_base::trunc),
outfileGPS("PPK_bell412_dataset1.txt", std::ios_base::trunc),
outfileFusion("Fusion_bell412_dataset1.txt", std::ios_base::trunc),
laserCloudCornerLast(new pcl::PointCloud<PointType>()),
laserCloudSurfLast(new pcl::PointCloud<PointType>()),
laserCloudSurround(new pcl::PointCloud<PointType>()),
laserCloudCornerFromMap(new pcl::PointCloud<PointType>()),
laserCloudSurfFromMap(new pcl::PointCloud<PointType>()),
laserCloudFullRes(new pcl::PointCloud<PointType>()),
laserCloudCornerStack(new pcl::PointCloud<PointType>()),
laserCloudSurfStack(new pcl::PointCloud<PointType>()),
kdtreeCornerFromMap(new pcl::KdTreeFLANN<PointType>()),
kdtreeSurfFromMap(new pcl::KdTreeFLANN<PointType>())
//timeLaserCloudFullResLast(0.0),
//timeLaserCloudFullRes(0.0),
//laserCloudFullRes2(new pcl::PointCloud<PointType>())
{
    initGPS = false;
    newGPS = false;
    newGPSPR = false;
    newCloudFullRes = false;
    newCloud = false;
    WGPS_T_WVIO = Eigen::Matrix4d::Identity();
    WGPS_T_WVIO_viz = Eigen::Matrix4d::Identity();
    update_count =0;
    GTframeCount = 0;
    threadOpt = std::thread(&GlobalOptimization::optimize, this);
    last_update_time =0.0;


    float lineRes = 0.4;
    float planeRes = 0.8;
    printf("line resolution %f plane resolution %f \n", lineRes, planeRes);
    downSizeFilterCorner.setLeafSize(lineRes, lineRes,lineRes);
    downSizeFilterSurf.setLeafSize(planeRes, planeRes, planeRes);

    for (int i = 0; i < laserCloudNum; i++)
    {
		laserCloudCornerArray[i].reset(new pcl::PointCloud<PointType>());
		laserCloudSurfArray[i].reset(new pcl::PointCloud<PointType>());
    }


    
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

void GlobalOptimization::pointAssociateToMap(PointType const *const pi, PointType *const po)
{

     // modified to include the Lidar extrinsics
	Eigen::Vector3d point_L(pi->x, pi->y, pi->z);

     Eigen::Vector3d point_curr = q_I_L *point_L + t_I_L;
	Eigen::Vector3d point_w = q_w_curr * point_curr + t_w_curr;
	po->x = point_w.x();
	po->y = point_w.y();
	po->z = point_w.z();
	po->intensity = pi->intensity;
	//po->intensity = 1.0;
}

// NEW: cloud handling
/*void GlobalOptimization::inputCloudFullRes(double t,pcl::PointCloud<PointType>::Ptr& laserCloudFullResIn){
      timeLaserCloudFullRes=t;
      //laserCloudFullRes->clear();
      //*laserCloudFullRes = *laserCloudFullResIn;
      //pcl::PointCloud<PointType>::Ptr laserCloudFullRes(new pcl::PointCloud<PointType>());
      *laserCloudFullRes2 = *laserCloudFullResIn;
      //laserCloudFullRes->clear();
	 //pcl::fromROSMsg(*laserCloudFullResIn, *laserCloudFullRes);
      if (timeLaserCloudFullRes > timeLaserCloudFullResLast){
	 	newCloudFullRes = true;
	 }
}*/

void GlobalOptimization::inputOdom(double t, Eigen::Vector3d OdomP, Eigen::Quaterniond OdomQ)
{
	mPoseMap.lock();
    vector<double> localPose{OdomP.x(), OdomP.y(), OdomP.z(), 
    					     OdomQ.w(), OdomQ.x(), OdomQ.y(), OdomQ.z()};
    localPoseMap[t] = localPose;

	//R--------------------------R//
	if(writeToFile)
	{
		//write file - VINS
    		Eigen::Matrix3d odomR = OdomQ.normalized().toRotationMatrix();
    		std::ofstream foutE("VINS_bell412_dataset1.txt", std::ios::app); 
    		vinsPosCounter++;
    		foutE.setf(std::ios::fixed, std::ios::floatfield);
    		foutE.precision(0);
    		foutE << vinsPosCounter << " ";
    		foutE.precision(9);
    		foutE << t  << " "
           << odomR(0,0) << " "
           << odomR(0,1) << " "
           << odomR(0,2) << " "
            << OdomP.x()  << " "
            << odomR(1,0) << " "
            << odomR(1,1) << " "
            << odomR(1,2) << " "
            << OdomP.y()  << " "
            << odomR(2,0) << " "
            << odomR(2,1) << " "
            << odomR(2,2) << " "
            << OdomP.z()  << std::endl;	
	}
	//R--------------------------R//


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
    frl_path.header = pose_stamped.header;


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
    // get ref mag heading
    // get current mag heading 
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
     mPoseMap.lock();
	GPSPositionMap[t] = tmp;
     newGPS = true;
     mPoseMap.unlock();
}

void GlobalOptimization::inputGPSPR(double t, double latitude, double longitude, double altitude, double posAccuracy)
{    
     double xyz[3];
	GPS2XYZ(latitude, longitude, altitude, xyz);
	vector<double> tmp{xyz[0], xyz[1], xyz[2], posAccuracy};
     mPoseMap.lock();
	GPSPRPositionMap[t] = tmp;
     newGPSPR = true;
     mPoseMap.unlock();
}

void GlobalOptimization::inputRot(double t, double q_w, double q_x, double q_y, double q_z, double rotAccuracy)
{
	vector<double> tmp{q_w, q_x, q_y, q_z, rotAccuracy};
     mPoseMap.lock();
	globalRotMap[t] = tmp;
     newRot = true;
     mPoseMap.unlock();
}

void GlobalOptimization::inputMag(double t, double mag_x, double mag_y, double mag_z, double magAccuracy)
{
	vector<double> tmp{mag_x, mag_y, mag_z, magAccuracy};
     mPoseMap.lock();
	magMap[t] = tmp;
     newMag = true;
     mPoseMap.unlock();
}

void GlobalOptimization::inputPPKviz(double t, double latitude, double longitude, double altitude, double posAccuracy)
{
	double xyz[3];
	GPS2XYZ(latitude, longitude, altitude, xyz);
	vector<double> tmp{xyz[0], xyz[1], xyz[2], posAccuracy};
     mPoseMap.lock();
	PPKPositionMap[t] = tmp;
     mPoseMap.unlock();     

	//R----------------------R//
     last_GPS=0;
	std::ofstream foutF("PPK_bell412_dataset1.txt", std::ios::app);
    	//std::ofstream foutF("GPS_LH.txt", std::ios::app); 
    	ppkPosCounter++;
    	foutF.setf(std::ios::fixed, std::ios::floatfield);
    	foutF.precision(0);
    	foutF << ppkPosCounter << " ";
    	foutF.precision(9);
    	foutF << t  << " "
          << xyz[0]  << " "
          << xyz[1]  << " "
          << xyz[2]  << std::endl;
     last_GPS=t;
	std::cout << t << std::endl;
	//R----------------------R//


}

void GlobalOptimization::inputFRLviz(double t, double latitude, double longitude, double altitude, double w , double x,  double y,  double z)
{
	double xyz[3];
	GPS2XYZ(latitude, longitude, altitude, xyz);
	vector<double> tmp{xyz[0], xyz[1], xyz[2], w, x, y, z};
     mPoseMap.lock();
	FRLPoseMap[t] = tmp;
	mPoseMap.unlock();
}



void GlobalOptimization::inputSurfnCorners(double t, pcl::PointCloud<PointType>::Ptr& laserCloudCornerLastin, pcl::PointCloud<PointType>::Ptr& laserCloudSurfLastin){
     
     mPoseMap.lock();
     // bufffer this to the optimize thread -  the optimize thread will discard as needed to keep things real time
     if (newCloud ==  false){
		laserCloudCornerLast = laserCloudCornerLastin;
          laserCloudSurfLast = laserCloudSurfLastin;
          timeLaserCloud = t;
          //cout << "cloud size "<<laserCloudCornerLastin->size() << "|" << laserCloudCornerLast->size() << "|" << timeLaserCloud <<endl;
          
          newCloud =  true;
	}
	else {
          printf("-x");
	}
     mPoseMap.unlock();
}

void GlobalOptimization::optimize()
{
    while(true)
    {

	   flag_lidar_sync = false;
        laserCloudSurfFromMapNum = 0;
        laserCloudCornerFromMapNum = 0;
        
        if(newGPS || newRot || newMag || newCloud || newGPSPR) //warning these flags are assumed thread safe ( only read here and locked at writing)
        {
            printf("\n --------------------------------------- \n");
            printf("global optimization %d,%d,%d,%d,%d\n",newGPS,newRot,newMag,newCloud,newGPSPR);
     
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
            
          
            map<double, vector<double>>::iterator iterVIO, iterVIONext, iterGPS, iterGPSPR, iterRot, iterMag;
            int pose_i = 0;
            int synced_pose_i =0;
            for (iterVIO = localPoseMap.begin(); iterVIO != localPoseMap.end(); iterVIO++, pose_i++)
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
                    problem.AddResidualBlock(vio_function, NULL, q_array[pose_i], t_array[pose_i], q_array[pose_i+1], t_array[pose_i+1]);
                }

			 
                //gps factor
                double t = iterVIO->first;// TODO: check if this should be iterVIONext  this corresponds to interator pose_i
                

                //initialization factor if GPS is available
                /*if (GPSPositionMap.size() >0 && pose_i<10){ // for the first 10 poses
			     iterGPS = GPSPositionMap.find(t);
                	if (iterGPS != GPSPositionMap.end())
                	{
                   	     ceres::CostFunction* gps_function = TError::Create(iterGPS->second[0], iterGPS->second[1], 
                                                                       iterGPS->second[2], 0.3);
                    	printf("Init. \n");
                    	problem.AddResidualBlock(gps_function, loss_function, t_array[pose_i]);

                	}
			 } // init factor - handled below */

                if(newGPS || GPSPositionMap.size() >0){
                iterGPS = GPSPositionMap.find(t);
                if (iterGPS != GPSPositionMap.end())
                {
                    ceres::CostFunction* gps_function = TError::Create(iterGPS->second[0], iterGPS->second[1], 
                                                                       iterGPS->second[2], iterGPS->second[3]);
                    //printf("inverse weight %f \n", iterGPS->second[3]);
                    problem.AddResidualBlock(gps_function, loss_function, t_array[pose_i]);

                }}//newGPS


                if(newGPSPR || GPSPRPositionMap.size() >0){
                iterGPSPR = GPSPRPositionMap.find(t);
                if (iterGPSPR != GPSPRPositionMap.end())
                {
                    ceres::CostFunction* gps_pr_function = XYError::Create(iterGPSPR->second[0], iterGPSPR->second[1], 
                                                                       iterGPSPR->second[2], iterGPSPR->second[3]);
                    //printf("inverse weight %f \n", iterGPS->second[3]);
				cout << "recieved place recognition match with size: " << iterGPSPR->second[0]<< ','<< iterGPSPR->second[1] << " at pose node: " << pose_i << " of " << length <<endl;
                    problem.AddResidualBlock(gps_pr_function, loss_function, t_array[pose_i]);

                }}//newGPSPR


                // O- Rot factor (FULL AHRS INPUT) --k
                if(newRot || globalRotMap.size() >0){iterRot = globalRotMap.find(t);
                if (iterRot != globalRotMap.end())
                {
                    ceres::CostFunction* rot_function = RError::Create(iterRot->second[0], iterRot->second[1], 
                                                                       iterRot->second[2], iterRot->second[3],0.01);
                    //printf("inverse weight %f \n", iterGPS->second[3]);
                    problem.AddResidualBlock(rot_function, loss_function, q_array[pose_i]);

                }}
			
			// O- Rot factor (Yaw INPUT) - k possible lag ~1-2 second
               /*if(newRot){
               iterRot = globalRotMap.find(t);
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
                    problem.AddResidualBlock(rot_function, loss_function, q_array[pose_i]);	

               }}*///newRot

              
			//O- mag factor as heading // note AHRS input is implicitly added here - roll pitch from AHRS IMU, Yaw from Mag 
               //TODO: to evaluate without Mag we need a roll pitch only global factor ( i.e. global VRU reference residual = R^T*g_E-y_vru)
               if(newMag || magMap.size() > 0){
               iterMag = magMap.find(t);
               if (iterMag != magMap.end())
               {
				double mag_meas[3] = {iterMag->second[0], iterMag->second[1], iterMag->second[2]};
				//cout << "| Xsense Mag | " << mag_meas[0] <<  "," << mag_meas[1] <<  ","<< mag_meas[2] <<t <<"'" << magMap.size() ;
				
				// this has the vio attitude info
				Eigen::Quaterniond q_vio = Eigen::Quaterniond(iterVIO->second[3], iterVIO->second[4], iterVIO->second[5], iterVIO->second[6]);
                    Eigen::Vector3d euler = q_vio.toRotationMatrix().eulerAngles(2, 1, 0);
				
				//adjust mag reading                    
				Eigen::Quaterniond q_vio_no_yaw =  Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitZ())
    				* Eigen::AngleAxisd(euler[1], Eigen::Vector3d::UnitY())
    				* Eigen::AngleAxisd(euler[2], Eigen::Vector3d::UnitX());

				//Eigen::Vector3d mag_meas_flat = q_vio_no_yaw * Eigen::Vector3d((mag_meas[0]-0.1829)/1.2073,(mag_meas[1]+0.1630)/1.1773,(mag_meas[2]+0.3197)/1.2761); //manual calib
                    Eigen::Vector3d mag_meas_flat = q_vio_no_yaw * Eigen::Vector3d((mag_meas[0]-0.2435)/1.3211,(mag_meas[1]+0.1588)/1.3241,(mag_meas[2]+1.8277)/2.2852); // using frl and declination data


				//double mag_ref[3]= {0.3633,0.0639,-0.4980}; //manual calib
                    double mag_ref[3]= {0.3338,0.0884,-0.9385};  // using frl and declination data
				
                    double yaw_mag = atan2(mag_ref[1],mag_ref[0]) - atan2(mag_meas_flat[1],mag_meas_flat[0]);



                    //cout << "| Vio q | " << q_vio.w() << "," << q_vio.x() << "," << q_vio.y() << "," << q_vio.z()  <<"| Vio eul | "<< atan2(sin(euler[0]),cos(euler[0])) <<  "," << atan2(sin(euler[1]),cos(euler[1])) <<  ","<< atan2(sin(euler[2]),cos(euler[2]))  << "," << -90.0 + yaw_mag/M_PI*180 << endl;
				
				// remove attitude of the mag vector

				// use the magnetic reference to find the yaw - true heading

				// add factor -error
                    //ceres::CostFunction* rot_function = YError::Create(-3*M_PI/2+yaw_mag,0.05);
                    //problem.AddResidualBlock(rot_function, loss_function, q_array[pose_i]);


                    Eigen::Quaterniond q_meas =  Eigen::AngleAxisd(-3*M_PI/2+yaw_mag, Eigen::Vector3d::UnitZ()) // this +90 or - 270 is needed due to baing caliberated for NWU and missing wrap to pi
    				* Eigen::AngleAxisd(euler[1], Eigen::Vector3d::UnitY())
    				* Eigen::AngleAxisd(euler[2], Eigen::Vector3d::UnitX());
                    ceres::CostFunction* rot_function = RError::Create(q_meas.w(), q_meas.x(), 
                                                                       q_meas.y(), q_meas.z(),0.01);
                    //printf("inverse weight %f \n", iterGPS->second[3]);
                    problem.AddResidualBlock(rot_function, loss_function, q_array[pose_i]);
				//char test;
				//cin >> test;
               }}//newMag

               
               if(newCloud){
	               // check times of the pose iterator to only trigger at correct pose
                    //
                    if (timeLaserCloud ==t){
 					flag_lidar_sync = true;	
                         synced_pose_i = pose_i;		          
 					cout << "recieved feature cloud with size: " << laserCloudCornerLast->size() << ","<< 
                         laserCloudSurfLast->size() << "at pose node: " << pose_i << "of" << length <<endl;
                          
	                    TicToc t_whole;
                         // the curertn pose in global map is here : q_array[pose_i], t_array[pose_i];
                         
					q_w_curr.w() = q_array[pose_i][0];
					q_w_curr.x() = q_array[pose_i][1];
	                    q_w_curr.y() = q_array[pose_i][2];
	                    q_w_curr.z() = q_array[pose_i][3];
                         t_w_curr.x() = t_array[pose_i][0];
					t_w_curr.y() = t_array[pose_i][1];
                         t_w_curr.z() = t_array[pose_i][2];
					//mPoseMap.unlock();
				     
					// move the octree so there is a 2 cube margin around the current pose for it to grow
                         // keep the cubeindex of the current pose always >2 or < full size -2 (2 cube margin)
                         TicToc t_shift;
					int centerCubeI = int((t_w_curr.x() + 25.0) / 50.0) + laserCloudCenWidth;
					int centerCubeJ = int((t_w_curr.y() + 25.0) / 50.0) + laserCloudCenHeight;
					int centerCubeK = int((t_w_curr.z() + 25.0) / 50.0) + laserCloudCenDepth;

					if (t_w_curr.x() + 25.0 < 0)
						centerCubeI--;
					if (t_w_curr.y() + 25.0 < 0)
						centerCubeJ--;
					if (t_w_curr.z() + 25.0 < 0)
						centerCubeK--;

					while (centerCubeI < 3)
					{
						for (int j = 0; j < laserCloudHeight; j++)
						{
							for (int k = 0; k < laserCloudDepth; k++)
							{ 
								int i = laserCloudWidth - 1;
								pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
									laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k]; 
								pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =
									laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
								for (; i >= 1; i--)
								{
									laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
										laserCloudCornerArray[i - 1 + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
									laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
										laserCloudSurfArray[i - 1 + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
								}
								laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
									laserCloudCubeCornerPointer;
								laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
									laserCloudCubeSurfPointer;
								laserCloudCubeCornerPointer->clear();
								laserCloudCubeSurfPointer->clear();
							}
						}

						centerCubeI++;
						laserCloudCenWidth++;
					}

					while (centerCubeI >= laserCloudWidth - 3)
					{ 
						for (int j = 0; j < laserCloudHeight; j++)
						{
							for (int k = 0; k < laserCloudDepth; k++)
							{
								int i = 0;
								pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
									laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
								pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =
									laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
								for (; i < laserCloudWidth - 1; i++)
								{
									laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
										laserCloudCornerArray[i + 1 + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
									laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
										laserCloudSurfArray[i + 1 + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
								}
								laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
									laserCloudCubeCornerPointer;
								laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
									laserCloudCubeSurfPointer;
								laserCloudCubeCornerPointer->clear();
								laserCloudCubeSurfPointer->clear();
							}
						}

						centerCubeI--;
						laserCloudCenWidth--;
					}

					while (centerCubeJ < 3)
					{
						for (int i = 0; i < laserCloudWidth; i++)
						{
							for (int k = 0; k < laserCloudDepth; k++)
							{
								int j = laserCloudHeight - 1;
								pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
									laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
								pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =
									laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
								for (; j >= 1; j--)
								{
									laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
										laserCloudCornerArray[i + laserCloudWidth * (j - 1) + laserCloudWidth * laserCloudHeight * k];
									laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
										laserCloudSurfArray[i + laserCloudWidth * (j - 1) + laserCloudWidth * laserCloudHeight * k];
								}
								laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
									laserCloudCubeCornerPointer;
								laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
									laserCloudCubeSurfPointer;
								laserCloudCubeCornerPointer->clear();
								laserCloudCubeSurfPointer->clear();
							}
						}

						centerCubeJ++;
						laserCloudCenHeight++;
					}

					while (centerCubeJ >= laserCloudHeight - 3)
					{
						for (int i = 0; i < laserCloudWidth; i++)
						{
							for (int k = 0; k < laserCloudDepth; k++)
							{
								int j = 0;
								pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
									laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
								pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =
									laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
								for (; j < laserCloudHeight - 1; j++)
								{
									laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
										laserCloudCornerArray[i + laserCloudWidth * (j + 1) + laserCloudWidth * laserCloudHeight * k];
									laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
										laserCloudSurfArray[i + laserCloudWidth * (j + 1) + laserCloudWidth * laserCloudHeight * k];
								}
								laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
									laserCloudCubeCornerPointer;
								laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
									laserCloudCubeSurfPointer;
								laserCloudCubeCornerPointer->clear();
								laserCloudCubeSurfPointer->clear();
							}
						}

						centerCubeJ--;
						laserCloudCenHeight--;
					}

					while (centerCubeK < 3)
					{
						for (int i = 0; i < laserCloudWidth; i++)
						{
							for (int j = 0; j < laserCloudHeight; j++)
							{
								int k = laserCloudDepth - 1;
								pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
									laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
								pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =
									laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
								for (; k >= 1; k--)
								{
									laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
										laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * (k - 1)];
									laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
										laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * (k - 1)];
								}
								laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
									laserCloudCubeCornerPointer;
								laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
									laserCloudCubeSurfPointer;
								laserCloudCubeCornerPointer->clear();
								laserCloudCubeSurfPointer->clear();
							}
						}

						centerCubeK++;
						laserCloudCenDepth++;
					}

					while (centerCubeK >= laserCloudDepth - 3)
					{
						for (int i = 0; i < laserCloudWidth; i++)
						{
							for (int j = 0; j < laserCloudHeight; j++)
							{
								int k = 0;
								pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
									laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
								pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =
									laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
								for (; k < laserCloudDepth - 1; k++)
								{
									laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
										laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * (k + 1)];
									laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
										laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * (k + 1)];
								}
								laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
									laserCloudCubeCornerPointer;
								laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
									laserCloudCubeSurfPointer;
								laserCloudCubeCornerPointer->clear();
								laserCloudCubeSurfPointer->clear();
							}
						}

						centerCubeK--;
						laserCloudCenDepth--;
					}
                         // end shift the Octree
                        
                         printf("Current cube index %d %d %d | for pose %f %f %f\n", centerCubeI, centerCubeJ, centerCubeK, t_w_curr.x(), t_w_curr.y(), t_w_curr.z());  
                         //find the indexes of the clouds which are in the 2 cube neighbourhood of the current pose
					laserCloudValidNum = 0;
					int laserCloudSurroundNum = 0;

					for (int i = centerCubeI - 2; i <= centerCubeI + 2; i++)
					{
						for (int j = centerCubeJ - 2; j <= centerCubeJ + 2; j++)
						{
							for (int k = centerCubeK - 1; k <= centerCubeK + 1; k++)
							{
								if (i >= 0 && i < laserCloudWidth &&
									j >= 0 && j < laserCloudHeight &&
									k >= 0 && k < laserCloudDepth)
								{ 
									laserCloudValidInd[laserCloudValidNum] = i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k;
									laserCloudValidNum++;
									laserCloudSurroundInd[laserCloudSurroundNum] = i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k;
									laserCloudSurroundNum++;
								}
							}
						}
					}


                         // use those indexes to create a map of the surrounding
					laserCloudCornerFromMap->clear();
					laserCloudSurfFromMap->clear();

					for (int i = 0; i < laserCloudValidNum; i++)
					{
						*laserCloudCornerFromMap += *laserCloudCornerArray[laserCloudValidInd[i]];
						*laserCloudSurfFromMap += *laserCloudSurfArray[laserCloudValidInd[i]];
					}
					laserCloudCornerFromMapNum = laserCloudCornerFromMap->points.size();
					laserCloudSurfFromMapNum = laserCloudSurfFromMap->points.size();

                         // include the current laser points in a down sampled pcl cloud   
                         //mPoseMap.lock();
					laserCloudCornerStack->clear();
					downSizeFilterCorner.setInputCloud(laserCloudCornerLast);
					downSizeFilterCorner.filter(*laserCloudCornerStack);
					laserCloudCornerStackNum = laserCloudCornerStack->points.size();

					laserCloudSurfStack->clear();
					downSizeFilterSurf.setInputCloud(laserCloudSurfLast);
					downSizeFilterSurf.filter(*laserCloudSurfStack);
					laserCloudSurfStackNum = laserCloudSurfStack->points.size();
                         //mPoseMap.unlock();

					printf("map prepare time %f ms\n", t_shift.toc());
					printf("map corner num %d  surf num %d \n", laserCloudCornerFromMapNum, laserCloudSurfFromMapNum);
                         printf("stack corner num %d  surf num %d \n", laserCloudCornerStackNum, laserCloudSurfStackNum);  
                         //scan matching - sent to optimization
                
					

                         // post optimization steps are resumed after global path update below
                         //mPoseMap.lock();

                    }//sync-found
            	}//newCloud	

            }// pose graph iterator    
            newGPS = false;
            newRot = false;
            newMag = false;
            newCloud = false;
            newGPSPR = false;
            mPoseMap.unlock();

		            
		  if(flag_lidar_sync && laserCloudCornerFromMapNum > 10 && laserCloudSurfFromMapNum > 50){
			
			TicToc t_tree;
               kdtreeCornerFromMap->setInputCloud(laserCloudCornerFromMap);
			kdtreeSurfFromMap->setInputCloud(laserCloudSurfFromMap);
			printf("build tree time %f ms \n", t_tree.toc());			
			
			TicToc t_opt;
			for (int iterCount = 0; iterCount < 2; iterCount++) // iterate twice for ICP- break if no matches
			{
				TicToc t_data;
				int corner_num = 0;
				for (int i = 0; i < laserCloudCornerStackNum; i++){
					pointOri = laserCloudCornerStack->points[i];
					//double sqrtDis = pointOri.x * pointOri.x + pointOri.y * pointOri.y + pointOri.z * pointOri.z;
					pointAssociateToMap(&pointOri, &pointSel); // warning uses the global t_w_curr
					kdtreeCornerFromMap->nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis); // find the nearest 5

					if (pointSearchSqDis[4] < 1.0)
					{ 
						std::vector<Eigen::Vector3d> nearCorners;
						Eigen::Vector3d center(0, 0, 0);
						// mean of nearest points
						for (int j = 0; j < 5; j++)
						{
							Eigen::Vector3d tmp(laserCloudCornerFromMap->points[pointSearchInd[j]].x,
											laserCloudCornerFromMap->points[pointSearchInd[j]].y,
											laserCloudCornerFromMap->points[pointSearchInd[j]].z);
							center = center + tmp;
							nearCorners.push_back(tmp);
						}
						center = center / 5.0;

						//covariance matrix of nearest points
						Eigen::Matrix3d covMat = Eigen::Matrix3d::Zero();
						for (int j = 0; j < 5; j++)
						{
							Eigen::Matrix<double, 3, 1> tmpZeroMean = nearCorners[j] - center;
							covMat = covMat + tmpZeroMean * tmpZeroMean.transpose();
						}

						Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(covMat);

						// if is indeed line feature ( checks the highest eigen value is larger than 3 times the second smallest)
						// note Eigen library sort eigenvalues in increasing order
						Eigen::Vector3d unit_direction = saes.eigenvectors().col(2);
    						Eigen::Vector3d curr_point_L(pointOri.x, pointOri.y, pointOri.z);
                               
                              //O - change the current point to IMU frame  
                              Eigen::Vector3d curr_point = q_I_L * curr_point_L + t_I_L;                 

						if (saes.eigenvalues()[2] > 3 * saes.eigenvalues()[1])
						{ 
							Eigen::Vector3d point_on_line = center;
							Eigen::Vector3d point_a, point_b;
							point_a = 0.1 * unit_direction + point_on_line; //find two points to define the line
							point_b = -0.1 * unit_direction + point_on_line;

							ceres::CostFunction *cost_function = LidarEdgeFactor::Create(curr_point, point_a, point_b, 1.0);
							problem.AddResidualBlock(cost_function, loss_function, q_array[synced_pose_i], t_array[synced_pose_i]);
							corner_num++;	
						}							
					}
					/*
					else if(pointSearchSqDis[4] < 0.01 * sqrtDis)
					{
						Eigen::Vector3d center(0, 0, 0);
						for (int j = 0; j < 5; j++)
						{
							Eigen::Vector3d tmp(laserCloudCornerFromMap->points[pointSearchInd[j]].x,
													laserCloudCornerFromMap->points[pointSearchInd[j]].y,
													laserCloudCornerFromMap->points[pointSearchInd[j]].z);
							center = center + tmp;
						}
						center = center / 5.0;	
						Eigen::Vector3d curr_point(pointOri.x, pointOri.y, pointOri.z);
						ceres::CostFunction *cost_function = LidarDistanceFactor::Create(curr_point, center);
						problem.AddResidualBlock(cost_function, loss_function, parameters, parameters + 4);
					}
					*/
				}
								

				int surf_num = 0;
				for (int i = 0; i < laserCloudSurfStackNum; i++)
				{
					pointOri = laserCloudSurfStack->points[i];
					//double sqrtDis = pointOri.x * pointOri.x + pointOri.y * pointOri.y + pointOri.z * pointOri.z;
					pointAssociateToMap(&pointOri, &pointSel);
					kdtreeSurfFromMap->nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);

					Eigen::Matrix<double, 5, 3> matA0;
					Eigen::Matrix<double, 5, 1> matB0 = -1 * Eigen::Matrix<double, 5, 1>::Ones();
					if (pointSearchSqDis[4] < 1.0)				
					{						
						for (int j = 0; j < 5; j++)
						{
							matA0(j, 0) = laserCloudSurfFromMap->points[pointSearchInd[j]].x;
							matA0(j, 1) = laserCloudSurfFromMap->points[pointSearchInd[j]].y;
							matA0(j, 2) = laserCloudSurfFromMap->points[pointSearchInd[j]].z;
							//printf(" pts %f %f %f ", matA0(j, 0), matA0(j, 1), matA0(j, 2));
						}
						// find the norm of plane
						Eigen::Vector3d norm = matA0.colPivHouseholderQr().solve(matB0);
						double negative_OA_dot_norm = 1 / norm.norm();
						norm.normalize();

						// Here n(pa, pb, pc) is unit norm of plane
						bool planeValid = true;
						for (int j = 0; j < 5; j++)
						{
							// if OX * n > 0.2, then plane is not fit well
							if (fabs(norm(0) * laserCloudSurfFromMap->points[pointSearchInd[j]].x +
								norm(1) * laserCloudSurfFromMap->points[pointSearchInd[j]].y +
								norm(2) * laserCloudSurfFromMap->points[pointSearchInd[j]].z + negative_OA_dot_norm) > 0.2)
							{
								planeValid = false;
								break;
							}
						}
						Eigen::Vector3d curr_point_L(pointOri.x, pointOri.y, pointOri.z);
                               
                              //O - change the current point to IMU frame  
                              Eigen::Vector3d curr_point = q_I_L * curr_point_L + t_I_L;
						if (planeValid)
						{
							ceres::CostFunction *cost_function = LidarPlaneNormFactor::Create(curr_point, norm, negative_OA_dot_norm);
							problem.AddResidualBlock(cost_function, loss_function, q_array[synced_pose_i], t_array[synced_pose_i]);
							surf_num++;
						}
					}
					/*
					else if(pointSearchSqDis[4] < 0.01 * sqrtDis)
					{
						Eigen::Vector3d center(0, 0, 0);
						for (int j = 0; j < 5; j++)
						{
							Eigen::Vector3d tmp(laserCloudSurfFromMap->points[pointSearchInd[j]].x,
												laserCloudSurfFromMap->points[pointSearchInd[j]].y,
												laserCloudSurfFromMap->points[pointSearchInd[j]].z);
							center = center + tmp;
						}
						center = center / 5.0;	
						Eigen::Vector3d curr_point(pointOri.x, pointOri.y, pointOri.z);
						ceres::CostFunction *cost_function = LidarDistanceFactor::Create(curr_point, center);
						problem.AddResidualBlock(cost_function, loss_function, parameters, parameters + 4);
					}
					*/
				}

				printf("corner num %d used corner num %d \n", laserCloudCornerStackNum, corner_num);
			     printf("surf num %d used surf num %d \n", laserCloudSurfStackNum, surf_num);
				printf("mapping data assosiation time %f ms \n", t_data.toc());

				ceres::Solve(options, &problem, &summary);

				if(corner_num<5 && surf_num <5){ // there is not enough associations break
					ROS_WARN("Insufficient Lidar feature matches");
					break;
				}
			}
		  }
		  else{ //this is the update without scan matching
		  	ROS_WARN("No Lidar update");
            	ceres::Solve(options, &problem, &summary);
            	//std::cout << summary.BriefReport() << "\n";
            }
            // update global pose
            mPoseMap.lock();
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
          
		  


            // completing the rest of the lidar mapping steps
            if(flag_lidar_sync){
               // the pose graph is now updated by the optimizer -- retrieve current scan pose
			q_w_curr.w() = q_array[synced_pose_i][0];
			q_w_curr.x() = q_array[synced_pose_i][1];
	          q_w_curr.y() = q_array[synced_pose_i][2];
	          q_w_curr.z() = q_array[synced_pose_i][3];
               t_w_curr.x() = t_array[synced_pose_i][0];
			t_w_curr.y() = t_array[synced_pose_i][1];
               t_w_curr.z() = t_array[synced_pose_i][2];			
			
			// start adding scans to the map			
			TicToc t_add;
			for (int i = 0; i < laserCloudCornerStackNum; i++)
			{
				pointAssociateToMap(&laserCloudCornerStack->points[i], &pointSel);

				int cubeI = int((pointSel.x + 25.0) / 50.0) + laserCloudCenWidth;
				int cubeJ = int((pointSel.y + 25.0) / 50.0) + laserCloudCenHeight;
				int cubeK = int((pointSel.z + 25.0) / 50.0) + laserCloudCenDepth;

				if (pointSel.x + 25.0 < 0)
					cubeI--;
				if (pointSel.y + 25.0 < 0)
					cubeJ--;
				if (pointSel.z + 25.0 < 0)
					cubeK--;

				if (cubeI >= 0 && cubeI < laserCloudWidth &&
					cubeJ >= 0 && cubeJ < laserCloudHeight &&
					cubeK >= 0 && cubeK < laserCloudDepth)
				{
					int cubeInd = cubeI + laserCloudWidth * cubeJ + laserCloudWidth * laserCloudHeight * cubeK;
					laserCloudCornerArray[cubeInd]->push_back(pointSel);
				}

				//debugging
				/*if(i<10){
					printf("Last cube index %d %d %d | for point %f %f %f | for pose %f %f %f \n", cubeI, cubeJ, cubeK, pointSel.x, pointSel.y, pointSel.z, t_w_curr.x(), t_w_curr.y(), t_w_curr.z() );
				}*/

			}

			  
               

			for (int i = 0; i < laserCloudSurfStackNum; i++)
			{
				pointAssociateToMap(&laserCloudSurfStack->points[i], &pointSel);

				int cubeI = int((pointSel.x + 25.0) / 50.0) + laserCloudCenWidth;
				int cubeJ = int((pointSel.y + 25.0) / 50.0) + laserCloudCenHeight;
				int cubeK = int((pointSel.z + 25.0) / 50.0) + laserCloudCenDepth;

				if (pointSel.x + 25.0 < 0)
					cubeI--;
				if (pointSel.y + 25.0 < 0)
					cubeJ--;
				if (pointSel.z + 25.0 < 0)
					cubeK--;

				if (cubeI >= 0 && cubeI < laserCloudWidth &&
					cubeJ >= 0 && cubeJ < laserCloudHeight &&
					cubeK >= 0 && cubeK < laserCloudDepth)
				{
					int cubeInd = cubeI + laserCloudWidth * cubeJ + laserCloudWidth * laserCloudHeight * cubeK;
					laserCloudSurfArray[cubeInd]->push_back(pointSel);
				}
			}
			printf("add points time %f ms\n", t_add.toc());
			
			TicToc t_filter;
			for (int i = 0; i < laserCloudValidNum; i++)
			{
				int ind = laserCloudValidInd[i];

				pcl::PointCloud<PointType>::Ptr tmpCorner(new pcl::PointCloud<PointType>());
				downSizeFilterCorner.setInputCloud(laserCloudCornerArray[ind]);
				downSizeFilterCorner.filter(*tmpCorner);
				laserCloudCornerArray[ind] = tmpCorner;

				pcl::PointCloud<PointType>::Ptr tmpSurf(new pcl::PointCloud<PointType>());
				downSizeFilterSurf.setInputCloud(laserCloudSurfArray[ind]);
				downSizeFilterSurf.filter(*tmpSurf);
				laserCloudSurfArray[ind] = tmpSurf;
			}
			printf("filter time %f ms \n", t_filter.toc());

	          flag_lidar_sync = false;
		  }


		  if(newCloudFullRes){
			  // use the vio pose to convert the cloud to the global frame and update the pcl cloud	
                 // set a public pointer of the point cloud 
                 // use that ot publish the map 

		  }


		 // update teh publish cloud
           
          printf("global time %f \n", globalOptimizationTime.toc());
          printf("--------------------------------------- \n");
          mPoseMap.unlock();


		
           
        }
        std::chrono::milliseconds dura(500);  // delay 2 seconds // should target time - spent time
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
        last_update_time = pose_stamped.header.stamp.toSec();
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
  
    frl_path.poses.clear();
    map<double, vector<double>>::iterator iter5;
    for (iter5 = FRLPoseMap.begin(); iter5 != FRLPoseMap.end(); iter5++)
    {
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header.stamp = ros::Time(iter5->first);
        pose_stamped.header.frame_id = "worldGPS";
        pose_stamped.pose.position.x = iter5->second[0];
        pose_stamped.pose.position.y = iter5->second[1];
        pose_stamped.pose.position.z = iter5->second[2];
        pose_stamped.pose.orientation.w = iter5->second[3];
        pose_stamped.pose.orientation.x = iter5->second[4];
        pose_stamped.pose.orientation.y = iter5->second[5];
        pose_stamped.pose.orientation.z = iter5->second[6];
        frl_path.poses.push_back(pose_stamped);
    }
   
	//save the path here when value > 6769
	updateGlobalPathCount++;

    //save results for KITTI evaluation tool
    int length = globalPoseMap.size();
    Eigen::Quaterniond odomQ;
    Eigen::Vector3d odomP;
    Eigen::Quaterniond gtQ;
    Eigen::Vector3d gtP;
    map<double, vector<double>>::iterator iter2;
    
    //fusion vs vins
    iter = localPoseMap.begin();
    iter2 = globalPoseMap.begin();

   


     // time sequence check-k  
    //double time_first = iter->first;

	//R---------------------------R//
	//to save full map
	Eigen::Quaterniond odomQAll;
	Eigen::Vector3d odomPAll;
	//local pose
	map<double, vector<double>>::iterator iter_lOdom;
	iter_lOdom = localPoseMap.begin();
	//global pose
	map<double, vector<double>>::iterator iter_gOdom;
	iter_gOdom = globalPoseMap.begin();
	//write the whole map to a text file
	map<double, vector<double>>::iterator iterFull_gOdom;
	iterFull_gOdom = globalPoseMap.begin();
	//test opt count
	int printOnceInTerminal = 1;
	//R---------------------------R//

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

			//R----------------------------------R//
			if(writeToFile)
			{
				if(printOnceInTerminal){
					printOnceInTerminal = 0;
                    	std::cout << "Map Update Counter:  " << updateGlobalPathCount << '\n';
					std::cout << std::to_string(last_GPS) << std::endl;
                	}
				if(updateGlobalPathCount >= 10) //bell412_dataset1 - 149|bell412_dataset5 - 136 fusion| dataset3 - 150 | dataset4 - 155
                	//if(0) //quarry1-102 | quarry2 - 132 (start-450 stop-269) | quarry3-240
                	{
		               int GTframeCountFull = 0;
		               //std::ofstream foutG("Fusion_LH.txt", std::ios::app);
		               std::ofstream foutG("Fusion_bell412_dataset1.txt", std::ios::app);
		               for(iterFull_gOdom = globalPoseMap.begin(); iterFull_gOdom != globalPoseMap.end(); iterFull_gOdom++)
		               {
		                   //read map
		                   odomPAll.x() = iterFull_gOdom->second[0];
		                   odomPAll.y() = iterFull_gOdom->second[1];
		                   odomPAll.z() = iterFull_gOdom->second[2];
		                   odomQAll.w() = iterFull_gOdom->second[3];
		                   odomQAll.x() = iterFull_gOdom->second[4];
		                   odomQAll.y() = iterFull_gOdom->second[5];
		                   odomQAll.z() = iterFull_gOdom->second[6];

		                   //calculate pose
		                   Eigen::Quaterniond globalQAll;
		                   globalQAll = WGPS_T_WVIO_viz.block<3, 3>(0, 0) * odomQAll;
		                   Eigen::Vector3d globalPAll = WGPS_T_WVIO_viz.block<3, 3>(0, 0) * odomPAll + WGPS_T_WVIO_viz.block<3, 1>(0, 3);
		                   Eigen::Matrix3d globalRAll = globalQAll.normalized().toRotationMatrix();  
		                   //std::cout << "Gloabal Rotm: " << globalRAll << std::endl;

		                   GTframeCountFull++;
		                   foutG.setf(std::ios::fixed, std::ios::floatfield);
		                   foutG.precision(0);
		                   foutG << GTframeCountFull << " ";
		                   foutG.precision(9);
					    //std::cout << std::to_string(last_GPS) << " " << ros::Time(last_GPS) << std::endl;
		                   foutG << std::to_string(last_GPS) << " " << "OK" //added time - check
		                       << globalRAll(0,0) << " "
		                       << globalRAll(0,1) << " "
		                       << globalRAll(0,2) << " "
		                       << globalPAll.x()  << " "
		                       << globalRAll(1,0) << " "
		                       << globalRAll(1,1) << " "
		                       << globalRAll(1,2) << " "
		                       << globalPAll.y()  << " "
		                       << globalRAll(2,0) << " "
		                       << globalRAll(2,1) << " "
		                       << globalRAll(2,2) << " "
		                       << globalPAll.z()  << std::endl;
		               }
               	}

			}
			//R-------------------------------R//
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
