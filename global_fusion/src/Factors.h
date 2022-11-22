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
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <cmath>

template <typename T> inline
void QuaternionInverse(const T q[4], T q_inverse[4])
{
	q_inverse[0] = q[0];
	q_inverse[1] = -q[1];
	q_inverse[2] = -q[2];
	q_inverse[3] = -q[3];
};


struct TError
{
	TError(double t_x, double t_y, double t_z, double var)
				  :t_x(t_x), t_y(t_y), t_z(t_z), var(var){}

	template <typename T>
	bool operator()(const T* tj, T* residuals) const
	{
		residuals[0] = (tj[0] - T(t_x)) / T(var);
		residuals[1] = (tj[1] - T(t_y)) / T(var);
		residuals[2] = (tj[2] - T(t_z)) / T(var);

		return true;
	}

	static ceres::CostFunction* Create(const double t_x, const double t_y, const double t_z, const double var) 
	{
	  return (new ceres::AutoDiffCostFunction<
	          TError, 3, 3>(
	          	new TError(t_x, t_y, t_z, var)));
	}

	double t_x, t_y, t_z, var;

};


struct XYError
{
	XYError(double t_x, double t_y, double t_z, double var)
				  :t_x(t_x), t_y(t_y), t_z(t_z), var(var){}

	template <typename T>
	bool operator()(const T* tj, T* residuals) const
	{
		residuals[0] = (tj[0] - T(t_x)) / T(var);
		residuals[1] = (tj[1] - T(t_y)) / T(var);

		return true;
	}

	static ceres::CostFunction* Create(const double t_x, const double t_y, const double t_z, const double var) 
	{
	  return (new ceres::AutoDiffCostFunction<
	          XYError, 2, 3>(
	          	new XYError(t_x, t_y, t_z, var)));
	}

	double t_x, t_y, t_z, var;

};

struct RError
{
	RError(double q_w, double q_x, double q_y, double q_z, double var)
				  :q_w(q_w), q_x(q_x), q_y(q_y), q_z(q_z), var(var){}

	template <typename T>
	bool operator()(const T* w_q_i, T* residuals) const
	{
		T meas_q[4];
		meas_q[0] = T(q_w); 
		meas_q[1] = T(q_x);
		meas_q[2] = T(q_y);
		meas_q[3] = T(q_z);

		T meas_q_inv[4];
		QuaternionInverse(meas_q, meas_q_inv);

		T error_q[4];
		ceres::QuaternionProduct(meas_q_inv, w_q_i, error_q); 

		residuals[0] = T(2) * error_q[1] / T(var);
		residuals[1] = T(2) * error_q[2] / T(var);
		residuals[2] = T(2) * error_q[3] / T(var);

		return true;
	}

	static ceres::CostFunction* Create(const double q_w, const double q_x, const double q_y, const double q_z, const double var) 
	{
	  return (new ceres::AutoDiffCostFunction<RError, 3, 4>(
	          	new RError(q_w, q_x, q_y, q_z, var)));
	}

	double q_w, q_x, q_y, q_z, var;

};

struct YError  //takes two quaternions and finds the yaw residual 
{
	YError(double yaw, double var)
				  :yaw(yaw), var(var){}

	template <typename T>
	bool operator()(const T* w_q_i, T* residuals) const
	{    
          //convert measured yaw to a quaternion
          T meas_q[4];
		T axis_angle[3] = {T(0), T(0), T(yaw)};
  		ceres::AngleAxisToQuaternion(axis_angle, meas_q);

  		// convert the state to a no yaw quaternion
		T q_no_yaw[4];
          T siny_cosp = T(2) * (w_q_i[0] * w_q_i[3] + w_q_i[1] * w_q_i[2]);
    		T cosy_cosp = T(1) - T(2) * (w_q_i[2] * w_q_i[2] + w_q_i[3] * w_q_i[3]);
    		T yaw_est = atan2(siny_cosp, cosy_cosp);
          T axis_angle_est[3] = {T(0), T(0), yaw_est};
		ceres::AngleAxisToQuaternion(axis_angle_est, q_no_yaw);
          
          // calculate the residual
		T meas_q_inv[4];
		QuaternionInverse(meas_q, meas_q_inv);

		T error_q[4];
		ceres::QuaternionProduct(meas_q_inv, q_no_yaw, error_q); 

		residuals[0] = T(2) * error_q[3] / T(var);

		return true;
	}

	static ceres::CostFunction* Create(const double yaw, const double var) 
	{
	  return (new ceres::AutoDiffCostFunction<YError, 1, 4>(
	          	new YError(yaw, var)));
	}

	double yaw, var;

};


struct RelativeRTError
{
	RelativeRTError(double t_x, double t_y, double t_z, 
					double q_w, double q_x, double q_y, double q_z,
					double t_var, double q_var)
				  :t_x(t_x), t_y(t_y), t_z(t_z), 
				   q_w(q_w), q_x(q_x), q_y(q_y), q_z(q_z),
				   t_var(t_var), q_var(q_var){}

	template <typename T>
	bool operator()(const T* const w_q_i, const T* ti, const T* w_q_j, const T* tj, T* residuals) const
	{
		T t_w_ij[3];
		t_w_ij[0] = tj[0] - ti[0];
		t_w_ij[1] = tj[1] - ti[1];
		t_w_ij[2] = tj[2] - ti[2];

		T i_q_w[4];
		QuaternionInverse(w_q_i, i_q_w);

		T t_i_ij[3];
		ceres::QuaternionRotatePoint(i_q_w, t_w_ij, t_i_ij);

		residuals[0] = (t_i_ij[0] - T(t_x)) / T(t_var);
		residuals[1] = (t_i_ij[1] - T(t_y)) / T(t_var);
		residuals[2] = (t_i_ij[2] - T(t_z)) / T(t_var);

		T relative_q[4];
		relative_q[0] = T(q_w);
		relative_q[1] = T(q_x);
		relative_q[2] = T(q_y);
		relative_q[3] = T(q_z);

		T q_i_j[4];
		ceres::QuaternionProduct(i_q_w, w_q_j, q_i_j);

		T relative_q_inv[4];
		QuaternionInverse(relative_q, relative_q_inv);

		T error_q[4];
		ceres::QuaternionProduct(relative_q_inv, q_i_j, error_q); 

		residuals[3] = T(2) * error_q[1] / T(q_var);
		residuals[4] = T(2) * error_q[2] / T(q_var);
		residuals[5] = T(2) * error_q[3] / T(q_var);

		return true;
	}

	static ceres::CostFunction* Create(const double t_x, const double t_y, const double t_z,
									   const double q_w, const double q_x, const double q_y, const double q_z,
									   const double t_var, const double q_var) 
	{
	  return (new ceres::AutoDiffCostFunction<
	          RelativeRTError, 6, 4, 3, 4, 3>(
	          	new RelativeRTError(t_x, t_y, t_z, q_w, q_x, q_y, q_z, t_var, q_var)));
	}

	double t_x, t_y, t_z, t_norm;
	double q_w, q_x, q_y, q_z;
	double t_var, q_var;

};
