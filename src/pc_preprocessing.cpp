/*
 * Copyright (C) 2021, Chuanbeibei Shi, Yushu Yu and Mauro Bellone
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */


#include "imageBasics.h"

void* multi_thread_preprocess(void* threadsstruct){
     Threadsstruct* manager  = (Threadsstruct*)(threadsstruct);
     int thread_id = manager->thread_id;
     //int num_outliner = 0;
     int start_index = manager->start_index;
     int end_index = manager->end_index;
     int cur_id = pc_manager.current_id;
     //vector<Eigen::Vector4d>& Pc_vector = manager->pc_vector;
     Eigen::Vector4d pc_i;
     Eigen::VectorXd pix_pc(4);
     for (int i=start_index; i< end_index; i++){
          pc_i<< cloud.points[i].x, cloud.points[i].y, cloud.points[i].z, 1;
          pix_pc = T_pc_ima*pc_i;
          
          pix_pc[0] = pix_pc[0]/pix_pc[2];
          pix_pc[1] = pix_pc[1]/pix_pc[2];

      	pointcoordinate thispoint;
          //check pix in the camera feild of view
          if(  pix_pc[0] >= 1  && (int)pix_pc[0] <= w_img-1 &&  pix_pc[1] >= 1  && (int)pix_pc[1] < h_img -1){
               //num_outliner++;
               thispoint.x_3d = pc_i.x();
          	thispoint.y_3d = pc_i.y();
          	thispoint.z_3d = pc_i.z();
          	thispoint.u_px = pix_pc[0];
          	thispoint.v_px = pix_pc[1];
          

               if  (thispoint.u_px > manager->minmaxuv_thread.umax) {manager->minmaxuv_thread.umax = thispoint.u_px;}
               if  (thispoint.u_px < manager->minmaxuv_thread.umin) {manager->minmaxuv_thread.umin = thispoint.u_px;}
               if  (thispoint.v_px > manager->minmaxuv_thread.vmax) {manager->minmaxuv_thread.vmax = thispoint.v_px;}
               if  (thispoint.v_px < manager->minmaxuv_thread.vmin) {manager->minmaxuv_thread.vmin = thispoint.v_px;}

               //int test = max(2, 4);
               //full 10x10 grid
               //manager->pc_mask_thread.emplace_back(thispoint, grid);
               //这样会有问题吗？
               pc_manager.mask_win[cur_id].pc_masks[thread_id].emplace_back(thispoint, grid);
          }
     }
     return threadsstruct;
}


void poseCallback(const  geometry_msgs::PoseStamped::ConstPtr& msg)
{
     geometry_msgs::PoseStamped posek;

     posek = *msg;
     //pose_series.push(posek);
	// pose_global.pose = msg->pose;
     //pose_global = *msg;
     p_drone = Vector3d(posek.pose.position. x,posek.pose.position.y, posek.pose.position.z );
     q_drone = Quaterniond(posek.pose.orientation.w, posek.pose.orientation.x, posek.pose.orientation.y, posek.pose.orientation.z);
     pose_timestamp = posek.header.stamp.toNSec();
}

double t_v_last = 0;
double t_v_cur = 0;
bool first_msg = true;
void velCallback(const  geometry_msgs::TwistStamped::ConstPtr& msg)
{    
     TicToc t_predict;
     // ROS_DEBUG_STREAM("velcallback");
     geometry_msgs::TwistStamped velk;
     velk = *msg;
     t_v_last = t_v_cur;
     t_v_cur = velk.header.stamp.toSec();
     if(first_msg){
          first_msg = false;
          return;
     }
     //int grid_ave = 50;
     
     int sum_no = 0; // the total points near the feature point
     //ekf predict part
     if(ifdetection == 1){
          //initialization flags
          flag_int_xkk_last = flag_int_xkk;
          flag_int_xkk = 1;
          //initial state
          if ((flag_int_xkk_last == 0) & (flag_int_xkk == 1)){
               x_k_k = p_f_L;  //initialization of the variable in kalman filter
          }
          if (flag_int_xkk == 1){               
               Eigen::Vector3d omega_s,  v_s; //angular velocity of the sensor relative to earth, and the velocity relative to earth
               double deltat = t_v_cur - t_v_last;  //sampling time
               // ROS_INFO_STREAM("dt = "<<deltat);

               omega_s << velk.twist.angular.x, velk.twist.angular.y, velk.twist.angular.z;
               v_s << velk.twist.linear.x, velk.twist.linear.y, velk.twist.linear.z;

               Eigen::Matrix3d omega_s_hat(3,3);  //rotation matrix of sensor relative to earth frame.
               omega_s_hat << 0, -omega_s(2), omega_s(1),
                              omega_s(2), 0, -omega_s(0),
                              -omega_s(1), omega_s(0), 0;
               
               // R_variance  << 0, 0, 0, 0, 0, 0,
               //                0, 0, 0, 0, 0, 0,
               //                0, 0, 0, 0, 0, 0,
               //                0, 0, 0, 0, 0, 0,
               //                0, 0, 0, 0, 0, 0,
               //                0, 0, 0, 0, 0, 0;
               
               //need to be adjusted
               double cox = 0.1;
               R_variance  << cox, 0, 0, 0, 0, 0,
                              0, cox, 0, 0, 0, 0,
                              0, 0, cox, 0, 0, 0,
                              0, 0, 0, cox, 0, 0,
                              0, 0, 0, 0, cox, 0,
                              0, 0, 0, 0, 0, cox;
               

               Eigen::Matrix3d p_hat(3,3);
               p_hat << 0, -x_k_k(2), x_k_k(1),
                         x_k_k(2), 0, -x_k_k(0),
                         -x_k_k(1), x_k_k(0), 0;

               G_T =  -deltat*omega_s_hat+eye3;//f(p)对p求导
               Eigen::MatrixXd   H_T(3,6);
               H_T << p_hat, -eye3;
               H_T = -deltat*H_T;
               //C_T = eye3;
               Eigen::VectorXd  u_k(6);  //the velocity of sensor frame relative to the earth frame, in the sensor frame
               u_k << omega_s, v_s;
               //x_k_k = G_T*x_k_k + H_T* u_k;  //linear
               x_k_k = deltat*(x_k_k.cross(omega_s) - v_s) + x_k_k;  //non-linear    预测的部分   速度和角速度用的是local_velocity_body   x_k_k is uav's position in lidar frame
               P_k_k = G_T*P_k_k*G_T.transpose() + H_T*R_variance*H_T.transpose();
               
               Vector3d p_cam = (R_cam_lidar*x_k_k + t_cam_lidar);// uav's position in camera frame
               Vector3d project_uav = K_in*p_cam;
               //update featur_point
               // ROS_INFO("update featur_point");
               m_feature.lock();
               feat_point[0] = project_uav.x()/project_uav.z();
               feat_point[1] = project_uav.y()/project_uav.z();
               m_feature.unlock();
               //p_b and t_bc is virtual setting in HILS, need apply R_cam_imu and t_cam_imu in real fly.
               // Vector3d p_b = q_bc*p_cam+t_bc;//uav's position in body frame
               Vector3d p_b = R_cam_imu.inverse()*p_cam - R_cam_imu.inverse()*t_cam_imu;//uav's position in body frame
               p_b = q_drone*p_b + p_drone;//world frame
               geometry_msgs::PointStamped p_box;
               p_box.point.x = p_b.x();
               p_box.point.y = p_b.y();
               p_box.point.z = p_b.z();
               ROS_INFO_STREAM("yolo_depth_estimate = "<<x_k_k.x()<<", "<<x_k_k.y()<<", "<<x_k_k.z());
               v_ekf.publish(p_box);
          }
          
     }
     ROS_DEBUG_STREAM("vekf predict: "<<t_predict.toc()<<" ms");
}

void detectCallback(const  darknet_ros_msgs::BoundingBoxes::ConstPtr& msg){
     darknet_ros_msgs::BoundingBoxes boundingBoxesResults_ = *msg;
     int boxno = boundingBoxesResults_.bounding_boxes.size();
     long long cur_yolo_timestamp = msg->header.stamp.toNSec();
     // ROS_INFO_STREAM("timestamp = "<<cur_yolo_timestamp);
     Pc_Vector yolo_cur_pc_data;
     if(boxno == 0){
          ROS_ERROR("nox = 0 !!!!!");
          return;
     }
     if(boxno != 1){
          ROS_ERROR("wrong detection result! return");
          return;
     }
     if(last_yolo_timestamp == cur_yolo_timestamp){
          return;
     }
     //data match
     while(!yolo_depth.empty()){
          ROS_INFO_STREAM("yolo_depth.front().timestamp = "<<yolo_depth.front().timestamp);
          if(cur_yolo_timestamp >= yolo_depth.front().timestamp){
               if(cur_yolo_timestamp == yolo_depth.front().timestamp){
                    yolo_cur_pc_data = yolo_depth.front();
                    data_matched = true;
                    ROS_INFO("data matched!!!");
                    last_yolo_timestamp = cur_yolo_timestamp;
                    break;
               }else{
                    yolo_depth.pop();
               }
          }else{
               ROS_ERROR("The matched data has been poped out!");
          }
     }
     if(yolo_depth.empty() && !data_matched){
          ROS_ERROR("you need extend your queue size!");
     }
     
     if(data_matched){
          TicToc t_update;
          //calculate the 3D position of yolo detection object(uav) in camera frame with matched pc data   
          double u0 = (boundingBoxesResults_.bounding_boxes[0].xmin + boundingBoxesResults_.bounding_boxes[0].xmax)/2;//yolo输出的u，v是输入图像的u，v，没有经过缩放的
          double v0 = (boundingBoxesResults_.bounding_boxes[0].ymin + boundingBoxesResults_.bounding_boxes[0].ymax)/2;
          int w_u = (boundingBoxesResults_.bounding_boxes[0].xmax - boundingBoxesResults_.bounding_boxes[0].xmin)/2.0;
          int w_v = (boundingBoxesResults_.bounding_boxes[0].ymax - boundingBoxesResults_.bounding_boxes[0].ymin)/2.0;
          int search_radius = (w_u > w_v)? w_u : w_v;
          sigma_feature[0] = boundingBoxesResults_.bounding_boxes[0].utx;
          sigma_feature[1] = boundingBoxesResults_.bounding_boxes[0].uty;
          ifdetection = 1;
          Vector3d uav_position_incamframe = calculate_yolo_depth(yolo_cur_pc_data.pc_cam_3d, yolo_cur_pc_data.pc_uv, u0, v0, search_radius*1.2); 
          ROS_INFO_STREAM("yolo_depth_no_rect = "<<uav_position_incamframe.x()<<", "<<uav_position_incamframe.y()<<", "<<uav_position_incamframe.z());
          // Vector3d rect_project_uav = K_in*uav_position_incamframe;
          // rect_feat_point[0] = rect_project_uav.x()/rect_project_uav.z();
          // rect_feat_point[1] = rect_project_uav.y()/rect_project_uav.z();
          Vector3d uav_position_inlidframe = R_cam_lidar.transpose()*uav_position_incamframe - R_cam_lidar.transpose()*t_cam_lidar;
          ROS_INFO_STREAM("yolo_depth in lidar frame = "<<uav_position_inlidframe.x()<<", "<<uav_position_inlidframe.y()<<", "<<uav_position_inlidframe.z());
          //position correction with uav's own relative pose
          Vector3d uav_pose_bodyframe = R_cam_imu.transpose()*uav_position_incamframe - R_cam_imu.transpose()*t_cam_imu;
          Vector3d uav_pose_worldframe = yolo_cur_pc_data.q_wb*uav_pose_bodyframe + yolo_cur_pc_data.t_wb;
          // uav_pose_bodyframe = q_drone.inverse()*uav_pose_worldframe - q_drone*p_drone;//rected uav position in body frame
          // Vector3d uav_position_incamframe_rect = R_cam_imu*uav_pose_bodyframe + t_cam_imu;//rected uav position in camera frame
          // Vector3d uav_position_inlidframe_rect = R_imu_lidar.transpose()*uav_pose_bodyframe - R_imu_lidar.transpose()*t_imu_lidar;//rected uav position in lidar frame
          // Vector3d rect_project_uav = K_in*uav_position_incamframe_rect;
          rect_feat_point[0] = uav_pose_worldframe.x();//因为不知道下一帧pc的时间戳，所以在pc收到的时候在转到pc对应的lidar系下
          rect_feat_point[1] = uav_pose_worldframe.y();
          rect_feat_point[2] = uav_pose_worldframe.z();
          
          //初始化阶段
          if(flag_int_xkk == 0){
               p_f_L = uav_position_inlidframe;//initial value for vekf
               //update feature point        
               Vector3d project_uav = K_in*uav_position_incamframe;
               m_feature.lock();
               feat_point[0] = project_uav.x()/project_uav.z();
               feat_point[1] = project_uav.y()/project_uav.z();
               m_feature.unlock();
               return;
          }

          //ROS_INFO("you catch me!!!!");

          /**************************************************************/
          //ekf update part
          double x_f_l = uav_position_inlidframe.x();

          Eigen::Matrix3d fp_tra(3,3);   //see notebook
          fp_tra = -R_cam_lidar;
          Vector3d tmp = Vector3d((feat_point[0]-343.76)/264, (feat_point[1] -183.8795)/263.70, 1.0);
          fp_tra.block<3,1>(0,0) =tmp;
          // ROS_INFO_STREAM("fp_tra = \n"<<fp_tra.matrix());
          
          Eigen::Vector3d fp_vec(3);
          Eigen::Vector3d r1(3);
          r1 = R_cam_lidar.block<3,1>(0,0);
          
          Eigen::Vector3d t_c_L = t_cam_lidar;
          fp_vec = x_f_l*r1 + t_c_L;
          Eigen::Vector3d v_mi;
          v_mi = fp_tra.inverse()*fp_vec;//求解的【cz，y，z】
          double z_f_c = v_mi(0);//相机坐标系下的深度值
          double x_f_c = (feat_point[0] - 343.76)* z_f_c /264;//
          double y_f_c = (feat_point[1] - 183.8795)* z_f_c /263.700;
          
          p_f_L << x_f_l, v_mi(1), v_mi(2);//雷达系下的目标位置
          ROS_INFO_STREAM("yolo_scbb = "<<p_f_L.x()<<", "<<p_f_L.y()<<", "<<p_f_L.z());
          p_f_L = uav_position_inlidframe;
          

          //the jacobian matrix for covariance calculation
          Eigen::Matrix3d pApu, pApv;
          pApu <<   1/fx, 0, 0,
                    0, 0, 0,
                    0, 0, 0;
          pApv <<   0, 0, 0,
                    1/fy, 0, 0,
                    0, 0, 0;
          Eigen::Vector3d pfpuvx1 =  -fp_tra.inverse()*pApu*fp_tra.inverse()*fp_vec;
          Eigen::Vector3d pfpuvx2 =  -fp_tra.inverse()*pApv*fp_tra.inverse()*fp_vec;
          Eigen::Vector3d pfpuvx3 =  -fp_tra.inverse()*(r1 + t_cam_lidar);
          Eigen::Matrix3d pfpuvx;
          pfpuvx << pfpuvx1,pfpuvx2, pfpuvx3;

          Eigen::MatrixXd pfpuvx23row(2,3);
          pfpuvx23row = pfpuvx.block<2,3>(1,0);

          Eigen::Matrix3d vari_pix_z;
          vari_pix_z <<  0.1, 0, 0,
                         0, sigma_feature[0], 0,
                         0, 0, sigma_feature[1];  //from the yoloros, the uncertainty
          // else
          //      vari_pix_z << 0.1, 0, 0,
          //                          0, 0.18, 0,
          //                          0, 0, 0.18;  //from the yoloros, the uncertainty为什么没有观测的时候也做更新？？？？

          Q_variance(0,0) = 0.1;  //the variance of x component.
          Q_variance.block<2,2>(1,1) = pfpuvx23row*vari_pix_z*pfpuvx23row.adjoint();
          // cout << "Q_variance" << Q_variance << endl;
          Eigen::Vector3d y, z_k;
          Eigen::Matrix3d  S, K;
          z_k = p_f_L; //output from yolov3
          y = z_k - C_T*x_k_k;   //nonlinear
          S=C_T*P_k_k*C_T.transpose() + Q_variance; //observation
     // S = 0.01*eye3;
          K = P_k_k*C_T.transpose()*S.inverse();
     // K = 0.1*eye3;
          x_k_k = x_k_k + K*y;
          P_k_k=( eye3 -K*C_T)*P_k_k;
          Vector3d p_cam = (R_cam_lidar*x_k_k + t_cam_lidar);// uav's position in camera frame
          Vector3d project_uav = K_in*p_cam;
          //update featur_point
          m_feature.lock();
          feat_point[0] = project_uav.x()/project_uav.z();
          feat_point[1] = project_uav.y()/project_uav.z();
          m_feature.unlock();
          //p_b and t_bc is virtual setting in HILS, need apply R_cam_imu and t_cam_imu in real fly.
          // Vector3d p_b = q_bc*p_cam+t_bc;//uav's position in body frame
          Vector3d p_b = R_cam_imu.inverse()*p_cam - R_cam_imu.inverse()*t_cam_imu;//uav's position in body frame
          p_b = q_drone*p_b + p_drone;//world frame
          geometry_msgs::PointStamped p_box;
          p_box.point.x = p_b.x();
          p_box.point.y = p_b.y();
          p_box.point.z = p_b.z();
          ROS_INFO_STREAM("yolo_depth_estimate = "<<x_k_k.x()<<", "<<x_k_k.y()<<", "<<x_k_k.z());
          v_ekf.publish(p_box);
          ROS_DEBUG_STREAM("update time : "<<t_update.toc()<<" ms");
     }
}


//原来的代码
// void velCallback(const  geometry_msgs::TwistStamped::ConstPtr& msg, int a)
// {
// 	 geometry_msgs::TwistStamped velk;

// 	 velk = *msg;

//      int grid_ave = 50;
//      int w_img = 3;
//      double sum_x = 0;
//      int sum_no = 0; // the total points near the feature point
// 	 //calculate the x coordinate of the feature point
// 	 /*for (int u =   (int)(feat_point[0]- grid_ave); u <   (int)(feat_point[0]+ grid_ave); u++)
// 	      for (int v =   (int)(feat_point[1]- grid_ave); v < (int)(feat_point[1] + grid_ave); v++)
// 	    	  for (int i_pc =  0; i_pc < pc_array_grid[v*w_img+u].size(); i_pc++){
// 	    		  sum_x =  pc_array_grid[v*w_img+u][i_pc].x_3d + sum_x;
// 	    		  sum_no++;
// 	    	  }*/

//      if (pc_array_feature.size() == 0){
//     	 cerr << "No point near the feature region." << endl;
//     	 sum_no = 1;
//      }
//      else{

//      for (int i_pc =  0; i_pc < pc_array_feature.size(); i_pc++){
//      	    		  sum_x = pc_array_feature[i_pc].x_3d + sum_x;
//      	    		  sum_no++;
//      	    	  }
//      if (pc_array_feature.size() == 0){
//     	 cerr << "No point near the feature region." << endl;
//     	 sum_no = 1;
//      }
// 	 double x_f_l = sum_x/sum_no;
// 	 sum_no  = 0;
// 	 sum_x = 0;

// 	 //cout << "feature point in LiDAR frame, x coordinate: " << x_f_l << endl;
// 	 int ii = feat_point[1]*w_img+feat_point[0];
// 	 cout << "Count of point near the feature region: " << pc_array_feature.size() << endl;

//      /*projection matrix:
// 	 264	0	343.760000000000
// 	 0	263.700000000000	183.879500000000
// 	 0	0	1 */

//      /*lidar to cam matrix:
//      0.0101012000000000	-0.998801000000000	0.0479053000000000	0.155312000000000
// -0.195541000000000	-0.0489563000000000	-0.979473000000000	-0.0598381000000000
// 0.980644000000000	0.000526794000000000	-0.195801000000000	0.283747000000000
// 0	0	0	1
//       */

// 	 double fx, fy, cx, cy; //the intrinsic parameters of the camera
// 	 fx = 264.0;
// 	 fy = 263.700000000000;
// 	 cx = 343.760000000000;
// 	 cy = 183.879500000000;

// 	 Eigen::Matrix3d fp_tra(3,3);   //see notebook
//      fp_tra << (feat_point[0]-343.76)/264,   0.998801000000000,  -0.0479053000000000,
//     		 (feat_point[1] -183.8795)/263.70,   0.0489563000000000,  0.979473000000000,
//                   1,   -0.000526794000000000,  0.195801000000000;




//      Eigen::Vector3d fp_vec(3);
//      Eigen::Vector3d r1(3);
//      r1 <<  0.0101012000000000,  -0.195541000000000,  0.980644000000000;
//      Eigen::Vector3d t_c_L;
// 	 t_c_L <<  0.155312000000000, -0.0598381000000000,  0.283747000000000;
//      fp_vec = x_f_l*r1 + t_c_L;
//      Eigen::Vector3d v_mi;
//      v_mi = fp_tra.inverse()*fp_vec;//求解的【cz，y，z】

//      //the jacobian matrix for covariance calculation
//      Eigen::Matrix3d pApu, pApv;
//      pApu << 1/fx,  0,  0,
//          		 0, 0, 0,
//      			 0, 0, 0;
//      pApv << 0,  0,  0,
//     		 1/fy, 0, 0,
//           			 0, 0, 0;
//      Eigen::Vector3d pfpuvx1 =  -fp_tra.inverse()*pApu*fp_tra.inverse()*fp_vec;
//      Eigen::Vector3d pfpuvx2 =  -fp_tra.inverse()*pApv*fp_tra.inverse()*fp_vec;
//      Eigen::Vector3d pfpuvx3 =  -fp_tra.inverse()*(r1 + t_c_L);
//      Eigen::Matrix3d pfpuvx;
//      pfpuvx << pfpuvx1,pfpuvx2, pfpuvx3;
//      Eigen::MatrixXd pfpuvx23row(2,3);
//      pfpuvx23row = pfpuvx.block<2,3>(1,0);

//      double z_f_c = v_mi(0);//相机坐标系下的深度值
//      double x_f_c = (feat_point[0] - 343.76)* z_f_c /264;//
//      double y_f_c = (feat_point[1] - 183.8795)* z_f_c /263.700;
     
//      p_f_L << x_f_l, v_mi(1), v_mi(2);//雷达系下的目标位置
//      p_f_c << x_f_c, y_f_c, z_f_c;//像素坐标乘以深度（camera系下的深度）


//      flag_int_xkk_last = flag_int_xkk;
//      flag_int_xkk = 1;

//      cout << "feature point in LiDAR frame: " << p_f_L(0)  << ", " << p_f_L(1)  << ", "   << p_f_L(2)  << endl;
//      cout << "feature point in camera frame: " << p_f_c(0)  << ", "  << p_f_c(1) << ", "  << p_f_c(2)  << endl;

//      //the covariance calculation:

//      if ((flag_int_xkk_last == 0) & (flag_int_xkk == 1)){
//     	 x_k_k = p_f_L;  //initialization of the variable in kalman filter
//      }

//      if (flag_int_xkk == 1){
//      Eigen::MatrixXd J_M(2,3);
//      J_M << 1/p_f_c(2)*fx , 0,  -fx*p_f_c(0)/(p_f_c(2)* p_f_c(2)),
//     		 0,  1/p_f_c(2)*fy,  -fy*p_f_c(1)/(p_f_c(2)* p_f_c(2));


// //     G_T = [eye(3,3), t_est*eye(3,3); zeros(3,3), eye(3,3)]; %state transition matrix
// //     H_T = [0.5*t_est*t_est*eye(3,3); t_est*eye(3,3)];  %input matrix
// //     C_T = [eye(3,3), zeros(3,3)]; %output matrix
// //
// //     k_input = round(k*t_est/t_input);
// //     if k_input > size_T_input
// //         u_k = u_array_noise(1:3, end);
// //     else
// //         u_k = u_array_noise(1:3, k_input);
// //     end
// //
// //     k_ob = round(k*t_est/t_ob_global);
// //     if k_ob > size_T_ob
// //         z_k = z_array(1:3, end);
// //     elseif k_ob ==0
// //         z_k = z_array(1:3, 1);
// //     else
// //         z_k = z_array(1:3, k_ob);
// //     end
// //
// //     x_k_k_1 = G_T*x_k_k + H_T* u_k;
// //     P_k_k_1 = G_T*P_k_k*G_T' + H_T*R*H_T';
// //
// //     y = z_k -C_T*x_k_k_1;
// //     S=C_T*P_k_k_1*C_T'+ Q; %observation
// //     K=P_k_k_1*C_T'*inv(S);
// //     x_k_k=x_k_k_1+K*y;
// //     P_k_k=(eye(6,6)-K*C_T)*P_k_k_1;
// //     x_array_est(:,k)=x_k_k;

     
//      Eigen::Vector3d omega_s,  v_s; //angular velocity of the sensor relative to earth, and the velocity relative to earth
//      double deltat = 0.0333333;  //sampling time
//      eye3.setIdentity();
//      omega_s << velk.twist.angular.x, velk.twist.angular.y, velk.twist.angular.z;
//      v_s << velk.twist.linear.x, velk.twist.linear.y, velk.twist.linear.z;

//      //omega_s = Eigen::Vector3d::Zero();
//      //v_s = Eigen::Vector3d::Zero();
//      //omega_s = Eigen::Vector3d::Random(3,1);
//      //v_s = Eigen::Vector3d::Random(3,1);

//      Eigen::Matrix3d omega_s_hat(3,3), R_s_E;  //rotation matrix of sensor relative to earth frame.
     
//      R_variance  <<  0, 0, 0, 0, 0, 0,
//     		 0, 0, 0, 0, 0, 0,
// 			 0, 0, 0, 0, 0, 0,
// 			 0, 0, 0, 0, 0, 0,
// 			 0, 0, 0, 0, 0, 0,
// 			 0, 0, 0, 0, 0, 0;

//      R_variance  <<  0.1, 0, 0, 0, 0, 0,
//          		 0, 0.1, 0, 0, 0, 0,
//      			 0, 0, 0.1, 0, 0, 0,
//      			 0, 0, 0, 0.1, 0, 0,
//      			 0, 0, 0, 0, 0.1, 0,
//      			 0, 0, 0, 0, 0, 0.1;

//      Eigen::Matrix3d J_Mtinv;
//      J_Mtinv = J_M.transpose()*J_M;
//     // J_Mtinv = J_Mtinv.inverse();  //needs to be revised, current equation cannot be inverted, Dec, 25, 2021
//      double sigma_var = 0.1; //the variance of the feature points in pixel frame
//      //Q_variance = sigma_var*sigma_var*J_Mtinv; //see notebook
//      //Q_variance = Eigen::Matrix3d::Random(3,3);  //how to solve it?

//      Eigen::Matrix3d vari_pix_z;
//      if (ifdetection == 1)
//          vari_pix_z << 0.1, 0, 0,
//                    0, sigma_feature[0], 0,
// 				   0, 0, sigma_feature[1];  //from the yoloros, the uncertainty
//      else
//     	 vari_pix_z << 0.1, 0, 0,
//     	                    0, 0.18, 0,
//     	 				   0, 0, 0.18;  //from the yoloros, the uncertainty

//      Q_variance(0,0) = 0.1;  //the variance of x component.
//      Q_variance.block<2,2>(1,1) = pfpuvx23row*vari_pix_z*pfpuvx23row.adjoint();
//      cout << "Q_variance" << Q_variance << endl;

     
//      Eigen::Quaterniond q_3(pose_global.pose.orientation.w,pose_global.pose.orientation.x ,pose_global.pose.orientation.y,pose_global.pose.orientation.z);
//      q_3.normalize();
//      R_s_E = q_3.toRotationMatrix();

// 	 omega_s_hat << 0, -omega_s(2), omega_s(1),
//     		 omega_s(2), 0, -omega_s(0),
//              -omega_s(1), omega_s(0), 0;

// 	 Eigen::Matrix3d p_hat(3,3);
// 	 p_hat << 0, -x_k_k(2), x_k_k(1),
// 			 x_k_k(2), 0, -x_k_k(0),
// 			 -x_k_k(1), x_k_k(0), 0;

//      G_T =  -deltat*omega_s_hat+eye3;
//      Eigen::MatrixXd   H_T(3,6);
//      H_T << p_hat, -eye3;
//      H_T = -deltat*H_T;
//      C_T = eye3;
//      Eigen::VectorXd  u_k(6);  //the velocity of sensor frame relative to the earth frame, in the sensor frame
//      u_k << omega_s, v_s;
//      //x_k_k = G_T*x_k_k + H_T* u_k;  //linear
//      x_k_k = deltat*(x_k_k.cross(omega_s) - v_s) + x_k_k;  //non-linear    预测的部分   速度和角速度用的是local_velocity_body
//      P_k_k = G_T*P_k_k*G_T.transpose() + H_T*R_variance*H_T.transpose();    

// //      Eigen::Vector3d y, z_k;

// //      z_k = p_f_L; //output from yolov3
// //      y = z_k - C_T*x_k_k;   //nonlinear
// //      S=C_T*P_k_k*C_T.transpose() + Q_variance; //observation
// //     // S = 0.01*eye3;
// //      K = P_k_k*C_T.transpose()*S.inverse();
// //     // K = 0.1*eye3;
// //      x_k_k = x_k_k + K*y;
// //      P_k_k=( eye3 -K*C_T)*P_k_k;

// // //     cout << "input: " << v_s << omega_s << endl;
// // //     cout << "S:" << S <<endl;
// // //     cout << "P_k_k:" << P_k_k <<endl;
// //      cout << "z_k:" << z_k <<endl;
// // //     cout << " C_T:" << C_T << endl;
// // //     cout << " G_T:"  << G_T << endl;
// // //     cout << "H_T: " << H_T << endl;
// // //     cout << "K:" << K << endl;
// //      cout << "y in Kalman filter: " << y(0)  << ", " << y(1)  << ", "   << y(2)  << endl;
// //      cout << "After Kalman filter: " << x_k_k(0)  << ", " << x_k_k(1)  << ", "   << x_k_k(2)  << endl;
//           }
//      }
// }






int main(int argc, char **argv)
{
     ros::init(argc, argv, "pc_preprocessing");
     ros::NodeHandle n("~");
     ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
     ros::Subscriber subpc = n.subscribe("/livox/lidar", 20, pc2Callback);
     ros::Subscriber subimg = n.subscribe("/zed2/zed_node/left/image_rect_color", 1, imgCallback);
     ros::Subscriber subpos = n.subscribe("/mavros/local_position/pose", 1000, poseCallback);
     ros::Subscriber subvel = n.subscribe("/mavros/local_position/velocity_body", 1000, velCallback);

     ros::Subscriber subdetection = n.subscribe("/darknet_ros/bounding_boxes", 1000, detectCallback);  //dependency: darknet_ros_msgs

     pubimg = n.advertise<sensor_msgs::Image>("/camera/rgb/image_raw",  1000);
     pubimg_upsample = n.advertise<sensor_msgs::Image>("/camera/xyz/image_upsampling",  1000);
     v_ekf = n.advertise<geometry_msgs::PointStamped>("/vekf",100);
     K_in<<
        264.0, 0, 343.76,
        0, 263.7, 183.8795,
        0, 0, 1;
     R_cam_lidar<<
                    0.0293523,  -0.999117,  0.0300603,
                    -0.0142596,  -0.0304887,  -0.999433,
                    0.999467,  0.028907,  -0.015142;
     t_cam_lidar = Vector3d(0.108313, 0.0907808, 0.384258);
     
     /*******************************************************/
     Matrix4d T_imu_lidar = Matrix4d::Identity();
     Matrix4d T_lidar_imu = Matrix4d::Identity();
     Matrix4d T_cam_imu = Matrix4d::Identity();

     Vector3d t_Imu_Lidar = Vector3d(-0.352103, -0.057517, -0.087161);
     t_cam_imu = Vector3d(0.05813448, -0.10000966, -0.09331424);

     //calibration results by imu-livox calibration tool
     // R_Imu_Lidar<<0.998553, -0.052115, -0.013263,
     //             0.051144,  0.996558, -0.065240,
     //             0.016617,  0.064468,  0.997781;
     R_cam_imu<<0.03312962, -0.99918091, -0.02323669,
                    -0.01556299,  0.0227309,  -0.99962048,
                    0.99932989,  0.03347868, -0.01479718;
     
     
     R_imu_lidar = R_cam_imu.transpose()*R_cam_lidar;
     t_imu_lidar = R_cam_imu.transpose()*t_cam_lidar -R_cam_imu.transpose()*t_cam_imu;
     ROS_INFO_STREAM("R_imu_lidar = \n"<<R_imu_lidar.matrix());
     ROS_INFO_STREAM("t_imu_lidar = ("<<t_imu_lidar.x()<<", "<<t_imu_lidar.y()<<", "<<t_imu_lidar.z()<<")");
     T_imu_lidar.block<3,3>(0,0) = R_imu_lidar;
     T_imu_lidar.block<3,1>(0,3) = t_imu_lidar;
     T_lidar_imu.block<3,3>(0,0) = R_imu_lidar.inverse();
     T_lidar_imu.block<3,1>(0,3) = R_imu_lidar.inverse() * -t_imu_lidar;
     /********************************************************/
     



     T_pc_ima = T_pc_ima.setIdentity();
     T_pc_ima.block<3,3>(0,0) = K_in*R_cam_lidar;
     T_pc_ima.block<3,1>(0,3) = K_in*t_cam_lidar;
     T_cam_lidar = T_cam_lidar.setIdentity();
     T_cam_lidar.block<3,3>(0,0) = R_cam_lidar;
     T_cam_lidar.block<3,1>(0,3) = t_cam_lidar;


     std::thread prepro_pc = std::thread(&Preprocess);
     ros::spin();
     //prepro_pc.join();
     return 0;
}
