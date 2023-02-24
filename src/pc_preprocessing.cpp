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
//计算平面内两点之间的距离
float squareDistance(point a, point b)
{
	return sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
}
void DBSCAN(vector<point> &dataset, float Eps, int MinPts)
{
	int count = 0;
	int len = dataset.size();
	//calculate pts
	// cout << "计算各点的邻域数量" << endl;

	for (int i = 0; i < len; i++)
	{
		//特别注意 !!! 这里如果j从i开始，表明某点的邻域范围内样本数量包含自己，若j从i+1开始则不包含自己。
		for (int j = i; j < len; j++)
		{
			if (squareDistance(dataset[i], dataset[j]) < Eps)
			{
				dataset[i].pts++;
				dataset[j].pts++;
			}
		}
	}

	//core point ，若某个点在其领域Eps范围内的点个数>=MinPts，称该点为core point核心点
	// cout << "寻找核心点" << endl;
	//核心点集合索引（索引为样本点原本的索引，从0开始）
	vector<int> corePtInxVec;
	for (int i = 0; i < len; i++)
	{
		if (dataset[i].pts >= MinPts)
		{
			dataset[i].pointType = CORE;
               //z这里只要是核心点就分配一个cluster
			dataset[i].cluster = (++count);//返回加一后的值
			corePtInxVec.push_back(i);
			// printf("样本(%.1f, %.1f)的邻域点数量为:%d,被确立为核心点, cluster: %d\n", dataset[i].x, dataset[i].y, dataset[i].pts, dataset[i].cluster);
		}
	}

	//合并core point
	// cout << "合并核心点" << endl;
	for (int i = 0; i < corePtInxVec.size(); i++)
	{
		for (int j = i + 1; j < corePtInxVec.size(); j++)
		{
			//对所有的corepoint，将其eps范围内的core point下标添加到vector<int> corepts中
			if (squareDistance(dataset[corePtInxVec[i]], dataset[corePtInxVec[j]]) < Eps)
			{
				dataset[corePtInxVec[i]].neighborCoreIdx.push_back(corePtInxVec[j]);
				dataset[corePtInxVec[j]].neighborCoreIdx.push_back(corePtInxVec[i]);

				// printf("核心点%.1f, %.1f)与核心点%.1f, %.1f)处在半径范围内，相互连接，可以合并\n",
					//    dataset[corePtInxVec[i]].x, dataset[corePtInxVec[i]].y, dataset[corePtInxVec[j]].x, dataset[corePtInxVec[j]].y);
			}
		}
	}

	//对于所有的corepoint，采用深度优先的方式遍历每个core point的所有corepts，使得相互连接的core point具有相同的cluster编号
     //把联通的核心点cluster设为唯一数
	for (int i = 0; i < corePtInxVec.size(); i++)
	{
		for (int j = 0; j < dataset[corePtInxVec[i]].neighborCoreIdx.size(); j++)
		{
			int idx = dataset[corePtInxVec[i]].neighborCoreIdx[j];
			dataset[idx].cluster = dataset[corePtInxVec[i]].cluster;
		}
	}

	//不属于核心点但在某个核心点的邻域内的点叫做边界点
	// cout << "边界点，把边界点加入到靠近的核心点" << endl;
	//border point,joint border point to core point
	for (int i = 0; i < len; i++)
	{
		if (dataset[i].pointType == CORE) //忽略核心点
			continue;

		for (int j = 0; j < corePtInxVec.size(); j++)
		{
			int idx = corePtInxVec[j]; //核心点索引
			if (squareDistance(dataset[i], dataset[idx]) < Eps)
			{
				dataset[i].pointType = BORDER;
				dataset[i].cluster = dataset[idx].cluster;
				// printf("样本(%.1f, %.1f)被确立为边界点, cluster: %d\n", dataset[i].x, dataset[i].y, dataset[i].cluster);
				break;
			}
		}
	}

}






// void* multi_thread_preprocess(void* threadsstruct){
//      Threadsstruct* manager  = (Threadsstruct*)(threadsstruct);
//      int thread_id = manager->thread_id;
//      //int num_outliner = 0;
//      int start_index = manager->start_index;
//      int end_index = manager->end_index;
//      int cur_id = pc_manager.current_id;
//      //vector<Eigen::Vector4d>& Pc_vector = manager->pc_vector;
//      Eigen::Vector4d pc_i;
//      Eigen::VectorXd pix_pc(4);
//      for (int i=start_index; i< end_index; i++){
//           pc_i<< cloud.points[i].x, cloud.points[i].y, cloud.points[i].z, 1;
//           pix_pc = T_pc_ima*pc_i;
          
//           pix_pc[0] = pix_pc[0]/pix_pc[2];
//           pix_pc[1] = pix_pc[1]/pix_pc[2];

//       	pointcoordinate thispoint;
//           //check pix in the camera feild of view
//           if(  pix_pc[0] >= 1  && (int)pix_pc[0] <= w_img-1 &&  pix_pc[1] >= 1  && (int)pix_pc[1] < h_img -1){
//                //num_outliner++;
//                thispoint.x_3d = pc_i.x();
//           	thispoint.y_3d = pc_i.y();
//           	thispoint.z_3d = pc_i.z();
//           	thispoint.u_px = pix_pc[0];
//           	thispoint.v_px = pix_pc[1];
          

//                if  (thispoint.u_px > manager->minmaxuv_thread.umax) {manager->minmaxuv_thread.umax = thispoint.u_px;}
//                if  (thispoint.u_px < manager->minmaxuv_thread.umin) {manager->minmaxuv_thread.umin = thispoint.u_px;}
//                if  (thispoint.v_px > manager->minmaxuv_thread.vmax) {manager->minmaxuv_thread.vmax = thispoint.v_px;}
//                if  (thispoint.v_px < manager->minmaxuv_thread.vmin) {manager->minmaxuv_thread.vmin = thispoint.v_px;}

//                //int test = max(2, 4);
//                //full 10x10 grid
//                //manager->pc_mask_thread.emplace_back(thispoint, grid);
//                //这样会有问题吗？
//                pc_manager.mask_win[cur_id].pc_masks[thread_id].emplace_back(thispoint, grid);
//           }
//      }
//      return threadsstruct;
// }


void poseCallback(const  geometry_msgs::PoseStamped::ConstPtr& msg)
{
     TicToc pose_callback_t;
     geometry_msgs::PoseStamped posek;

     posek = *msg;
     //pose_series.push(posek);
	// pose_global.pose = msg->pose;
     //pose_global = *msg;
     m_state.lock();
     p_drone = Vector3d(posek.pose.position. x,posek.pose.position.y, posek.pose.position.z );
     q_drone = Quaterniond(posek.pose.orientation.w, posek.pose.orientation.x, posek.pose.orientation.y, posek.pose.orientation.z);
     m_state.unlock();
     pose_timestamp = posek.header.stamp.toNSec();
     // ROS_INFO_STREAM("pose callback time = "<<pose_callback_t.toc()<< " ms");
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
               double cox = 1;
               R_variance  << cox, 0, 0, 0, 0, 0,
                              0, cox, 0, 0, 0, 0,
                              0, 0, cox, 0, 0, 0,
                              0, 0, 0, cox, 0, 0,
                              0, 0, 0, 0, cox, 0,
                              0, 0, 0, 0, 0, cox;
               
               m_ekf.lock();
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
               m_ekf.unlock();
               
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
               // ROS_INFO_STREAM("yolo_depth_predict = "<<x_k_k.x()<<", "<<x_k_k.y()<<", "<<x_k_k.z());
               v_ekf.publish(p_box);
          }
          
     }
     // ROS_INFO_STREAM("vekf predict: "<<t_predict.toc()<<" ms");
}

void detectCallback(const  darknet_ros_msgs::BoundingBoxes::ConstPtr& msg){
     
     bool data_matched = false;//判断yolo检测和点云数据是否匹配成功
     darknet_ros_msgs::BoundingBoxes boundingBoxesResults_ = *msg;
     int boxno = boundingBoxesResults_.bounding_boxes.size();
     long long cur_yolo_timestamp = msg->header.stamp.toNSec();
     // ROS_INFO_STREAM("timestamp = "<<cur_yolo_timestamp);
     // ROS_INFO_STREAM("timestamp_last = "<<last_yolo_timestamp);
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
     m_yolo_match.lock();
     while(!yolo_depth.empty()){
          ROS_DEBUG_STREAM("yolo_depth.front().timestamp = "<<yolo_depth.front()->timestamp);
          if(cur_yolo_timestamp >= yolo_depth.front()->timestamp){
               if(cur_yolo_timestamp == yolo_depth.front()->timestamp){
                    yolo_cur_pc_data = *yolo_depth.front();
                    data_matched = true;
                    // ROS_INFO("data matched!!!");
                    last_yolo_timestamp = cur_yolo_timestamp;
                    yolo_depth.pop();
                    break;
               }else{
                    yolo_depth.pop();
               }
          }else{
               ROS_ERROR("The matched data has been poped out!");
               m_yolo_match.unlock();
               return;
          }
     }
     if(yolo_depth.empty() && !data_matched){
          ROS_ERROR("you need extend your queue size!");
     }
     m_yolo_match.unlock();
     
     if(data_matched){
          m_yolo.lock();
          while(!box_buffer.empty()){
               box_buffer.pop();
          }
          while(!yolo_pc_buffer.empty()){
               yolo_pc_buffer.pop();
          }
          box_buffer.push(msg);
          yolo_pc_buffer.push(yolo_cur_pc_data);
          m_yolo.unlock();
          yolo_con.notify_one();
     }
     
}
Vector3d depth_estimate(vector<vector<Vector4d>> &ave_grid_3d, double yolo_u_min, double yolo_u_max, double yolo_v_min, double yolo_v_max,int grid_k, int k_threshold){
     try{
          int size_threshold = 5;
          int num_threshold = grid_k*60/3;

          vector<Vector3d> uav_pose;
          vector<int> main_cluster_number_vector;
          Vector3d uav_final_pos;

          double n_points = 0;
          int last_k = -1;
          // ROS_INFO_STREAM("ave_grid_3d.front().size() = "<<ave_grid_3d.front().size());
          if(k_threshold >=ave_grid_3d.front().size()-1 ){
               ROS_WARN_STREAM("k_threshold "<<k_threshold<<" is too high than "<<ave_grid_3d.front().size()-1<<" , please turn it down");
               k_threshold = int((ave_grid_3d.front().size()-1)*0.9)-1;    
          }
          if(k_threshold < 0.2*ave_grid_3d.front().size()){
               k_threshold = int((ave_grid_3d.front().size()-1)*0.9)-1;
               ROS_WARN_STREAM("k_threshold "<<k_threshold<<" is too low than "<<ave_grid_3d.front().size()*0.2<<" , please turn it up");            
          }
          vector<point> grid_uv;
          for(int k = 0 ;k<ave_grid_3d.front().size()-1; k++){
               //u
               grid_uv.clear();
               int id = 0;
               bool empty_grid = true;
               int grid_u = 0;
               int grid_v = 0;
               int num_points = 0;
               
               for(int i=yolo_u_min; i<yolo_u_max; i = i+grid_k*2){
                    //v
                    grid_v = 0;
                    for(int j=yolo_v_min; j<yolo_v_max; j=j+grid_k*2){
                         if(id >= ave_grid_3d.size()){
                              ROS_ERROR("ave_grid_3d.size() is not right!!!!! Pleace make sure right usage of vector!!!!!!!!!!!!!!");
                              return Vector3d(-1,-1,-1);
                         }
                         // ROS_INFO_STREAM("id = "<<id);
                         // ROS_INFO_STREAM("depth_k = "<<k);
                         ave_grid_3d[id][k].w() += ave_grid_3d[id][k+1].w();
                         
                         // continue;
                         if(ave_grid_3d[id][k].w() != 0){
                              // ROS_INFO_STREAM("w = "<<ave_grid_3d[id][k].x());
                              ave_grid_3d[id][k].x() += ave_grid_3d[id][k+1].x();
                              ave_grid_3d[id][k].y() += ave_grid_3d[id][k+1].y();
                              ave_grid_3d[id][k].z() += ave_grid_3d[id][k+1].z();
                              empty_grid = false;
                              grid_uv.emplace_back(grid_u, grid_v, id);
                              num_points += ave_grid_3d[id][k].w();
                         }
                         id ++;
                         grid_v++;
                    }
                    grid_u++;
               }
               //There should not be anything within 1m center of uav;
               //非空栅格 && 有值栅格数>n && 栅格内点运数>m
               if(!empty_grid && grid_uv.size()>size_threshold && num_points>num_threshold){
                    last_k = k;
                    // get_candidate = true;//选择当前深度为目标深度
                    // //判断相邻性
                    // int size_grid_uv = grid_uv.size();
                    // vector<int> valid_grid(size_grid_uv);
                    cv::Mat down_sample = cv::Mat::zeros((yolo_v_max - yolo_v_min)+10, (yolo_u_max - yolo_u_min)+10, CV_8UC1);

                    //DBSCAN算法(Density-Based Spatial Clustering of Applications with Noise) 
                    DBSCAN(grid_uv,1.5,3);
                    //get the cluster number and points number in each cluster
                    vector<int> num_in_cluster;//points number of each cluster
                    vector<int> cluster_array;
                    int cluster = grid_uv[0].cluster;
                    cluster_array.emplace_back(cluster);
                    num_in_cluster.emplace_back(ave_grid_3d[grid_uv[0].id][k].w());
                    int cluster_index = 0;
                    for(int i = 0; i < grid_uv.size(); i++){
                         if(grid_uv[i].pointType != NOISE){
                              cluster = grid_uv[i].cluster;
                              bool new_cluster = true;
                              for(int j = 0;j <cluster_array.size(); j++){
                                   if(cluster == cluster_array[j]){
                                        new_cluster = false;
                                        if(j<num_in_cluster.size()){
                                             num_in_cluster[j] += ave_grid_3d[grid_uv[i].id][k].w();
                                        }else{
                                             ROS_ERROR("num_in_cluster cause core dumped, return!");
                                             return Vector3d(-1,-1,-1);
                                        }
                                   }
                              }
                              if(new_cluster){
                                   cluster_array.emplace_back(cluster);
                                   num_in_cluster.emplace_back(ave_grid_3d[grid_uv[i].id][k].w());
                              }
                         }
                    }
                    //choose the main cluster according to points number
                    int main_cluster = cluster_array[0];
                    int main_cluster_num = num_in_cluster[0];
                    
                    for(int i = 0; i < cluster_array.size(); i++){
                         if(main_cluster_num < num_in_cluster[i]){
                              main_cluster_num = num_in_cluster[i];
                              main_cluster = cluster_array[0];
                         }
                    }
                    
                    //estimate uav's position in lidar frame
                    double uav_x = 0;
                    double uav_y = 0;
                    double uav_z = 0;
                    n_points = main_cluster_num;
                    // ROS_INFO_STREAM("main_cluster_num = "<<main_cluster_num);
                    //ave_x = (x1*n1 + x2*n2)/(n1+n2)
                    for(int i = 0; i < grid_uv.size(); i++){
                         if(grid_uv[i].pointType != NOISE){
                              if(grid_uv[i].cluster == main_cluster){
                                   uav_x += ave_grid_3d[grid_uv[i].id][k].x();
                                   // ROS_INFO_STREAM("uav_x = "<<uav_x);
                                   uav_y += ave_grid_3d[grid_uv[i].id][k].y();
                                   uav_z += ave_grid_3d[grid_uv[i].id][k].z();
                                   // n_points += ave_grid_3d[grid_uv[i].id][k].w();
                                   // ROS_INFO_STREAM("uav_x = "<<ave_grid_3d[grid_uv[i].id][k].w());
                              }
                         }
                    }
                    uav_x = uav_x/n_points;
                    uav_y = uav_y/n_points;
                    uav_z = uav_z/n_points;
                    uav_pose.emplace_back(uav_x, uav_y, uav_z);
                    main_cluster_number_vector.emplace_back(n_points);
                    //可视化
                    if(visualization){
                         for(int i = 0; i < grid_uv.size(); i++){
                              cluster = grid_uv[i].cluster;
                              int index_i_u = grid_uv[i].x;
                              int index_i_v = grid_uv[i].y;
                              if(grid_uv[i].pointType != NOISE){
                                   if(cluster == grid_uv[0].cluster){
                                        
                                             cv::circle(down_sample,cv::Point2d(index_i_u*grid_k*2+grid_k+grid_k, index_i_v*grid_k*2+grid_k),grid_k,255);
                                   }else{
                                        // ROS_WARN("cluster changed!!!!!!!!!!!!!!!!!!!!!!");
                                             cv::circle(down_sample,cv::Point2d(index_i_u*grid_k*2+grid_k+grid_k, index_i_v*grid_k*2+grid_k),grid_k,180);
                                   }
                                   
                              }else{
                                   // ROS_ERROR("noise point!!!!!!!!!!!!!!!!!!!!!!!!!!");
                                   
                                   cv::circle(down_sample,cv::Point2d(index_i_u*grid_k*2+grid_k+grid_k, index_i_v*grid_k*2+grid_k),grid_k,100);
                              }
                              
                         }
                         cv::imshow("image-down_sample", down_sample);
                         // cv::waitKey(1);
                    }
                    
               }
               grid_uv.clear();  
               if(last_k>0 && k - last_k >= 1){
                    if(main_cluster_number_vector.size() == 0){
                         ROS_ERROR("main_cluster_number_vector size is zero, segmentation fault! return!");
                         return Vector3d(-1,-1,-1);
                    }
                    int max_points_num = main_cluster_number_vector[0];
                    int index_max = 0;
                    for(int i = 0; i < main_cluster_number_vector.size(); i++){
                         if(max_points_num < main_cluster_number_vector[i]){
                              max_points_num = main_cluster_number_vector[i];
                              index_max = i;
                         }
                    }
                    uav_final_pos = uav_pose[index_max];
                    // ROS_INFO_STREAM("uav_position = "<<uav_final_pos.x()<<", "<<uav_final_pos.y()<<", "<<uav_final_pos.z());
                    if(uav_final_pos.x()<1){
                         //failed results
                         ROS_ERROR_STREAM("estimated depth = "<<uav_final_pos<<"m < 1m, depth estimation failed! continue !");
                    }
                    return uav_final_pos;
               }
               if(k > k_threshold){
                    uav_final_pos = -Vector3d::Identity();
                    ROS_ERROR_STREAM("k ="<<k<<" >k_threshold = "<<k_threshold<<". Initialization fault!!");
                    return uav_final_pos;
               }
          }
          ave_grid_3d.clear();
          
          // ROS_INFO_STREAM("initialization time: "<<t_update.toc()<<" ms");
          
          return Vector3d(-1,-1,-1);
     }
     catch(std::bad_alloc){
          ROS_ERROR("terminate called after throwing an instance of 'std::bad_alloc'     in depth_estimate.");
          return Vector3d(-1,-1,-1);
     }
}



void Yolo_Update(){
     ROS_INFO("Yolo_Update thread opened!!");
     while(true){
          try{
               
               std::unique_lock<std::mutex> lk(m_yolo);
               yolo_con.wait(lk,[&]
                    {
                         return (!box_buffer.empty());
                    });//后面是一个lanbuda函数
                    
               darknet_ros_msgs::BoundingBoxes::ConstPtr boundingBoxesResults_ = box_buffer.front();
               box_buffer.pop();
               Pc_Vector yolo_cur_pc_data = yolo_pc_buffer.front();
               yolo_pc_buffer.pop();
               m_yolo.unlock();
               

               TicToc t_update;
               //calculate the 3D position of yolo detection object(uav) in camera frame with matched pc data    
               sigma_feature[0] = boundingBoxesResults_->bounding_boxes[0].utx;
               sigma_feature[1] = boundingBoxesResults_->bounding_boxes[0].uty;   
               Vector3d uav_position_lidar = -Vector3d::Identity();
               vector<Vector4d> line_ij;
               
               //initialization： estimate an accurate depth initial value
               int grid_k = 3;
               int grid_num = 50;
               double uij,vij;
               double k = -0.2;
               int w_u = (boundingBoxesResults_->bounding_boxes[0].xmax - boundingBoxesResults_->bounding_boxes[0].xmin)/2.0;
               int w_v = (boundingBoxesResults_->bounding_boxes[0].ymax - boundingBoxesResults_->bounding_boxes[0].ymin)/2.0;
               double yolo_u_min = boundingBoxesResults_->bounding_boxes[0].xmin-w_u*k;
               double yolo_u_max = boundingBoxesResults_->bounding_boxes[0].xmax+w_u*k;
               double yolo_v_min = boundingBoxesResults_->bounding_boxes[0].ymin-w_v*k;
               double yolo_v_max = boundingBoxesResults_->bounding_boxes[0].ymax+w_v*k;
               //calculate grid_k;
               int area_uv = w_u*w_v;
               grid_k = sqrt(area_uv/grid_num);
               if(grid_k<2){
                    grid_k = 2;
               }
               if(grid_k>10){
                    grid_k = 10;
               }
               // ROS_INFO_STREAM("grid_k = "<<grid_k);
               // }
               // catch(std::bad_alloc){
               //      ROS_ERROR("terminate called after throwing an instance of 'std::bad_alloc'     in yolo depth estimation part 1.");
               //      break;
               // }
               ROS_INFO("estimating depth!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
               try{
                    // if(flag_int_xkk == 0){              
                    // ROS_INFO("initialization!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
                    vector<vector<Vector4d>> ave_grid_3d; 
                    m_visualization.lock();
                    TicToc box_t;
                    int size_of_grid = 0;
                    for(int i=yolo_u_min; i<yolo_u_max; i = i+grid_k*2){
                         for(int j=yolo_v_min; j<yolo_v_max; j=j+grid_k*2){
                              uij = i + grid_k;
                              vij = j + grid_k;
                              //for visualization
                              box_grid_points.emplace_back(i,j);   
                              size_of_grid++;                         
                         }
                    } 
                    ave_grid_3d.reserve(grid+2);
                    // ROS_INFO_STREAM("fill box time = "<<box_t.toc()<<" ms");
                    m_visualization.unlock(); 


                    calculate_yolo_depth_2(yolo_cur_pc_data.pc_lidar_3d, yolo_cur_pc_data.pc_uv, ave_grid_3d, yolo_u_min, yolo_u_max, yolo_v_min, yolo_v_max, grid_k, 0.5 , 0.5, 15);

                    // for(int i=yolo_u_min; i<yolo_u_max; i = i+grid_k*2){
                    //      for(int j=yolo_v_min; j<yolo_v_max; j=j+grid_k*2){
                    //           uij = i + grid_k;
                    //           vij = j + grid_k;
                    //           //for visualization
                    //           // box_grid_points.emplace_back(i,j);
                              
                    //           calculate_yolo_depth_init(yolo_cur_pc_data.pc_lidar_3d, yolo_cur_pc_data.pc_uv, line_ij, uij, vij, grid_k, 0.5 , 2, 30);
                    //           ave_grid_3d.emplace_back(line_ij);
                              
                    //      }
                    //      //visualization

                    //      // cout<<"===============================================================================================================================================================================";
                    // }
                    ROS_INFO_STREAM("calculate_yolo_depth_init time: "<<t_update.toc()<<" ms");
                    //depth
                    uav_position_lidar = depth_estimate(ave_grid_3d, yolo_u_min, yolo_u_max, yolo_v_min, yolo_v_max, grid_k, 20); 

                    ROS_INFO_STREAM("initialization time: "<<t_update.toc()<<" ms");
                         // uav_position_lidar = uav_final_pos;
                         
                    // }else{
                    //      // double u0 = (boundingBoxesResults_->bounding_boxes[0].xmin + boundingBoxesResults_->bounding_boxes[0].xmax)/2;//yolo输出的u，v是输入图像的u，v，没有经过缩放的
                    //      // double v0 = (boundingBoxesResults_->bounding_boxes[0].ymin + boundingBoxesResults_->bounding_boxes[0].ymax)/2;
                    //      // int w_u = (boundingBoxesResults_->bounding_boxes[0].xmax - boundingBoxesResults_->bounding_boxes[0].xmin)/2.0;
                    //      // int w_v = (boundingBoxesResults_->bounding_boxes[0].ymax - boundingBoxesResults_->bounding_boxes[0].ymin)/2.0;
                    //      // int search_radius = (w_u > w_v)? w_u : w_v;
                    //      // ROS_INFO_STREAM("search_radius = "<<search_radius);
                    //      // uav_position_lidar = calculate_yolo_depth(yolo_cur_pc_data.pc_lidar_3d, yolo_cur_pc_data.pc_uv, u0, v0, search_radius*1.5); 

                    //      vector<vector<Vector4d>> ave_grid_3d;
                         
                    //      //根据先验位置设置阈值
                    //      Vector3d cur_uav_pose = x_k_k;//in lidar frame
                    //      double depth_shreshold_down = cur_uav_pose.x() - 2;
                    //      double depth_shreshold_up = cur_uav_pose.x() + 2;
                    //      // ROS_WARN_STREAM("("<<depth_shreshold_down<<", "<<depth_shreshold_up<<")");
                    //      for(int i=yolo_u_min; i<yolo_u_max; i = i+grid_k*2){
                    //           for(int j=yolo_v_min; j<yolo_v_max; j=j+grid_k*2){
                                   
                    //                uij = i + grid_k;
                    //                vij = j + grid_k;
                    //                box_grid_points.emplace_back(i,j);
                    //                calculate_yolo_depth_init(yolo_cur_pc_data.pc_lidar_3d, yolo_cur_pc_data.pc_uv, line_ij, uij, vij, grid_k, 0.5 , depth_shreshold_down, depth_shreshold_up);
                    //                ave_grid_3d.emplace_back(line_ij);
                                   
                    //           }
                    //           //visualization
                    //           // cout<<"===============================================================================================================================================================================";
                    //      }
                    //      // ROS_INFO_STREAM("grid time: "<<t_update.toc()<<" ms");
                    //      //depth
                    //      uav_position_lidar = depth_estimate(ave_grid_3d, yolo_u_min, yolo_u_max, yolo_v_min, yolo_v_max, grid_k, ave_grid_3d.front().size()-1); 
                    //      ROS_INFO_STREAM("yolo depth estimating time: "<<t_update.toc()<<" ms");
                    // }
                    ROS_INFO_STREAM("yolo_depth_no_rect = "<<uav_position_lidar.x()<<", "<<uav_position_lidar.y()<<", "<<uav_position_lidar.z());
                    if(uav_position_lidar.x()<1){
                         //failed results
                         // ROS_ERROR("estimated depth < 1m, depth estimation failed! continue !");
                         continue;
                    }

               }
               catch(std::bad_alloc){
                    ROS_ERROR("terminate called after throwing an instance of 'std::bad_alloc'     in yolo depth estimation part.");
                    continue;
               }
               

               /**************************************************/
               //motion compensation
               try{
                    m_state.lock();
                    Quaterniond q_drone_now = q_drone;
                    Vector3d p_drone_now = p_drone;
                    m_state.unlock();
                    Quaterniond q_b0_b1 = q_drone_now.inverse()*yolo_cur_pc_data.q_wb;
                    Vector3d t_b0_b1 = q_drone_now.inverse()*(yolo_cur_pc_data.t_wb-p_drone_now);
                    // ROS_INFO_STREAM("relative p = ("<<t_b0_b1.x()<<", "<<t_b0_b1.y()<<", "<<t_b0_b1.z()<<")");
                    // ROS_INFO_STREAM("relative q = \n"<<q_b0_b1.toRotationMatrix().matrix());
                    Matrix3d R_b0_lidar = q_b0_b1*R_imu_lidar;
                    Vector3d t_b0_lidar = q_b0_b1*t_imu_lidar + t_b0_b1;
                    
                    Matrix3d R_lidar0_lidar = R_imu_lidar.inverse()*R_b0_lidar;
                    Vector3d t_lidar0_lidar = R_imu_lidar.inverse()*(t_b0_lidar)-R_imu_lidar.inverse()*t_imu_lidar;






                    
                    Vector3d uav_pose_body = R_imu_lidar*uav_position_lidar + t_imu_lidar;
                    Vector3d uav_pose_world = yolo_cur_pc_data.q_wb*uav_pose_body + yolo_cur_pc_data.t_wb;
                    // Vector3d rect_uav_pose_body = q_drone_now.inverse()*uav_pose_world - q_drone_now*p_drone_now;//rected with current uav's pose
                    // Vector3d uav_position_incamframe_rect = R_cam_imu*uav_pose_bodyframe + t_cam_imu;//rected uav position in camera frame
                    Vector3d rect_uav_pose_lidar = R_lidar0_lidar*uav_position_lidar + t_lidar0_lidar;//rected uav position in lidar frame
                    // Vector3d rect_project_uav = K_in*uav_position_incamframe_rect;
                    Vector3d rect_uav_pose_cam = R_cam_lidar*rect_uav_pose_lidar + t_cam_lidar;
                    ROS_INFO_STREAM("rect_uav_pose_lidar = "<<rect_uav_pose_lidar.x()<<", "<<rect_uav_pose_lidar.y()<<", "<<rect_uav_pose_lidar.z());
                    //for visualization 
                    rect_feat_point[0] = uav_pose_world.x();//因为不知道下一帧pc的时间戳，所以在pc收到的时候在转到pc对应的lidar系下
                    rect_feat_point[1] = uav_pose_world.y();
                    rect_feat_point[2] = uav_pose_world.z();
                    
                    //初始化阶段
                    if(flag_int_xkk == 0){
                         ROS_INFO("initialization!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
                         // flag_int_xkk = 1;//for test
                         p_f_L = rect_uav_pose_lidar;//initial value for vekf
                         //update feature point        
                         Vector3d project_uav = K_in*rect_uav_pose_lidar;
                         m_feature.lock();
                         //for visualization   not rected pose
                         feat_point[0] = project_uav.x()/project_uav.z();
                         feat_point[1] = project_uav.y()/project_uav.z();
                         m_feature.unlock();
                         ifdetection = 1;
                         continue;
                    }

                    // continue;//for test
                    //ROS_INFO("you catch me!!!!");

                    /**************************************************************/
                    //ekf update part
                    m_ekf.lock();
                    // double x_f_l = rect_uav_pose_lidar.x();//no rect

                    // Eigen::Matrix3d fp_tra(3,3);   //see notebook
                    // fp_tra = -R_cam_lidar;
                    // Vector3d tmp = Vector3d((feat_point[0]-343.76)/264, (feat_point[1] -183.8795)/263.70, 1.0);
                    // fp_tra.block<3,1>(0,0) =tmp;
                    // // ROS_INFO_STREAM("fp_tra = \n"<<fp_tra.matrix());
                    
                    // Eigen::Vector3d fp_vec(3);
                    // Eigen::Vector3d r1(3);
                    // r1 = R_cam_lidar.block<3,1>(0,0);
                    
                    // Eigen::Vector3d t_c_L = t_cam_lidar;
                    // fp_vec = x_f_l*r1 + t_c_L;
                    // Eigen::Vector3d v_mi;
                    // v_mi = fp_tra.inverse()*fp_vec;//求解的【cz，y，z】
                    // double z_f_c = v_mi(0);//相机坐标系下的深度值
                    // double x_f_c = (feat_point[0] - 343.76)* z_f_c /264;//
                    // double y_f_c = (feat_point[1] - 183.8795)* z_f_c /263.700;
                    
                    // // p_f_L << x_f_l, v_mi(1), v_mi(2);//雷达系下的目标位置
                    // // ROS_INFO_STREAM("yolo_scbb = "<<p_f_L.x()<<", "<<p_f_L.y()<<", "<<p_f_L.z());
                    p_f_L = rect_uav_pose_lidar;//bo rect
                    

                    // //the jacobian matrix for covariance calculation
                    // Eigen::Matrix3d pApu, pApv;
                    // pApu <<   1/fx, 0, 0,
                    //           0, 0, 0,
                    //           0, 0, 0;
                    // pApv <<   0, 0, 0,
                    //           1/fy, 0, 0,
                    //           0, 0, 0;
                    // Eigen::Vector3d pfpuvx1 =  -fp_tra.inverse()*pApu*fp_tra.inverse()*fp_vec;
                    // Eigen::Vector3d pfpuvx2 =  -fp_tra.inverse()*pApv*fp_tra.inverse()*fp_vec;
                    // Eigen::Vector3d pfpuvx3 =  -fp_tra.inverse()*(r1 + t_cam_lidar);
                    // Eigen::Matrix3d pfpuvx;
                    // pfpuvx << pfpuvx1,pfpuvx2, pfpuvx3;

                    // Eigen::MatrixXd pfpuvx23row(2,3);
                    // pfpuvx23row = pfpuvx.block<2,3>(1,0);

                    // Eigen::Matrix3d vari_pix_z;
                    // vari_pix_z <<  0.1, 0, 0,
                    //                0, sigma_feature[0], 0,
                    //                0, 0, sigma_feature[1];  //from the yoloros, the uncertainty
                    // // else
                    // //      vari_pix_z << 0.1, 0, 0,
                    // //                          0, 0.18, 0,
                    // //                          0, 0, 0.18;  //from the yoloros, the uncertainty为什么没有观测的时候也做更新？？？？

                    // Q_variance(0,0) = 0.1;  //the variance of x component.
                    // Q_variance.block<2,2>(1,1) = pfpuvx23row*vari_pix_z*pfpuvx23row.adjoint();
                    // // cout << "Q_variance" << Q_variance << endl;
                    Eigen::Vector3d y, z_k;
                    Eigen::Matrix3d  S, K;
                    z_k = p_f_L; //output from yolov3
                    
                    /********/
                    Q_variance<<
                              0.25, 0, 0,
                              0, 0.01, 0,
                              0, 0, 0.01;
                    // Matrix3d f1,sigma_yolo_x;
                    Matrix2d sigma_yolo_x;
                    MatrixXd f1(3,2);
                    f1<<rect_uav_pose_cam.x()/fx, 0,
                         0, rect_uav_pose_cam.x()/fy,
                         0, 0;
                    // sigma_yolo_x<<
                    //                sigma_feature[0], 0, 0,
                    //                0, sigma_feature[1], 0,
                    //                0, 0, 0;
                    sigma_yolo_x<<
                                   sigma_feature[0], 0,
                                   0, sigma_feature[1];
                    Q_variance = R_cam_lidar.transpose()*f1*sigma_yolo_x*f1.transpose()*R_cam_lidar + Q_variance;
                    ROS_DEBUG_STREAM("Q_variance = "<<endl<<Q_variance.matrix());
                    ROS_DEBUG_STREAM("sigma_feature = "<< sigma_feature[0]<<", "<<sigma_feature[1]);
                    /*********/
                    y = z_k - C_T*x_k_k;   //nonlinear
                    S=C_T*P_k_k*C_T.transpose() + Q_variance; //observation
                    // S = 0.01*eye3;
                    K = P_k_k*C_T.transpose()*S.inverse();
                    // K = 0.1*eye3;
                    MatrixXd I_KH = (eye3 -K*C_T); 

                    /*update*/   
                    x_k_k = x_k_k + K*y;
                    P_k_k=I_KH*P_k_k*I_KH.transpose() + K*Q_variance*K.transpose();
                    m_ekf.unlock();

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
                    ROS_INFO_STREAM("yolo_depth_update = "<<x_k_k.x()<<", "<<x_k_k.y()<<", "<<x_k_k.z());
                    v_ekf.publish(p_box);
                    // ROS_DEBUG_STREAM("update time : "<<t_update.toc()<<" ms");
               }catch(std::bad_alloc){
                    ROS_ERROR("terminate called after throwing an instance of 'std::bad_alloc'     in yolo update part.");
                   continue;
               }
          }catch(std::bad_alloc){
                    ROS_ERROR("terminate called after throwing an instance of 'std::bad_alloc'     in YOLO thread.");
                    continue;
               }
     }
     
}





int main(int argc, char **argv)
{
     ros::init(argc, argv, "pc_preprocessing");
     ros::NodeHandle n("~");
     ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
     ros::Subscriber subpc = n.subscribe("/livox/lidar", 20, pc2Callback);
     ros::Subscriber subimg = n.subscribe("/zed2/zed_node/left/image_rect_color", 1, imgCallback);
     ros::Subscriber subpos = n.subscribe("/mavros/local_position/pose", 100, poseCallback);
     ros::Subscriber subvel = n.subscribe("/mavros/local_position/velocity_body", 100, velCallback);

     ros::Subscriber subdetection = n.subscribe("/darknet_ros/bounding_boxes", 1, detectCallback);  //dependency: darknet_ros_msgs

     pubimg = n.advertise<sensor_msgs::Image>("/camera/rgb/image_raw",  10);
     pubimg_upsample = n.advertise<sensor_msgs::Image>("/camera/xyz/image_upsampling",  10);
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
     Quaterniond text_q = Quaterniond(0.999997854233, 0.000991703942418, 0.00180647568777, 0.0);
     ROS_INFO_STREAM("test_q = \n"<<text_q.toRotationMatrix().matrix());



     T_pc_ima = T_pc_ima.setIdentity();
     T_pc_ima.block<3,3>(0,0) = K_in*R_cam_lidar;
     T_pc_ima.block<3,1>(0,3) = K_in*t_cam_lidar;
     T_cam_lidar = T_cam_lidar.setIdentity();
     T_cam_lidar.block<3,3>(0,0) = R_cam_lidar;
     T_cam_lidar.block<3,1>(0,3) = t_cam_lidar;
     

     std::thread prepro_pc = std::thread(&Preprocess);
     std::thread yolo_update = std::thread(&Yolo_Update);
     ros::spin();
     //prepro_pc.join();
     return 0;
}
