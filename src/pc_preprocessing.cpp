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


/***
void pc2Callback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
     ROS_INFO("pc received");

     
     T_pc_ima <<
          339.772898240000,	-263.502373294560,	-54.6615525600000,	138.543236720000,
          128.756166698000,	-12.8129096926770,	-294.290820079500,	36.3959495165000,
          0.980644000000000,	0.000526794000000000,	-0.195801000000000,	0.283747000000000,
          0,	0,	0,	1; //1105
     sensor_msgs::PointCloud2 MSG = *msg;
     chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
     const int thread_num = 4;
     Threadsstruct threadsstruct[thread_num];
     pcl::fromROSMsg (*msg, cloud);
     ROS_DEBUG("cloud size: %d",cloud.points.size());
     //size = 10000/0.1s
    
     for(int i = 0; i<cloud.points.size(); i++){
          pc_i<< cloud.points[i].x, cloud.points[i].y, cloud.points[i].z, 1;
          threadsstruct[i%thread_num].pc_vector.push_back(pc_i);
     }
     chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
     chrono::duration<double> time_used1 = chrono::duration_cast < chrono::duration < double >> (t2 - t1);
     cout << "split time: " << time_used1.count() << " s." << endl;
     
     for(int i = 0; i<4; i++){
          int ret = pthread_create( &tids[i], NULL, multi_thread_preprocess ,(void*)&(threadsstruct[i]));
          if (ret != 0)
          {
               ROS_WARN("pthread_create error");
               ROS_BREAK();
          }
     }
     chrono::steady_clock::time_point t3 = chrono::steady_clock::now();
     chrono::duration<double> time_used2 = chrono::duration_cast < chrono::duration < double >> (t3 - t2);
     cout << "preprocessing time: " << time_used2.count() << " s." << endl;
     for( int i = thread_num - 1; i >= 0; i--)  
     {
          pthread_join( tids[i], NULL ); 
          if  (point_max.x < threadsstruct[i].point_max_thread.x) {point_max.x = threadsstruct[i].point_max_thread.x; }
          if  (point_max.y < threadsstruct[i].point_max_thread.y) {point_max.y = threadsstruct[i].point_max_thread.y; }
          if  (point_max.z < threadsstruct[i].point_max_thread.z) {point_max.z = threadsstruct[i].point_max_thread.z; }
          if  (minmaxuv.umax < threadsstruct[i].minmaxuv_thread.umax) {minmaxuv.umax = threadsstruct[i].minmaxuv_thread.umax;}
          if  (minmaxuv.umin > threadsstruct[i].minmaxuv_thread.umin) {minmaxuv.umin = threadsstruct[i].minmaxuv_thread.umin;}
          if  (minmaxuv.vmax < threadsstruct[i].minmaxuv_thread.vmax) {minmaxuv.vmax = threadsstruct[i].minmaxuv_thread.vmax;}
          if  (minmaxuv.vmin > threadsstruct[i].minmaxuv_thread.vmin) {minmaxuv.vmin = threadsstruct[i].minmaxuv_thread.vmin;}
          //pc_array_grid
     }
     chrono::steady_clock::time_point t4 = chrono::steady_clock::now();
     chrono::duration<double> time_used3 = chrono::duration_cast < chrono::duration < double >> (t4 - t3);
     cout << "integrate time: " << time_used3.count() << " s." << endl;
          
     
     //pcl::PointCloud<pcl::PointXYZ> cloud[sum_pc];
     
     sum_pc_i ++;

     chrono::steady_clock::time_point t5 = chrono::steady_clock::now();
     chrono::duration<double> time_used = chrono::duration_cast < chrono::duration < double >> (t5 - t1);
     cout << "total preprocessing time: " << time_used.count() << " s." << endl;
     
     
     if (sum_pc_i == sum_pc) {
          sum_pc_i = 0;
          //cout << "array size: " << pc_array.size() << endl;
          int ups = upsampling_pro(point_max, point_min, minmaxuv, w_img, h_img, c_img, i_pc_count);
          //  i_pc_count ++;
          pc_size = 0;
          //pc_array.clear();
     //      for (int u = minmaxuv.umin  - grid; u < minmaxuv.umax  + grid; u++)
     //           for (int v = minmaxuv.vmin  - grid; v < minmaxuv.vmax + grid; v++){
     //                pc_array_grid[v*w_img+u].clear();
     //           }

     //      for (int u = 0; u < w_img; u++)
     //           for (int v = 0; v < h_img; v++){
     //                pc_array_grid[v*w_img+u].clear();
     //           }

          point_max.x = 0;
          point_max.y = 0;
          point_max.z = 0;
          point_min.x = 0;
          point_min.y = 0;
          point_min.z = 0;
          minmaxuv.umax = w_img/2;
          minmaxuv.umin = 40;
          minmaxuv.vmax = h_img/2;
          minmaxuv.vmin = 60;

          //char img1[50];
          //sprintf(img1, "/tmp/%02dimg.png",i_pc_count);
          //cv::imwrite(img1, img); //save the image
          //i_pc_count ++;
     }
     
  
}
*///
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







// void pc2Callback(const sensor_msgs::PointCloud2ConstPtr& msg)
// {
//      TicToc T1;
//      sensor_msgs::PointCloud2 MSG = *msg;
//      ROS_INFO("pc received");
//      //pcl::PointCloud<pcl::PointXYZ> cloud[sum_pc];

//      pcl::fromROSMsg (*msg, cloud);
//      ROS_INFO_STREAM("transport: "<<T1.toc()<<"ms");
//      sum_pc_i ++;
//      //pc_size += cloud.points.size();
//      pc_manager.CreateMask(sum_pc_i);

//    // ROS_INFO("cloud.width: %d", cloud.width);
//     //ROS_INFO("cloud.height: %d", cloud.height); //if height ==1, it is 1-D point cloud
//      //ROS_DEBUG("cloud size: %d",cloud.points.size());
//      //ROS_DEBUG("pc timestamp: %.9fs",MSG.header.stamp.toSec());

// //    vector<pcl::PointXYZ> data;
// //
// //    pcl::PointXYZ point;
// //    point.x = 2.0f;
// //    point.y = 1;
// //    point.z = 3;
// //    cloud.points.push_back(point);
// //
// //    ROS_INFO("cloud.width: %d", cloud.width);
// //    ROS_INFO("cloud.height: %d", cloud.height); //if height ==1, it is 1-D point cloud
// //    ROS_INFO("cloud size: %d",cloud.points.size());
// //
// //    ROS_INFO("x: %f", cloud.points[0].x);

//      //int grid = 5;
//      Eigen::VectorXd pc_i(4);
//      Eigen::MatrixXd T_pc_ima(4,4);
//      T_pc_ima <<
//                339.772898240000,	-263.502373294560,	-54.6615525600000,	138.543236720000,
//                128.756166698000,	-12.8129096926770,	-294.290820079500,	36.3959495165000,
//                0.980644000000000,	0.000526794000000000,	-0.195801000000000,	0.283747000000000,
//                0,	0,	0,	1; //1105

//      Eigen::VectorXd pix_pc(4);
//      int num_outliner = 0;
//      //pointcoordinate thispoints[10000];
//      //计算10，000
//       /***
//      chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
//      for (int i=0; i< cloud.points.size(); i++){
//           pc_i<< cloud.points[i].x, cloud.points[i].y, cloud.points[i].z, 1;
//           pix_pc = T_pc_ima*pc_i;

//           pix_pc[0] = pix_pc[0]/pix_pc[2];
//           pix_pc[1] = pix_pc[1]/pix_pc[2];
//           //pointcoordinate thispoint;
//           //check pix in the camera feild of view
//      }
     
//      chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
//      //chrono::duration<double> time_used2 = chrono::duration_cast < chrono::duration < double >> (t2 - t1) ;
//      //cout << "calculating: " << time_used2.count() << " s." << endl;
//      //计算+赋值10，000
    
//      for (int i=0; i< cloud.points.size(); i++){
//           pc_i<< cloud.points[i].x, cloud.points[i].y, cloud.points[i].z, 1;
//           pix_pc = T_pc_ima*pc_i;

//           pix_pc[0] = pix_pc[0]/pix_pc[2];
//           pix_pc[1] = pix_pc[1]/pix_pc[2];
//           pointcoordinate thispoint;
//           //check pix in the camera feild of view
//           if(  pix_pc[0] >= 1  && (int)pix_pc[0] <= w_img-1 &&  pix_pc[1] >= 1  && (int)pix_pc[1] < h_img -1){
//                num_outliner++;
//                thispoint.x_3d = cloud.points[i].x;
//           	thispoint.y_3d = cloud.points[i].y;
//           	thispoint.z_3d = cloud.points[i].z;
//           	thispoint.u_px = pix_pc[0];
//           	thispoint.v_px = pix_pc[1];
//                //thispoints[i]=thispoint;
//           	//pc_array.push_back(thispoint);

//                if  (thispoint.x_3d > point_max.x) { point_max.x = thispoint.x_3d; }
//                if  (thispoint.y_3d > point_max.y) { point_max.y = thispoint.y_3d; }
//                if  (thispoint.z_3d > point_max.z) { point_max.z = thispoint.z_3d; }

// //            if  (thispoint.x_3d < point_min.x) { point_min.x = thispoint.x_3d; }
// //            if  (thispoint.y_3d < point_min.y) { point_min.y = thispoint.y_3d; }
// //            if  (thispoint.z_3d < point_min.z) { point_min.z = thispoint.z_3d; }

//                if  (thispoint.u_px > minmaxuv.umax) {minmaxuv.umax = thispoint.u_px;}
//                if  (thispoint.u_px < minmaxuv.umin) {minmaxuv.umin = thispoint.u_px; }
//                if  (thispoint.v_px > minmaxuv.vmax) {minmaxuv.vmax = thispoint.v_px;}
//                if  (thispoint.v_px < minmaxuv.vmin) {minmaxuv.vmin = thispoint.v_px;}
//                pc_masks.emplace_back(thispoint, grid);
//           }
//      }
     
     
//      chrono::steady_clock::time_point t3 = chrono::steady_clock::now();
//      chrono::duration<double> time_used3 = chrono::duration_cast < chrono::duration < double >> (t3 - t2) ;
//      cout << "copy: " << time_used3.count() << " s." << endl;
//      Mask_pc pc_mask;
//      ***/
//      TicToc start;
//      int outliner=0;
//      for (int i=0; i< cloud.points.size(); i++){
//           pc_i<< cloud.points[i].x, cloud.points[i].y, cloud.points[i].z, 1;
//           //ROS_INFO_STREAM("pc: "<<pc_i.matrix());
//           if(pc_i.x()>20 || pc_i.x()<1){
//                outliner++;
//                continue;
//           }

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
//                //thispoints[i]=thispoint;
//           	//pc_array.push_back(thispoint);

//                if  (thispoint.x_3d > point_max.x) { point_max.x = thispoint.x_3d; }
//                if  (thispoint.y_3d > point_max.y) { point_max.y = thispoint.y_3d; }
//                if  (thispoint.z_3d > point_max.z) { point_max.z = thispoint.z_3d; }

//                if  (thispoint.u_px > minmaxuv.umax) {minmaxuv.umax = thispoint.u_px;}
//                if  (thispoint.u_px < minmaxuv.umin) {minmaxuv.umin = thispoint.u_px; }
//                if  (thispoint.v_px > minmaxuv.vmax) {minmaxuv.vmax = thispoint.v_px;}
//                if  (thispoint.v_px < minmaxuv.vmin) {minmaxuv.vmin = thispoint.v_px;}

//                //int test = max(2, 4);
//                //full 10x10 grid
               
//                //for (int u = max( (int)(pix_pc[0]- grid), 0); u < min ( (int)(pix_pc[0] + grid), w_img); u++)
//                //     for (int v = max( (int)( pix_pc[1]- grid), 0);  v < min ( (int)(pix_pc[1] + grid), h_img); v++){
//                //          pc_array_grid[v*w_img+u].push_back(thispoint);           
//                //     }
//                //pc_manager.mask_win[sum_pc_i].pc_masks.emplace_back(thispoint, grid);
//                pc_masks.emplace_back(thispoint, grid);
          
// //            for (int u = 0; u < w_img; u++)
// //                 for (int v = 0; v < h_img; v++){
// //                	 pc_array_grid[v*w_img+u].push_back(thispoint);
// //                 }
//             //cout << "u_px: " << pix_pc[0] << ". v: " << pix_pc[1] << endl;
//           }
//      }
//      ROS_DEBUG_STREAM("mask processing: "<<start.toc()<<"ms              outliner: "<<outliner);
//     //ROS_DEBUG_STREAM("outlier: " << cloud.points.size()-num_outliner);
     
//      /***
//      chrono::steady_clock::time_point t4 = chrono::steady_clock::now();
//      for (int i=0; i< cloud.points.size(); i++){
//           pc_i<< cloud.points[i].x, cloud.points[i].y, cloud.points[i].z, 1;
//           pix_pc = T_pc_ima*pc_i;

//           pix_pc[0] = pix_pc[0]/pix_pc[2];
//           pix_pc[1] = pix_pc[1]/pix_pc[2];

//       	pointcoordinate thispoint;
//           //check pix in the camera feild of view
//           if(  pix_pc[0] >= 1  && (int)pix_pc[0] <= w_img-1 &&  pix_pc[1] >= 1  && (int)pix_pc[1] < h_img -1){
//                num_outliner++;
//                thispoint.x_3d = cloud.points[i].x;
//           	thispoint.y_3d = cloud.points[i].y;
//           	thispoint.z_3d = cloud.points[i].z;
//           	thispoint.u_px = pix_pc[0];
//           	thispoint.v_px = pix_pc[1];
//                //thispoints[i]=thispoint;
//           	//pc_array.push_back(thispoint);

//                if  (thispoint.x_3d > point_max.x) { point_max.x = thispoint.x_3d; }
//                if  (thispoint.y_3d > point_max.y) { point_max.y = thispoint.y_3d; }
//                if  (thispoint.z_3d > point_max.z) { point_max.z = thispoint.z_3d; }

// //            if  (thispoint.x_3d < point_min.x) { point_min.x = thispoint.x_3d; }
// //            if  (thispoint.y_3d < point_min.y) { point_min.y = thispoint.y_3d; }
// //            if  (thispoint.z_3d < point_min.z) { point_min.z = thispoint.z_3d; }

//                if  (thispoint.u_px > minmaxuv.umax) {minmaxuv.umax = thispoint.u_px;}
//                if  (thispoint.u_px < minmaxuv.umin) {minmaxuv.umin = thispoint.u_px; }
//                if  (thispoint.v_px > minmaxuv.vmax) {minmaxuv.vmax = thispoint.v_px;}
//                if  (thispoint.v_px < minmaxuv.vmin) {minmaxuv.vmin = thispoint.v_px;}

//                //int test = max(2, 4);
//                //full 10x10 grid
               
//                pc_masks.emplace_back(thispoint, grid);
          
// //            for (int u = 0; u < w_img; u++)
// //                 for (int v = 0; v < h_img; v++){
// //                	 pc_array_grid[v*w_img+u].push_back(thispoint);
// //                 }
//             //cout << "u_px: " << pix_pc[0] << ". v: " << pix_pc[1] << endl;
//           }
//      }
//      ***/
//     //ROS_DEBUG_STREAM("outlier: " << cloud.points.size()-num_outliner);

//      if (sum_pc_i == sum_pc) 
//           sum_pc_i = 0;
//      //cout << "array size: " << pc_array.size() << endl;
//      int ups = upsampling_pro(point_max, point_min, minmaxuv, w_img, h_img, c_img, i_pc_count);
//      //  i_pc_count ++;
//      pc_size = 0;
//      //pc_array.clear();
// //      for (int u = minmaxuv.umin  - grid; u < minmaxuv.umax  + grid; u++)
// //           for (int v = minmaxuv.vmin  - grid; v < minmaxuv.vmax + grid; v++){
// //                pc_array_grid[v*w_img+u].clear();
// //           }

// //      for (int u = 0; u < w_img; u++)
// //           for (int v = 0; v < h_img; v++){
// //                pc_array_grid[v*w_img+u].clear();
// //           }

//      point_max.x = 0;
//      point_max.y = 0;
//      point_max.z = 0;
//      point_min.x = 0;
//      point_min.y = 0;
//      point_min.z = 0;
//      minmaxuv.umax = w_img/2;
//      minmaxuv.umin = 40;
//      minmaxuv.vmax = h_img/2;
//      minmaxuv.vmin = 60;

//      //char img1[50];
//      //sprintf(img1, "/tmp/%02dimg.png",i_pc_count);
//      //cv::imwrite(img1, img); //save the image
//      //i_pc_count ++;
     

   
// }






void poseCallback(const  geometry_msgs::PoseStamped::ConstPtr& msg)
{
     geometry_msgs::PoseStamped posek;

     posek = *msg;
     //pose_series.push(posek);
	// pose_global.pose = msg->pose;
     //pose_global = *msg;
     p_drone = Vector3d(posek.pose.position. x,posek.pose.position.y, posek.pose.position.z );
     q_drone = Quaterniond(posek.pose.orientation.w, posek.pose.orientation.x, posek.pose.orientation.y, posek.pose.orientation.z);
}

void velCallback(const  geometry_msgs::TwistStamped::ConstPtr& msg)
{
     //ROS_DEBUG_STREAM("velcallback");
	 geometry_msgs::TwistStamped velk;

	 velk = *msg;

     int grid_ave = 50;
     int w_img = 3;
     
     int sum_no = 0; // the total points near the feature point
	Matrix3d R_CL;
          R_CL<<
          0.0101012000000000,	-0.998801000000000,	0.0479053000000000,
          -0.195541000000000,	-0.0489563000000000, -0.979473000000000,
          0.980644000000000, 0.000526794000000000, -0.195801000000000;
     Vector3d t_CL = Vector3d(0.155312000000000, -0.0598381000000000, 0.283747000000000);
     double fx, fy, cx, cy; //the intrinsic parameters of the camera
     fx = 264.0;
     fy = 263.700000000000;
     cx = 343.760000000000;
     cy = 183.879500000000;
     m_feature.lock();
     if (pc_array_feature.size() == 0){
          //cerr << "No point near the feature region." << endl;
          sum_no = 1;
          int u0 = feat_point[0];
          int v0 = feat_point[1];
          m_feature.unlock();
          Vector3d p_b = Vector3d((u0-cx)/fx, (v0-cy)/fy, 1);
          //ROS_INFO_STREAM("normalized point: ("<<p_b(0)<<", "<<p_b(1)<<", "<<p_b(2)<<")");
          p_b = (q_bc*p_b+t_bc);

          geometry_msgs::PointStamped p_box;
          p_box.point.x = p_b(0);
          p_box.point.y = p_b(1);
          p_box.point.z = p_b(2);
          //ROS_INFO_STREAM("interested point: ("<<p_b(0)<<", "<<p_b(1)<<", "<<p_b(2)<<")");
          v_ekf.publish(p_box);
          

     }
     else{
          //cerr << "ekf=======================================" << endl;
          for (int i_pc =  0; i_pc < pc_array_feature.size(); i_pc++){
               sum_x = pc_array_feature[i_pc].x_3d + sum_x;
               sum_no++;
          }
          int u0 = feat_point[0];
          int v0 = feat_point[1];
          m_feature.unlock();
          /***????
          if (pc_array_feature.size() == 0){
          cerr << "No point near the feature region." << endl;
          sum_no = 1;
          }
          ***/
          double x_f_l = sum_x/sum_no;

          sum_no  = 0;
          sum_x = 0;

          //cout << "feature point in LiDAR frame, x coordinate: " << x_f_l << endl;
          int ii = v0*w_img+u0;
          //cout << "Count of point near the feature region: " << pc_array_feature.size() << endl;

          /*projection matrix:
          264	0	343.760000000000
          0	263.700000000000	183.879500000000
          0	0	1 */

          /*lidar to cam matrix:
          0.0101012000000000	-0.998801000000000	0.0479053000000000	0.155312000000000
     -0.195541000000000	-0.0489563000000000	-0.979473000000000	-0.0598381000000000
     0.980644000000000	0.000526794000000000	-0.195801000000000	0.283747000000000
     0	0	0	1
          */
          

          Eigen::Matrix3d fp_tra;
          
          fp_tra << (u0-343.76)/264,   0.998801000000000,  -0.0479053000000000,
               (v0 -183.8795)/263.70,   0.0489563000000000,  0.979473000000000,
                    1,   -0.000526794000000000,  0.195801000000000;




          Eigen::Vector3d fp_vec(3);
          Eigen::Vector3d r1(3);
          r1 <<  0.0101012000000000,  -0.195541000000000,  0.980644000000000;
          Eigen::Vector3d t_c_L;
          t_c_L <<  0.155312000000000, -0.0598381000000000,  0.283747000000000;
          fp_vec = x_f_l*r1 + t_c_L;
          Eigen::Vector3d v_mi;
          v_mi = fp_tra.inverse()*fp_vec;

          //the jacobian matrix for covariance calculation
          Eigen::Matrix3d pApu, pApv;
          pApu << 1/fx,  0,  0,
                    0, 0, 0,
                         0, 0, 0;
          pApv << 0,  0,  0,
               1/fy, 0, 0,
                              0, 0, 0;
          Eigen::Vector3d pfpuvx1 =  -fp_tra.inverse()*pApu*fp_tra.inverse()*fp_vec;
          Eigen::Vector3d pfpuvx2 =  -fp_tra.inverse()*pApv*fp_tra.inverse()*fp_vec;
          Eigen::Vector3d pfpuvx3 =  -fp_tra.inverse()*(r1 + t_c_L);//???应该没有t
          Eigen::Matrix3d pfpuvx;
          pfpuvx << pfpuvx1,pfpuvx2, pfpuvx3;
          Eigen::MatrixXd pfpuvx23row(2,3);
          pfpuvx23row = pfpuvx.block<2,3>(1,0);//

          double z_f_c = v_mi(0);
          double x_f_c = (u0 - 343.76)* z_f_c /264;
          double y_f_c = (v0 - 183.8795)* z_f_c /263.700;
          Eigen::Vector3d p_f_L, p_f_c;
          p_f_L << x_f_l, v_mi(1), v_mi(2);//lidar坐标系下的坐标
          p_f_c << x_f_c, y_f_c, z_f_c;//相机坐标系下的坐标


          flag_int_xkk_last = flag_int_xkk;
          flag_int_xkk = 1;

          //cout << "feature point in LiDAR frame: " << p_f_L(0)  << ", " << p_f_L(1)  << ", "   << p_f_L(2)  << endl;
          //cout << "feature point in camera frame: " << p_f_c(0)  << ", "  << p_f_c(1) << ", "  << p_f_c(2)  << endl;

          //the covariance calculation:

          if ((flag_int_xkk_last == 0) & (flag_int_xkk == 1)){
          x_k_k = p_f_L;  //initialization of the variable in kalman filter使用lidar系下的坐标进行初始化
          }

          if (flag_int_xkk == 1){
               Eigen::MatrixXd J_M(2,3);//没用上？？？
               J_M << 1/p_f_c(2)*fx , 0,  -fx*p_f_c(0)/(p_f_c(2)* p_f_c(2)),
                    0,  1/p_f_c(2)*fy,  -fy*p_f_c(1)/(p_f_c(2)* p_f_c(2));


          //     G_T = [eye(3,3), t_est*eye(3,3); zeros(3,3), eye(3,3)]; %state transition matrix
          //     H_T = [0.5*t_est*t_est*eye(3,3); t_est*eye(3,3)];  %input matrix
          //     C_T = [eye(3,3), zeros(3,3)]; %output matrix
          //
          //     k_input = round(k*t_est/t_input);
          //     if k_input > size_T_input
          //         u_k = u_array_noise(1:3, end);
          //     else
          //         u_k = u_array_noise(1:3, k_input);
          //     end
          //
          //     k_ob = round(k*t_est/t_ob_global);
          //     if k_ob > size_T_ob
          //         z_k = z_array(1:3, end);
          //     elseif k_ob ==0
          //         z_k = z_array(1:3, 1);
          //     else
          //         z_k = z_array(1:3, k_ob);
          //     end
          //
          //     x_k_k_1 = G_T*x_k_k + H_T* u_k;
          //     P_k_k_1 = G_T*P_k_k*G_T' + H_T*R*H_T';
          //
          //     y = z_k -C_T*x_k_k_1;
          //     S=C_T*P_k_k_1*C_T'+ Q; %observation
          //     K=P_k_k_1*C_T'*inv(S);
          //     x_k_k=x_k_k_1+K*y;
          //     P_k_k=(eye(6,6)-K*C_T)*P_k_k_1;
          //     x_array_est(:,k)=x_k_k;

               Eigen::Matrix3d  eye3, G_T, C_T;
               Eigen::Vector3d omega_s,  v_s; //angular velocity of the sensor relative to earth, and the velocity relative to earth
               double deltat = 0.0333333;  //sampling time
               eye3.setIdentity();
               omega_s << velk.twist.angular.x, velk.twist.angular.y, velk.twist.angular.z;
               v_s << velk.twist.linear.x, velk.twist.linear.y, velk.twist.linear.z;

               //omega_s = Eigen::Vector3d::Zero();
               //v_s = Eigen::Vector3d::Zero();
               //omega_s = Eigen::Vector3d::Random(3,1);
               //v_s = Eigen::Vector3d::Random(3,1);

               Eigen::Matrix3d omega_s_hat(3,3), R_s_E;  //rotation matrix of sensor relative to earth frame.
               Eigen::Matrix3d  Q_variance;  //covariance of feature position in the sensor frame
               Eigen::MatrixXd R_variance(6,6);  //covariance of angular velocity and linear velocity
               R_variance  <<  0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0,
                         0, 0, 0, 0, 0, 0,
                         0, 0, 0, 0, 0, 0,
                         0, 0, 0, 0, 0, 0,
                         0, 0, 0, 0, 0, 0;

               R_variance  << 0.1, 0, 0, 0, 0, 0,
                              0, 0.1, 0, 0, 0, 0,
                              0, 0, 0.1, 0, 0, 0,
                              0, 0, 0, 0.1, 0, 0,
                              0, 0, 0, 0, 0.1, 0,
                              0, 0, 0, 0, 0, 0.1;

               Eigen::Matrix3d J_Mtinv;
               J_Mtinv = J_M.transpose()*J_M;
          // J_Mtinv = J_Mtinv.inverse();  //needs to be revised, current equation cannot be inverted, Dec, 25, 2021
               double sigma_var = 0.1; //the variance of the feature points in pixel frame
               //Q_variance = sigma_var*sigma_var*J_Mtinv; //see notebook
               //Q_variance = Eigen::Matrix3d::Random(3,3);  //how to solve it?

               Eigen::Matrix3d vari_pix_z;
               if (ifdetection == 1)
               vari_pix_z << 0.1, 0, 0,
                         0, sigma_feature[0], 0,
                              0, 0, sigma_feature[1];  //from the yoloros, the uncertainty
               else
               vari_pix_z << 0.1, 0, 0,
                                   0, 0.18, 0,
                                   0, 0, 0.18;  //from the yoloros, the uncertainty

               Q_variance(0,0) = 0.1;  //the variance of x component.
               Q_variance.block<2,2>(1,1) = pfpuvx23row*vari_pix_z*pfpuvx23row.adjoint();
               //cout << "Q_variance" << Q_variance << endl;

               Eigen::Matrix3d  S, K;
               Eigen::Quaterniond q_3(pose_global.pose.orientation.w,pose_global.pose.orientation.x ,pose_global.pose.orientation.y,pose_global.pose.orientation.z);
               q_3.normalize();
               R_s_E = q_3.toRotationMatrix();

               omega_s_hat << 0, -omega_s(2), omega_s(1),
                    omega_s(2), 0, -omega_s(0),
                    -omega_s(1), omega_s(0), 0;

               Eigen::Matrix3d p_hat(3,3);
               p_hat << 0, -x_k_k(2), x_k_k(1),
                         x_k_k(2), 0, -x_k_k(0),
                         -x_k_k(1), x_k_k(0), 0;

               G_T =  -deltat*omega_s_hat+eye3;
               Eigen::MatrixXd   H_T(3,6);
               H_T << p_hat, -eye3;
               H_T = -deltat*H_T;
               C_T = eye3;
               Eigen::VectorXd  u_k(6);  //the velocity of sensor frame relative to the earth frame, in the sensor frame
               u_k << omega_s, v_s;
               //x_k_k = G_T*x_k_k + H_T* u_k;  //linear
               x_k_k = deltat*(x_k_k.cross(omega_s) - v_s) + x_k_k;  //non-linear
               P_k_k = G_T*P_k_k*G_T.transpose() + H_T*R_variance*H_T.transpose();

               Eigen::Vector3d y, z_k;

               z_k = p_f_L; //output from yolov3
               y = z_k - C_T*x_k_k;   //nonlinear
               S=C_T*P_k_k*C_T.transpose() + Q_variance; //observation
          // S = 0.01*eye3;
               K = P_k_k*C_T.transpose()*S.inverse();
          // K = 0.1*eye3;
               x_k_k = x_k_k + K*y;
               P_k_k=( eye3 -K*C_T)*P_k_k;


          //     cout << "input: " << v_s << omega_s << endl;
          //     cout << "S:" << S <<endl;
          //     cout << "P_k_k:" << P_k_k <<endl;
               //cout << "z_k:" << z_k <<endl;
          //     cout << " C_T:" << C_T << endl;
          //     cout << " G_T:"  << G_T << endl;
          //     cout << "H_T: " << H_T << endl;
          //     cout << "K:" << K << endl;
               //cout << "y in Kalman filter: " << y(0)  << ", " << y(1)  << ", "   << y(2)  << endl;
               //cout << "After Kalman filter: " << x_k_k(0)  << ", " << x_k_k(1)  << ", "   << x_k_k(2)  << endl;
               Vector3d p_b = q_bc*(R_CL*x_k_k + t_CL)+t_bc;

               geometry_msgs::PointStamped p_box;
               p_box.point.x = p_b(0);
               p_box.point.y = p_b(1);
               p_box.point.z = p_b(2);
               v_ekf.publish(p_box);
          }
          
     }
}

void detectCallback(const  darknet_ros_msgs::BoundingBoxes::ConstPtr& msg){
     darknet_ros_msgs::BoundingBoxes boundingBoxesResults_ = *msg;
     int boxno = boundingBoxesResults_.bounding_boxes.size();

     cout << "you catch me!!!!" << endl;
     m_feature.lock();
     feat_point[0] = (boundingBoxesResults_.bounding_boxes[0].xmin + boundingBoxesResults_.bounding_boxes[0].xmax)/2;
     feat_point[1] = (boundingBoxesResults_.bounding_boxes[0].ymin + boundingBoxesResults_.bounding_boxes[0].ymax)/2;
     m_feature.unlock();
     sigma_feature[0] = boundingBoxesResults_.bounding_boxes[0].utx;
     sigma_feature[1] = boundingBoxesResults_.bounding_boxes[0].uty;
     ifdetection = 1;

//	    for (int i = 0; i < boxno; i++) {
//
//	        darknet_ros_msgs::BoundingBox boundingBox;
//
//	        for (int j = 0; j < rosBoxCounter_[i]; j++) {
//	          int xmin = (rosBoxes_[i][j].x - rosBoxes_[i][j].w / 2) * frameWidth_;
//	          int ymin = (rosBoxes_[i][j].y - rosBoxes_[i][j].h / 2) * frameHeight_;
//	          int xmax = (rosBoxes_[i][j].x + rosBoxes_[i][j].w / 2) * frameWidth_;
//	          int ymax = (rosBoxes_[i][j].y + rosBoxes_[i][j].h / 2) * frameHeight_;
//
//	          boundingBox.Class = classLabels_[i];
//	          boundingBox.id = i;
//	          boundingBox.probability = rosBoxes_[i][j].prob;
//	          boundingBox.xmin = xmin;
//	          boundingBox.ymin = ymin;
//	          boundingBox.xmax = xmax;
//	          boundingBox.ymax = ymax;
//	          boundingBox.utx = rosBoxes_[i][j].utx;
//	          boundingBox.uty = rosBoxes_[i][j].uty;
//	          boundingBox.utw = rosBoxes_[i][j].utw;
//	          boundingBox.uth = rosBoxes_[i][j].uth;
//	          boundingBoxesResults_.bounding_boxes.push_back(boundingBox);
//	        }
//
//	    }
}

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
     std::thread prepro_pc = std::thread(&Preprocess);
     ros::spin();
     //prepro_pc.join();
     return 0;
}
