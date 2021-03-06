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

using namespace std;
//int w_img = 1280, h_img = 720, c_img =3;
int w_img = 672, h_img = 376, c_img =3; //1105 dataset
int i_pc_count = 0;
int i_img_count = 0;
int sum_pc = 4;
int sum_pc_i = 0;
long int pc_size = 0;
pcl::PointCloud<pcl::PointXYZ> cloud;
vector<pointcoordinate> pc_array;
//vector<pointcoordinate> pc_array_grid[921600];
vector<pointcoordinate> pc_array_grid[252672];
//vector<pointcoordinate> pc_array_grid[w_img*h_img];
pcl::PointXYZ point_max(0,0,0); //the maximum value of XYZ channels
pcl::PointXYZ point_min(0,0,0); //the minimum value of XYZ channels
//struct
//{
//	double umax{0.0};
//	double umin{0.0};
//	double vmax{0.0};
//	double vmin{0.0};
//} minmaxuv;
minmaxuv_ minmaxuv;
cv::Mat img;
vector<geometry_msgs::PoseStamped>  pose_series;
geometry_msgs::PoseStamped  pose_global;
sensor_msgs::Image img1;
double feat_point[2] = {300,150}; //the feature position in the pixel frame, detected by the detector
vector<pointcoordinate> pc_array_feature; //put the feature points in the array
Eigen::Vector3d x_k_k;
Eigen::Matrix3d P_k_k = Eigen::Matrix3d::Identity();
int flag_int_xkk = 0;
int flag_int_xkk_last = 0;
ros::Publisher pubimg;
ros::Publisher pubimg_upsample;
sensor_msgs::Image imgrgb;
float sigma_feature[2]={0,0}; //the uncertainty of feature in pixel frame
int ifdetection = 0 ;

void pc2Callback(const  sensor_msgs::PointCloud2::ConstPtr& msg)
{
	chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    ROS_INFO("pc received");

    //pcl::PointCloud<pcl::PointXYZ> cloud[sum_pc];
    pcl::fromROSMsg (*msg, cloud);
    sum_pc_i ++;
    pc_size += cloud.points.size();

   // ROS_INFO("cloud.width: %d", cloud.width);
    //ROS_INFO("cloud.height: %d", cloud.height); //if height ==1, it is 1-D point cloud
    //ROS_INFO("cloud size: %d",cloud.points.size());

//    vector<pcl::PointXYZ> data;
//
//    pcl::PointXYZ point;
//    point.x = 2.0f;
//    point.y = 1;
//    point.z = 3;
//    cloud.points.push_back(point);
//
//    ROS_INFO("cloud.width: %d", cloud.width);
//    ROS_INFO("cloud.height: %d", cloud.height); //if height ==1, it is 1-D point cloud
//    ROS_INFO("cloud size: %d",cloud.points.size());
//
//    ROS_INFO("x: %f", cloud.points[0].x);

   int grid = 5;
   Eigen::VectorXd pc_i(4);
       	Eigen::MatrixXd T_pc_ima(4,4);
//       	T_pc_ima <<
//       	649.145832480000,	-527.021077203120,	-103.253274120000,	268.290316440000,
//       	242.802673396000,	-25.6337212953540,	-585.644625159000,	68.5356940330000,
//       	0.980644000000000,	0.000526794000000000,	-0.195801000000000,	0.283747000000000,
//       	0,	0,	0,	1; //from calibration

       	    	T_pc_ima <<
       			339.772898240000,	-263.502373294560,	-54.6615525600000,	138.543236720000,
       			128.756166698000,	-12.8129096926770,	-294.290820079500,	36.3959495165000,
       			0.980644000000000,	0.000526794000000000,	-0.195801000000000,	0.283747000000000,
       			0,	0,	0,	1; //1105

       	Eigen::VectorXd pix_pc(4);

   for (int i=0; i< cloud.points.size(); i++){
	    pc_i<< cloud.points[i].x, cloud.points[i].y, cloud.points[i].z, 1;
	    pix_pc = T_pc_ima*pc_i;

    	pix_pc[0] = pix_pc[0]/pix_pc[2];
    	pix_pc[1] = pix_pc[1]/pix_pc[2];

      	pointcoordinate thispoint;

        if(  pix_pc[0] >= 1  && (int)pix_pc[0] <= w_img-1 &&  pix_pc[1] >= 1  && (int)pix_pc[1] < h_img -1){
          	thispoint.x_3d = cloud.points[i].x;
          	thispoint.y_3d = cloud.points[i].y;
          	thispoint.z_3d = cloud.points[i].z;
          	thispoint.u_px = pix_pc[0];
          	thispoint.v_px = pix_pc[1];
          	//pc_array.push_back(thispoint);

            if  (thispoint.x_3d > point_max.x) { point_max.x = thispoint.x_3d; }
            if  (thispoint.y_3d > point_max.y) { point_max.y = thispoint.y_3d; }
            if  (thispoint.z_3d > point_max.z) { point_max.z = thispoint.z_3d; }

//            if  (thispoint.x_3d < point_min.x) { point_min.x = thispoint.x_3d; }
//            if  (thispoint.y_3d < point_min.y) { point_min.y = thispoint.y_3d; }
//            if  (thispoint.z_3d < point_min.z) { point_min.z = thispoint.z_3d; }

            if  (thispoint.u_px > minmaxuv.umax) {minmaxuv.umax = thispoint.u_px;}
            if  (thispoint.u_px < minmaxuv.umin) {minmaxuv.umin = thispoint.u_px; }
            if  (thispoint.v_px > minmaxuv.vmax) {minmaxuv.vmax = thispoint.v_px;}
            if  (thispoint.v_px < minmaxuv.vmin) {minmaxuv.vmin = thispoint.v_px;}

            int test = max(2, 4);
            for (int u = max( (int)(pix_pc[0]- grid), 0); u < min ( (int)(pix_pc[0] + grid), w_img); u++)
            	for (int v = max( (int)( pix_pc[1]- grid), 0);  v < min ( (int)(pix_pc[1] + grid), h_img); v++){
            		pc_array_grid[v*w_img+u].push_back(thispoint);
            	}

//            for (int u = 0; u < w_img; u++)
//                 for (int v = 0; v < h_img; v++){
//                	 pc_array_grid[v*w_img+u].push_back(thispoint);
//                 }
            //cout << "u_px: " << pix_pc[0] << ". v: " << pix_pc[1] << endl;
        }
    }

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

       char img1[50];
       sprintf(img1, "/tmp/%02dimg.png",i_pc_count);
       cv::imwrite(img1, img); //save the image
       i_pc_count ++;
     }

   chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
   chrono::duration<double> time_used = chrono::duration_cast < chrono::duration < double >> (t2 - t1);
   cout << "preprocessing time: " << time_used.count() << " s." << endl;

}

void imgCallback(const  sensor_msgs::ImageConstPtr& msg)
{
	cv_bridge::CvImagePtr cv_ptr;

	//cout << "encoding: " << msg->encoding << endl; //bgr8a
	//if(msg->encoding == "mono8" || msg->encoding == "bgr8" || msg->encoding == "rgb8"){
      //cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      //cv::Mat img  = cv_ptr -> image;
      img  = cv_ptr -> image;

      imgrgb = *msg;

     // cv::imshow("image", img);
	 // cv::waitKey(1);
//    char img1[50];
//    sprintf(img1, "/tmp/%02dimg.png",i_img_count);
//    cv::imwrite(img1, img); //save the image
//    i_img_count ++;

	  w_img = img.cols;
	  h_img = img.rows;
	  c_img = img.channels();
      //ROS_INFO("image width: %d, height: %d, channels: %d.", w_img, h_img, c_img);
	//}
}

void poseCallback(const  geometry_msgs::PoseStamped::ConstPtr& msg)
{
	 geometry_msgs::PoseStamped posek;

	 posek = *msg;
	 pose_series.push_back(posek);
	// pose_global.pose = msg->pose;
	 pose_global = *msg;
}

void velCallback(const  geometry_msgs::TwistStamped::ConstPtr& msg)
{
	 geometry_msgs::TwistStamped velk;

	 velk = *msg;

     int grid_ave = 50;
     int w_img = 3;
     double sum_x = 0;
     int sum_no = 0; // the total points near the feature point
	 //calculate the x coordinate of the feature point
	 /*for (int u =   (int)(feat_point[0]- grid_ave); u <   (int)(feat_point[0]+ grid_ave); u++)
	      for (int v =   (int)(feat_point[1]- grid_ave); v < (int)(feat_point[1] + grid_ave); v++)
	    	  for (int i_pc =  0; i_pc < pc_array_grid[v*w_img+u].size(); i_pc++){
	    		  sum_x =  pc_array_grid[v*w_img+u][i_pc].x_3d + sum_x;
	    		  sum_no++;
	    	  }*/

     if (pc_array_feature.size() == 0){
    	 cerr << "No point near the feature region." << endl;
    	 sum_no = 1;
     }
     else{

     for (int i_pc =  0; i_pc < pc_array_feature.size(); i_pc++){
     	    		  sum_x = pc_array_feature[i_pc].x_3d + sum_x;
     	    		  sum_no++;
     	    	  }
     if (pc_array_feature.size() == 0){
    	 cerr << "No point near the feature region." << endl;
    	 sum_no = 1;
     }
	 double x_f_l = sum_x/sum_no;
	 sum_no  = 0;
	 sum_x = 0;

	 //cout << "feature point in LiDAR frame, x coordinate: " << x_f_l << endl;
	 int ii = feat_point[1]*w_img+feat_point[0];
	 cout << "Count of point near the feature region: " << pc_array_feature.size() << endl;

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

	 double fx, fy, cx, cy; //the intrinsic parameters of the camera
	 fx = 264.0;
	 fy = 263.700000000000;
	 cx = 343.760000000000;
	 cy = 183.879500000000;

	 Eigen::Matrix3d fp_tra(3,3);   //see notebook
     fp_tra << (feat_point[0]-343.76)/264,   0.998801000000000,  -0.0479053000000000,
    		 (feat_point[1] -183.8795)/263.70,   0.0489563000000000,  0.979473000000000,
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
     Eigen::Vector3d pfpuvx3 =  -fp_tra.inverse()*(r1 + t_c_L);
     Eigen::Matrix3d pfpuvx;
     pfpuvx << pfpuvx1,pfpuvx2, pfpuvx3;
     Eigen::MatrixXd pfpuvx23row(2,3);
     pfpuvx23row = pfpuvx.block<2,3>(1,0);

     double z_f_c = v_mi(0);
     double x_f_c = (feat_point[0] - 343.76)* z_f_c /264;
     double y_f_c = (feat_point[1] - 183.8795)* z_f_c /263.700;
     Eigen::Vector3d p_f_L, p_f_c;
     p_f_L << x_f_l, v_mi(1), v_mi(2);
     p_f_c << x_f_c, y_f_c, z_f_c;


     flag_int_xkk_last = flag_int_xkk;
     flag_int_xkk = 1;

     cout << "feature point in LiDAR frame: " << p_f_L(0)  << ", " << p_f_L(1)  << ", "   << p_f_L(2)  << endl;
     cout << "feature point in camera frame: " << p_f_c(0)  << ", "  << p_f_c(1) << ", "  << p_f_c(2)  << endl;

     //the covariance calculation:

     if ((flag_int_xkk_last == 0) & (flag_int_xkk == 1)){
    	 x_k_k = p_f_L;  //initialization of the variable in kalman filter
     }

     if (flag_int_xkk == 1){
     Eigen::MatrixXd J_M(2,3);
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

     R_variance  <<  0.1, 0, 0, 0, 0, 0,
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
     cout << "Q_variance" << Q_variance << endl;

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
     cout << "z_k:" << z_k <<endl;
//     cout << " C_T:" << C_T << endl;
//     cout << " G_T:"  << G_T << endl;
//     cout << "H_T: " << H_T << endl;
//     cout << "K:" << K << endl;
     cout << "y in Kalman filter: " << y(0)  << ", " << y(1)  << ", "   << y(2)  << endl;
     cout << "After Kalman filter: " << x_k_k(0)  << ", " << x_k_k(1)  << ", "   << x_k_k(2)  << endl;
     }
     }
}

void detectCallback(const  darknet_ros_msgs::BoundingBoxes::ConstPtr& msg){
	 darknet_ros_msgs::BoundingBoxes boundingBoxesResults_ = *msg;
	 int boxno = msg->bounding_boxes.size();

	 cout << "box no: " << boxno << endl;

	 //feat_point[0] = (boundingBoxesResults_.bounding_boxes[0].xmin + boundingBoxesResults_.bounding_boxes[0].xmax)/2;
	 //feat_point[1] = (boundingBoxesResults_.bounding_boxes[0].ymin + boundingBoxesResults_.bounding_boxes[0].ymax)/2;
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
  ros::NodeHandle n;

  ros::Subscriber subpc = n.subscribe("/livox/lidar", 500, pc2Callback);
  ros::Subscriber subimg = n.subscribe("/zed2/zed_node/left/image_rect_color", 1000, imgCallback);
  ros::Subscriber subpos = n.subscribe("/mavros/local_position/pose", 1000, poseCallback);
  ros::Subscriber subvel = n.subscribe("/mavros/local_position/velocity_body", 1000, velCallback);

  ros::Subscriber subdetection = n.subscribe("/darknet_ros/bounding_boxes", 1000, detectCallback);  //dependency: darknet_ros_msgs

  pubimg = n.advertise<sensor_msgs::Image>("/camera/rgb/image_raw",  1000);
  pubimg_upsample = n.advertise<sensor_msgs::Image>("/camera/xyz/image_upsampling",  1000);


  ros::spin();

  return 0;
}
