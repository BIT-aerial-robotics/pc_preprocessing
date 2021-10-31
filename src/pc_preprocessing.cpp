/*
 * Copyright (C) 2021, Chuanbeibei Shi and Yushu Yu
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

int w_img, h_img, c_img;
//int upsampling(vector<pointcoordinate> &pc_array, int w, int h, int c, long int *no);



void pc2Callback(const  sensor_msgs::PointCloud2::ConstPtr& msg)
{
    ROS_INFO("pc received");

    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg (*msg, cloud);

    ROS_INFO("cloud.width: %d", cloud.width);
    ROS_INFO("cloud.height: %d", cloud.height); //if height ==1, it is 1-D point cloud
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

    vector<pointcoordinate> pc_array;
    pcl::PointXYZ point_max; //the maximum value of XYZ channels

   for (int i=0; i<cloud.points.size(); i++){
    	Eigen::VectorXd pc_i(4);
    	Eigen::MatrixXd T_pc_ima(4,4);
    	T_pc_ima <<
    	649.145832480000,	-527.021077203120,	-103.253274120000,	268.290316440000,
    	242.802673396000,	-25.6337212953540,	-585.644625159000,	68.5356940330000,
    	0.980644000000000,	0.000526794000000000,	-0.195801000000000,	0.283747000000000,
    	0,	0,	0,	1; //from calibration
    	pc_i<< cloud.points[i].x, cloud.points[i].y, cloud.points[i].z, 1;
    	Eigen::VectorXd pix_pc(4);
    	pix_pc = T_pc_ima*pc_i;
    	pix_pc[0] = pix_pc[0]/pix_pc[2];
    	pix_pc[1] = pix_pc[1]/pix_pc[2];

      	pointcoordinate thispoint;


        if(  pix_pc[0] > 0  && (int)pix_pc[0] <= w_img &&  pix_pc[1] > 0  && (int)pix_pc[1] <= h_img){
          	thispoint.x_3d = cloud.points[i].x;
          	thispoint.y_3d = cloud.points[i].y;
          	thispoint.z_3d = cloud.points[i].z;
          	thispoint.u_px = pix_pc[0];
          	thispoint.v_px = pix_pc[1];
          	pc_array.push_back(thispoint);

            if  (thispoint.x_3d> point_max.x) { point_max.x = thispoint.x_3d; }
            if  (thispoint.y_3d> point_max.y) { point_max.y = thispoint.y_3d; }
            if  (thispoint.z_3d> point_max.z) { point_max.z = thispoint.z_3d; }
        }
    }

   ROS_INFO("array size: %d", pc_array.size());

   int no_ini = 0;

   int ups = upsampling_pro(pc_array, point_max, w_img, h_img, c_img, &no_ini);

}

void imgCallback(const  sensor_msgs::ImageConstPtr& msg)
{
	cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

    const cv::Mat img = cv_ptr -> image;
    //cv::Mat img = cv::Mat(170, 500, CV_8UC3, cv::Scalar(0, 0, 0));

	//cv::Mat img;
	cv::imshow("image", img);
	cv::waitKey(1);
	w_img = img.cols;
	h_img = img.rows;
	c_img = img.channels();
    ROS_INFO("image width: %d, height: %d, channels: %d.", w_img, h_img, c_img);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pc_preprocessing");
  ros::NodeHandle n;

  ros::Subscriber subpc = n.subscribe("/livox/lidar", 1, pc2Callback);
  ros::Subscriber subimg = n.subscribe("/zed2/zed_node/left/image_rect_color", 50, imgCallback);

  ros::spin();

  return 0;
}
