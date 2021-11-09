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
//int w_img = 1280, h_img = 720, c_img =3;
int w_img = 672, h_img = 376, c_img =3; //1105 dataset
int i_pc_count = 0;
int i_img_count = 0;
int sum_pc = 2;
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
   for (int i=0; i< cloud.points.size(); i++){
    	Eigen::VectorXd pc_i(4);
    	Eigen::MatrixXd T_pc_ima(4,4);
//    	T_pc_ima <<
//    	649.145832480000,	-527.021077203120,	-103.253274120000,	268.290316440000,
//    	242.802673396000,	-25.6337212953540,	-585.644625159000,	68.5356940330000,
//    	0.980644000000000,	0.000526794000000000,	-0.195801000000000,	0.283747000000000,
//		0,	0,	0,	1; //from calibration

    	T_pc_ima <<
		339.772898240000,	-263.502373294560,	-54.6615525600000,	138.543236720000,
		128.756166698000,	-12.8129096926770,	-294.290820079500,	36.3959495165000,
		0.980644000000000,	0.000526794000000000,	-0.195801000000000,	0.283747000000000,
		0,	0,	0,	1; //1105

    	pc_i<< cloud.points[i].x, cloud.points[i].y, cloud.points[i].z, 1;
    	Eigen::VectorXd pix_pc(4);
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

            if  (thispoint.x_3d < point_min.x) { point_min.x = thispoint.x_3d; }
            if  (thispoint.y_3d < point_min.y) { point_min.y = thispoint.y_3d; }
            if  (thispoint.z_3d < point_min.z) { point_min.z = thispoint.z_3d; }

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
       i_pc_count ++;
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
     }

   chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
   chrono::duration<double> time_used = chrono::duration_cast < chrono::duration < double >> (t2 - t1);
   cout << "preprocessing time: " << time_used.count() << " s." << endl;

}

void imgCallback(const  sensor_msgs::ImageConstPtr& msg)
{
	cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

    //cv::Mat img  = cv_ptr -> image;
    img  = cv_ptr -> image;

 	// cv::imshow("image", img);
	// cv::waitKey(1);
//    char img1[50];
//    sprintf(img1, "/tmp/%02dimg.png",i_img_count);
//    cv::imwrite(img1, img); //save the image
//    i_img_count ++;

	w_img = img.cols;
	h_img = img.rows;
	c_img = img.channels();
    ROS_INFO("image width: %d, height: %d, channels: %d.", w_img, h_img, c_img);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pc_preprocessing");
  ros::NodeHandle n;

  ros::Subscriber subpc = n.subscribe("/livox/lidar", 50, pc2Callback);
  ros::Subscriber subimg = n.subscribe("/zed2/zed_node/left/image_rect_color", 1000, imgCallback);

  ros::spin();

  return 0;
}
