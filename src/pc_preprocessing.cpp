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

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/PointCloud2.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/point_types.h"
#include "pcl/PCLPointCloud2.h"
#include "pcl/conversions.h"
#include "pcl_ros/transforms.h"
#include <vector>
#include <sstream>
#include "imageBasics.h"

using namespace std;
void pc2Callback(const  sensor_msgs::PointCloud2::ConstPtr& msg)
{
    ROS_INFO("pc received");

    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg (*msg, cloud);

    ROS_INFO("cloud.width: %d", cloud.width);
    ROS_INFO("cloud.height: %d", cloud.height); //if height ==1, it is 1-D point cloud
    ROS_INFO("cloud size: %d",cloud.points.size());

    vector<pcl::PointXYZ> data;

    pcl::PointXYZ point;
          point.x = 2.0f;
          point.y = 1;
          point.z = 3;
          cloud.points.push_back(point);

          ROS_INFO("cloud.width: %d", cloud.width);
              ROS_INFO("cloud.height: %d", cloud.height); //if height ==1, it is 1-D point cloud
              ROS_INFO("cloud size: %d",cloud.points.size());

    ROS_INFO("x: %f", cloud.points[0].x);

    vector<pointcoordinate> pc_array;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pc_preprocessing");
  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("/livox/lidar", 1000, pc2Callback);

  ros::spin();

  return 0;
}
