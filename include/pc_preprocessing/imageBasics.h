/*
 * Copyright (C) 2021 Chuanbeibei Shi and Yushu Yu
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef IMAGEBASICS_H_
#define IMAGEBASICS_H_

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/Image.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/point_types.h"
#include "pcl/PCLPointCloud2.h"
#include "pcl/conversions.h"
#include "pcl_ros/transforms.h"
#include <vector>
#include <queue>
#include <map>
#include <unordered_map>

#include <sstream>
#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <Eigen/Dense>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <mutex>
#include <thread>
#include <pthread.h>
#include <condition_variable>
// darknet_ros_msgs
#include <darknet_ros_msgs/BoundingBox.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <darknet_ros_msgs/CheckForObjectsAction.h>
#include <darknet_ros_msgs/ObjectCount.h>
//livox-sdk
#include <livox_ros_driver/CustomMsg.h>
//并行
#include <omp.h>
//绘图
#include "matplotlibcpp.h"

using namespace std;
using namespace Eigen;
namespace plt = matplotlibcpp;
int w_img = 672, h_img = 376, c_img =3;
const int WINDOW_SIZE = 4;//0.4s
Matrix3d R_cam_lidar, K_in;
Vector3d t_cam_lidar;
Matrix3d R_Imu_Lidar, R_cam_imu, R_imu_lidar;
Vector3d t_imu_lidar;
Vector3d t_cam_imu;

//controll image saving frequency
double cur_setfre = 0;
double last_setfre = 0;
//intrinsic parameters of camera
double fx = 264.0;
double fy = 263.700000000000;
double cx = 343.760000000000;
double cy = 183.879500000000; 
enum ESampleType
{
	NOISE = 1,
	BORDER = 2,
	CORE = 3,
};
struct point
{
public:
	float x;
	float y;
    int num = 0;
    int id = 0;
    double depth_ave = 0 ;
	int cluster = 0; //所属类别（一个标识代号，属于同一类的样本具有相同的cluster）

	//邻域半径R内样本点的数量大于等于minpoints的点叫做核心点。
	//不属于核心点但在某个核心点的邻域内的点叫做边界点。既不是核心点也不是边界点的是噪声点
	int pointType = NOISE;		 // 1:noise 	2:border 	3:core  （初始默认为噪声点）
	int pts = 0;				 //points in MinPts （指定领域内样本点的个数）
	vector<int> neighborCoreIdx; //对所有的corepoint，将其eps范围内的core point下标添加到vector<int> neighborCoreIdx中
	int visited = 0;			 //是否被遍历访问过

	point()
	{
	}
	point(float a, float b, int _id)
	{
		x = a;
		y = b;
        id = _id;
		//cluster = c;
	}
};


class pointcoordinate
{
public:
    //pointcoordinate(){}
    //pointcoordinate(pcl::PointCloud<pcl::PointXYZ>& _pc):pc(_pc){}
    double x_3d{0.0};
    double y_3d{0.0};
    double z_3d{0.0};
    double u_px{0.0};
    double v_px{0.0};
    double t_offset;
    double Gr_x=0, Gr_y=0, Gr_z=0;
    double G_x = 0, G_y = 0, G_z=0;
    //Eigen::aligned_allocator<pcl::PointXYZ>& point;
    void print()
    {
        std::cout << "3D coordinate of this point: [" << x_3d << "; " << y_3d << "; " << z_3d <<  "]" << std::endl;
        std::cout << "2D coordinate of this point in pixel frame: [" << u_px << "; " << v_px <<  "]" << std::endl;
    }
};

struct Mask_pc{
    Mask_pc(){}
    Mask_pc(pointcoordinate _point,int _grid){
        SetMask(_point, _grid);
        //valid_pixles = (u_up-u_down)*(v_up-v_down); 
    }
    void SetMask(pointcoordinate _point,int _grid){
        point = _point;
        grid = _grid;
        u_down =  (int)(point.u_px- grid);
        u_up = (int)(point.u_px + grid);
        v_down = (int)( point.v_px- grid);
        v_up = (int)(point.v_px + grid);
        //valid_pixles = (u_up-u_down)*(v_up-v_down); 
    }
    int u_up, u_down, v_up, v_down;//[down,up)
    double timestamp;//second
    pointcoordinate point;
    int grid;
    int valid_pixles;

};

typedef struct
{
	double umax{450};
	double umin{260};
	double vmax{270};
	double vmin{85};
} minmaxuv_;  //the initial value should be carefully configured.
/**/
struct ID_MASK{
    public:
    
    ID_MASK(){
        image_size = w_img*h_img*c_img;
        ima3d = (double*)malloc(sizeof(double)*(image_size));
        ima3d_ = (double*)malloc(sizeof(double)*(image_size));
        SetZeros();
        //ROS_INFO_STREAM("ID_MASK recalled!!!!!");
        //pc_masks = (Mask_pc*)malloc(sizeof(Mask_pc)*(1));
    };
    ~ID_MASK(){
        free(ima3d);
        free(ima3d_);
        free(pc_masks);
        
    };
    // void ResetMaskSize(const int size);
    void SetZeros();
    vector<Mask_pc> pc_masks[4];
    vector<Mask_pc> pc_masks_single;
    //Mask_pc* pc_masks;
    int masksize;
    int mask_id;
    int image_size;//equal with image size
    double* ima3d;//denominator
    double* ima3d_;//numerator for deepth map calculating
    double maxima3d[3];
    int start_index, end_index;//[start, end)
    double timestamp;//ns
    Eigen::Quaterniond q_wb;
    Eigen::Vector3d t_wb;
    // Eigen::Matrix4d T_wb;
    // Eigen::Matrix4d T_bw;
    
};



class PC_Wrapper{
    public:
    PC_Wrapper(){
        init = false;
        Maxima3d[0] = 0;
        Maxima3d[1] = 0;
        Maxima3d[2] = 0;
        current_id = 0;
    }
    void UpdateMask(int id);
    void set_init(){init = true;};
    void reset_init(){
        init = false;
        ROS_INFO("Reset Init Flag.");
    };
    void UpdateMax3d();
    ID_MASK mask_win[4];//mask 
    ID_MASK mask4;
    bool init;
    double Maxima3d[3];
    int current_id;
    double timestamp;//ns
    livox_ros_driver::CustomMsg::ConstPtr pc_win_buffer[4];
};





struct Threadsstruct{
    Threadsstruct(){

    }
    std::vector<Eigen::Vector4d> pc_vector;  
    minmaxuv_ minmaxuv_thread;
    pcl::PointXYZ point_max_thread;
    vector<Mask_pc> pc_mask_thread;
    int start_index, end_index;//[start, end)
    int thread_id;
};

bool compare_pc_v(const pointcoordinate& left,const pointcoordinate& right);
int upsampling_pro(pcl::PointXYZ &maxxyz, pcl::PointXYZ &minxyz, minmaxuv_ &minmaxuv, int w, int h, int c, int nof);
void* multi_thread_preprocess(void* threadsstruct);
extern ros::Publisher pubimg;


class TicToc
{
  public:
    TicToc()
    {
        tic();
    }

    void tic()
    {
        start = std::chrono::system_clock::now();
    }

    double toc()
    {
        end = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsed_seconds = end - start;
        return elapsed_seconds.count() * 1000;
    }

  private:
    std::chrono::time_point<std::chrono::system_clock> start, end;
};

struct Pc_Vector{

    vector<Vector3d> pc_lidar_3d;
    vector<Vector2d> pc_uv;
    long long timestamp;
    Eigen::Quaterniond q_wb;
    Eigen::Vector3d t_wb;
};


#endif 



/******************************************************/

//int w_img = 1280, h_img = 720, c_img =3;
 //1105 dataset
int i_pc_count = 0;
int i_img_count = 0;
int sum_pc = 4;
int sum_pc_i = 0;
double sum_x = 0;

pcl::PointCloud<pcl::PointXYZ> cloud;
queue<livox_ros_driver::CustomMsg::ConstPtr> pc_buffer;
queue<darknet_ros_msgs::BoundingBoxes::ConstPtr> box_buffer;
queue<Pc_Vector> yolo_pc_buffer;
double pc_time_cur = 0;
double pc_time_last = 0;
queue<cv::Mat> cvMat_buffer;
queue<sensor_msgs::Image> img_buffer;
// lock when ekf state is freshing
std::mutex m_ekf;
// control the box buffer, wake up yolo_update thread when there are box datas.
std::mutex m_yolo;
std::mutex m_buf, m_feature, m_thread;
std::condition_variable con;
std::condition_variable yolo_con;

vector<pointcoordinate> pc_array;
//vector<pointcoordinate> pc_array_grid[921600];
//vector<pointcoordinate> pc_array_grid[252672];
//vector<Mask_pc> pc_masks;
PC_Wrapper pc_manager;
TicToc all_time;
const int thread_num = 4;
//unordered_map<int, std::vector<pointcoordinate>> pc_array_grid;

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
cv::Mat img_cur;
queue<geometry_msgs::PoseStamped>  pose_series;
geometry_msgs::PoseStamped  pose_global;
//drone states
Vector3d p_drone, p_drone_cur, p_origin;
long long cur_timestamp = 0;
long long pose_timestamp = 0;
Quaterniond q_drone, q_drone_cur, q_origin;
//T_b0_w
Matrix4d T_origin = Matrix4d::Identity();
sensor_msgs::Image img1;
double feat_point[3] = {300,150, 0}; //the feature position in the pixel frame, detected by the detector
double rect_feat_point[3] = {300,150, 0};
vector<pointcoordinate> pc_array_feature; //put the feature points in the array

/***********************************************/
//efk
//feat_point
double u0;
double v0;
double bottle = 1;//threshold for depth estimate in yolo box
Eigen::Vector3d x_k_k = Vector3d(0,0,0);
Eigen::Matrix3d P_k_k = Eigen::Matrix3d::Identity();
int flag_int_xkk = 0;
int flag_int_xkk_last = 0;
Eigen::Vector3d p_f_L, p_f_c;
Eigen::Matrix3d  eye3 = Matrix3d::Identity();
Eigen::Matrix3d  G_T;
Matrix3d C_T = Matrix3d::Identity();
Eigen::Matrix3d  Q_variance;  //covariance of feature position in the sensor frame
Eigen::MatrixXd R_variance(6,6);  //covariance of angular velocity and linear velocity

queue<Pc_Vector> yolo_depth;
int up_size = 20;//队列大小
bool data_matched = false;//判断yolo检测和点云数据是否匹配成功
long long last_yolo_timestamp = 0;
//yolo box中计算的点云中心
// double ave_x_last = 0;
// double ave_y_last = 0;
// double ave_z_last = 0;
Vector2d box_center_in_image;
cv::Point2d circle_center;//visualization 
cv::Point2d rect_circle_center;
vector<cv::Point2d> box_grid_points;
/*************************************************/

ros::Publisher pubimg;
ros::Publisher pubimg_upsample;
ros::Publisher v_ekf;
sensor_msgs::Image imgrgb;
sensor_msgs::Image imgrgb_cur;
float sigma_feature[2]={0,0}; //the uncertainty of feature in pixel frame
int ifdetection = 0 ;
Quaterniond q_bc = Quaterniond(-0.5, 0.5, -0.5, 0.5);
Vector3d t_bc = Vector3d::Zero();

int grid = 3;
Eigen::VectorXd pc_i(4);
Eigen::MatrixXd T_pc_ima(4,4);
Eigen::MatrixXd T_cam_lidar(4,4);
pthread_t tids[4];
//points clond threshold
double x_threshold = 100;
double y_threshold = 20;
double z_threshold = 20;

//save image for train
bool image_save = false;
//visualization
bool visualization = false;
bool compare_rect = false;
bool first_loop = false;
/******************************************/

void Preprocess(){
    
    while(true){
            try{
                std::unique_lock<std::mutex> lk(m_buf);
                con.wait(lk,[&]
                    {
                            return (!pc_buffer.empty());
                    });
                
                pc_manager.pc_win_buffer[sum_pc_i] = pc_buffer.back();
                
                img_cur = cvMat_buffer.back();
                imgrgb_cur = (img_buffer.back());
                //todo: check timestamp, pop out out data
                while(!cvMat_buffer.empty())
                    cvMat_buffer.pop();
                while(!img_buffer.empty())
                    img_buffer.pop();
                while(!pc_buffer.empty())
                    pc_buffer.pop();
                pc_manager.UpdateMask(sum_pc_i);
                int cur_id = pc_manager.current_id;
                pc_manager.mask_win[cur_id].q_wb = q_drone_cur;
                pc_manager.mask_win[cur_id].t_wb = p_drone_cur;
                pc_manager.timestamp = cur_timestamp;
                pc_manager.mask4.timestamp = cur_timestamp;

                m_buf.unlock();
                
                        
                all_time.tic();
                ROS_INFO("pc received");
                ROS_DEBUG_STREAM("sum_pc_i: "<<sum_pc_i);
                
                //pcl::fromROSMsg (pc_msg, cloud);
                
                sum_pc_i ++;
                if (sum_pc_i == sum_pc){
                    sum_pc_i = 0;
                    pc_manager.set_init();
                }
                


                // pc_manager.mask_win[cur_id].q_wb = q_drone_cur;
                // pc_manager.mask_win[cur_id].t_wb = p_drone_cur;
                // pc_manager.mask_win[cur_id].T_wb.block<3,3>(0,0) = q_drone_cur.toRotationMatrix();
                // pc_manager.mask_win[cur_id].T_wb.block<3,1>(0,3) = p_drone_cur;
                // pc_manager.mask_win[cur_id].T_bw.block<3,3>(0,0) = q_drone_cur.toRotationMatrix().transpose();
                // pc_manager.mask_win[cur_id].T_bw.block<3,1>(0,3) = -1*pc_manager.mask_win[cur_id].T_bw.block<3,3>(0,0)*p_drone_cur;
                
                //  pc_manager.mask_win[pc_manager.current_id].ResetMaskSize(cloud.points.size());
                if(!pc_manager.init) continue;

                //int grid = 5;
            
                
                

                
                

                TicToc start;
                int outliner=0;

                //multi thread processing
                // Threadsstruct threadsstruct[thread_num];
                // TicToc split_t;
                // int span = cloud.size()/ thread_num;
                // for(int i = 0; i<thread_num; i++){
                    
                //      threadsstruct[i].start_index = i*span;
                //      threadsstruct[i].end_index = (i+1)*span;
                //      threadsstruct[i].thread_id = i;
                //      if(i==3){
                //           threadsstruct[i].end_index = cloud.size();
                //      }
                //      //pc_i<< cloud.points[i].x, cloud.points[i].y, cloud.points[i].z, 1;
                //      //threadsstruct[i%thread_num].pc_vector.emplace_back(cloud.points[i].x, cloud.points[i].y, cloud.points[i].z, 1);
                //      //ROS_INFO_STREAM("point "<< threadsstruct[i%thread_num].pc_vector.back().matrix());
                // }
                // ROS_DEBUG_STREAM("splite time: "<<split_t.toc()<<"ms");

                // TicToc thread_calculate;
                // for(int i = 0; i<4; i++){
                //      int ret = pthread_create( &tids[i], NULL, multi_thread_preprocess ,(void*)&(threadsstruct[i]));
                //      if (ret != 0)
                //      {
                //           ROS_WARN("pthread_create error");
                //           ROS_BREAK();
                //      }
                // }
                

                
                // for( int i = thread_num - 1; i >= 0; i--)  
                // {
                //      pthread_join( tids[i], NULL ); 
                //      if  (minmaxuv.umax < threadsstruct[i].minmaxuv_thread.umax) {minmaxuv.umax = threadsstruct[i].minmaxuv_thread.umax;}
                //      if  (minmaxuv.umin > threadsstruct[i].minmaxuv_thread.umin) {minmaxuv.umin = threadsstruct[i].minmaxuv_thread.umin;}
                //      if  (minmaxuv.vmax < threadsstruct[i].minmaxuv_thread.vmax) {minmaxuv.vmax = threadsstruct[i].minmaxuv_thread.vmax;}
                //      if  (minmaxuv.vmin > threadsstruct[i].minmaxuv_thread.vmin) {minmaxuv.vmin = threadsstruct[i].minmaxuv_thread.vmin;}
                //      for(int j = 0; j<4; j++){
                //           for(int k = 0; k< pc_manager.mask_win[cur_id].pc_masks[j].size(); k++){
                //                pc_manager.mask_win[cur_id].pc_masks_single.push_back(pc_manager.mask_win[cur_id].pc_masks[j][k]);
                                
                //           }
                //           pc_manager.mask_win[cur_id].pc_masks[j].clear();
                //      }
                    
                // }
                // ROS_DEBUG_STREAM("thread calculating: "<<thread_calculate.toc()<<"ms");
                
                
                //single thread
                int pickout_n = 0;
                int pickout_n_100 = 0;
                int valid_num = 0;
                int num_outliner = 0;
                

                //omp_set_num_threads(4);
                

                Pc_Vector pc_vector;
                pc_vector.timestamp = imgrgb_cur.header.stamp.toNSec();//这里不能用cur_timestamp，因为这里已经解锁了，所以这个变量可能已近更新了
                pc_vector.q_wb = pc_manager.mask_win[cur_id].q_wb;
                pc_vector.t_wb = pc_manager.mask_win[cur_id].t_wb;
                //#pragma omp parallel for
                for(int k =0; k<sum_pc; k++){
                
                    int pc_size = pc_manager.pc_win_buffer[k]->points.size();
                    double* ima3d = pc_manager.mask_win[k].ima3d;
                    double* ima3d_ = pc_manager.mask_win[k].ima3d_;
                    livox_ros_driver::CustomMsg::ConstPtr pc_msg = pc_manager.pc_win_buffer[k];
                    //Matrix4d T_cam0_L1 =  pc_manager.mask_win[pc_manager.current_id].T_bw * pc_manager.mask_win[pc_manager.current_id].T_wb * T_imu_lidar;
                    Quaterniond q_b0_b1 = pc_manager.mask_win[cur_id].q_wb.inverse()*pc_manager.mask_win[k].q_wb;
                    Vector3d t_b0_b1 = pc_manager.mask_win[cur_id].q_wb.inverse()*(pc_manager.mask_win[k].t_wb-pc_manager.mask_win[cur_id].t_wb);
                    // ROS_INFO_STREAM("relative p = ("<<t_b0_b1.x()<<", "<<t_b0_b1.y()<<", "<<t_b0_b1.z()<<")");
                    // ROS_INFO_STREAM("relative q = \n"<<q_b0_b1.toRotationMatrix().matrix());
                    Matrix3d R_b0_lidar = q_b0_b1*R_imu_lidar;
                    Vector3d t_b0_lidar = pc_manager.mask_win[cur_id].q_wb.inverse()*(pc_manager.mask_win[k].q_wb*t_imu_lidar) + t_b0_b1;
                    
                    Matrix3d R_lidar0_lidar = R_imu_lidar.inverse()*R_b0_lidar;
                    Vector3d t_lidar0_lidar = R_imu_lidar.inverse()*(t_b0_lidar)-R_imu_lidar.inverse()*t_imu_lidar;
                    Matrix3d R_cam0_lidar = R_cam_imu*R_b0_lidar;
                    Vector3d t_cam0_lidar = (R_cam_imu*(t_b0_lidar)+t_cam_imu);
                    Matrix3d R_uv0_lidar = K_in*R_cam0_lidar;
                    Vector3d t_uv0_lidar = K_in*t_cam0_lidar;
                    Vector3d cam_offset = Vector3d(0,0,0);
                    
                    #pragma omp parallel for//会导致每组数据的处理时间有一些不稳定性存在，但是整体时间上是缩短的，如果不用并行，全部处理的平均时间大概在100ms
                    for (int i=0; i< pc_size; i=i+2){
                        
                        Eigen::Vector3d pix_pc;
                        Eigen::Vector3d pc_lidar;
                        Eigen::Vector3d pc_i;
                        //if(i%2 == 0) continue;//如果不筛一半，不并行的话，处理时间大概在180ms
                        pc_i<< pc_msg->points[i].x, pc_msg->points[i].y, pc_msg->points[i].z;
                        if(pc_i.x()<1 && pc_i.y()<1 && pc_i.z()<1){
                            
                            pickout_n ++;
                            continue;
                        }
                        if(pc_i.x()>x_threshold || (pc_i.y()>y_threshold || pc_i.y()<-y_threshold) || (pc_i.z()>z_threshold || pc_i.z()<-z_threshold)){
                            pickout_n_100 ++;
                            continue;
                        }
                        valid_num++;
                    
                        
                        //pc_cam = R_lidar0_lidar*pc_i + t_lidar0_lidar;
                        // pc_cam = R_cam0_lidar*pc_i + t_cam0_lidar + cam_offset;
                        pc_lidar = R_lidar0_lidar*pc_i + t_lidar0_lidar;
                        //pix_pc = T_pc_ima.block<3,3>(0,0)*pc_i + T_pc_ima.block<3,1>(0,3);
                        pix_pc = R_uv0_lidar*pc_i + t_uv0_lidar;
                        // pc_cam = T_cam_lidar*pc_i;
                        pix_pc[0] = pix_pc[0]/pix_pc[2];
                        pix_pc[1] = pix_pc[1]/pix_pc[2];
                        pointcoordinate thispoint;
                        //check pix in the camera feild of view
                        if(  pix_pc[0] >= 1  && (int)pix_pc[0] <= w_img-1 &&  pix_pc[1] >= 1  && (int)pix_pc[1] < h_img -1){
                            //num_outliner++;
                            // thispoint.x_3d = pc_i.x();
                            // thispoint.y_3d = pc_i.y();
                            // thispoint.z_3d = pc_i.z();
                            thispoint.x_3d = pc_lidar.x();
                            thispoint.y_3d = pc_lidar.y();
                            thispoint.z_3d = pc_lidar.z();
                            thispoint.u_px = pix_pc[0];
                            thispoint.v_px = pix_pc[1];
                            thispoint.t_offset = pc_msg->points[i].offset_time;

                            //thispoints[i]=thispoint;
                            //pc_array.push_back(thispoint);

                            // if  (thispoint.x_3d > point_max.x) { point_max.x = thispoint.x_3d; }
                            // if  (thispoint.y_3d > point_max.y) { point_max.y = thispoint.y_3d; }
                            // if  (thispoint.z_3d > point_max.z) { point_max.z = thispoint.z_3d; }
                            // if  (thispoint.x_3d < point_min.x) { point_min.x = thispoint.x_3d; }
                            // if  (thispoint.y_3d < point_min.y) { point_min.y = thispoint.y_3d; }
                            // if  (thispoint.z_3d < point_min.z) { point_min.z = thispoint.z_3d; }

                            // if  (thispoint.u_px > minmaxuv.umax) {minmaxuv.umax = thispoint.u_px;}
                            // if  (thispoint.u_px < minmaxuv.umin) {minmaxuv.umin = thispoint.u_px; }
                            // if  (thispoint.v_px > minmaxuv.vmax) {minmaxuv.vmax = thispoint.v_px;}
                            // if  (thispoint.v_px < minmaxuv.vmin) {minmaxuv.vmin = thispoint.v_px;}
                            // for (int u = max( (int)(pix_pc[0]- grid), 0); u < min ( (int)(pix_pc[0] + grid), w_img); u++)
                            //  for (int v = max( (int)( pix_pc[1]- grid), 0);  v < min ( (int)(pix_pc[1] + grid), h_img); v++){
                            //       pc_array_grid[v*w_img+u].push_back(thispoint);           
                            //  }
                            //save mask
                            // double Gr_x=0, Gr_y=0, Gr_z=0, Gs=0;
                            // double G_x = 0, G_y = 0, G_z=0;
                            double pu = pix_pc[0];
                            double pv = pix_pc[1];
                            double dx = abs(pc_lidar.x());
                            double dy = abs(pc_lidar.y());
                            double dz = abs(pc_lidar.z());
                            if(dy<0.01){
                            dy = 0.01;
                            //ROS_INFO_STREAM("dy = "<<dy);
                            }
                            if(dx<0.01){
                            dx = 0.01;
                            //ROS_INFO_STREAM("dy = "<<dy);
                            }
                            thispoint.Gr_x = 1.0/sqrt(dx);
                            thispoint.Gr_y = 1.0/sqrt(dy);
                            thispoint.Gr_z = 1.0/sqrt(dz);
                            thispoint.G_x = sqrt(dx);
                            thispoint.G_y = sqrt(dy);
                            thispoint.G_z = sqrt(dz);
                            // if(pc_masks[i_g].point.y_3d < 0){
                            //   G_y = -G_y;
                            // }
                            // if(pc_masks[i_g].point.z_3d < 0){
                            //   G_z = -G_z;
                            // }
                            m_thread.lock();
                            pc_manager.mask4.pc_masks_single.emplace_back(thispoint, grid);
                            pc_vector.pc_lidar_3d.emplace_back(pc_lidar);
                            pc_vector.pc_uv.emplace_back(pu, pv);
                            m_thread.unlock();    
                                    
                        }
                    }
                }
                if(yolo_depth.size()>up_size)
                {
                    yolo_depth.pop();
                }
                yolo_depth.push(pc_vector);
                pc_vector.pc_lidar_3d.clear();
                //ROS_INFO_STREAM("sizeof: "<<yolo_depth.back().pc_cam_3d.size());
                //ROS_INFO_STREAM("num_outliner = "<<num_outliner);
                ROS_DEBUG_STREAM("pickout number = "<<pickout_n);
                ROS_DEBUG_STREAM("out = "<<pickout_n_100);
                ROS_DEBUG_STREAM("valid = "<<valid_num);
                ROS_DEBUG_STREAM("mask processing: "<<start.toc()<<"ms");
                // ROS_DEBUG_STREAM("min = ("<<point_min.x<<", "<<point_min.y<<", "<<point_min.z<<")");
                // ROS_DEBUG_STREAM("max = ("<<point_max.x<<", "<<point_max.y<<", "<<point_max.z<<")");
                // ROS_INFO_STREAM("u ("<<minmaxuv.umax<<", "<<minmaxuv.umin<<")    v ("<<minmaxuv.vmax<<", "<<minmaxuv.vmin<<")");
                
                if(compare_rect){
                    
                    int pc_size = pc_manager.pc_win_buffer[cur_id]->points.size();
                    livox_ros_driver::CustomMsg::ConstPtr pc_msg = pc_manager.pc_win_buffer[cur_id];
                    //#pragma omp parallel for
                    for (int i=0; i< pc_size; i++){
                        if(!first_loop){
                            first_loop = true;
                            ROS_INFO("break!!!");
                            break;
                        }
                        Eigen::Vector3d pix_pc;
                        Eigen::Vector3d pc_i;
                        //if(i%2 == 0) continue;
                        pc_i<< pc_msg->points[i].x, pc_msg->points[i].y, pc_msg->points[i].z;
                        if(pc_i.x()<1 && pc_i.y()<1 && pc_i.z()<1){
                            
                            pickout_n ++;
                            continue;
                        }
                        if(pc_i.x()>x_threshold || (pc_i.y()>y_threshold || pc_i.y()<-y_threshold) || (pc_i.z()>z_threshold || pc_i.z()<-z_threshold)){
                            pickout_n_100 ++;
                            continue;
                        }
                        valid_num++;
                    
                        
                        //pc_cam = R_lidar0_lidar*pc_i + t_lidar0_lidar;
                        
                        pix_pc = T_pc_ima.block<3,3>(0,0)*pc_i + T_pc_ima.block<3,1>(0,3);
                        // pc_cam = T_cam_lidar*pc_i;
                        pix_pc[0] = pix_pc[0]/pix_pc[2];
                        pix_pc[1] = pix_pc[1]/pix_pc[2];
                        pointcoordinate thispoint;
                        pointcoordinate thispoint_no_rect;
                        //check pix in the camera feild of view
                        if(  pix_pc[0] >= 1  && (int)pix_pc[0] <= w_img-1 &&  pix_pc[1] >= 1  && (int)pix_pc[1] < h_img -1){
                            //num_outliner++;
                            thispoint.x_3d = pc_i.x();
                            thispoint.y_3d = pc_i.y();
                            thispoint.z_3d = pc_i.z();
                            thispoint.u_px = pix_pc[0];
                            thispoint.v_px = pix_pc[1];
                            thispoint.t_offset = pc_msg->points[i].offset_time;
                            //thispoints[i]=thispoint;
                            //pc_array.push_back(thispoint);

                            // if  (thispoint.x_3d > point_max.x) { point_max.x = thispoint.x_3d; }
                            // if  (thispoint.y_3d > point_max.y) { point_max.y = thispoint.y_3d; }
                            // if  (thispoint.z_3d > point_max.z) { point_max.z = thispoint.z_3d; }
                            // if  (thispoint.x_3d < point_min.x) { point_min.x = thispoint.x_3d; }
                            // if  (thispoint.y_3d < point_min.y) { point_min.y = thispoint.y_3d; }
                            // if  (thispoint.z_3d < point_min.z) { point_min.z = thispoint.z_3d; }

                            // if  (thispoint.u_px > minmaxuv.umax) {minmaxuv.umax = thispoint.u_px;}
                            // if  (thispoint.u_px < minmaxuv.umin) {minmaxuv.umin = thispoint.u_px; }
                            // if  (thispoint.v_px > minmaxuv.vmax) {minmaxuv.vmax = thispoint.v_px;}
                            // if  (thispoint.v_px < minmaxuv.vmin) {minmaxuv.vmin = thispoint.v_px;}
                            // for (int u = max( (int)(pix_pc[0]- grid), 0); u < min ( (int)(pix_pc[0] + grid), w_img); u++)
                            //  for (int v = max( (int)( pix_pc[1]- grid), 0);  v < min ( (int)(pix_pc[1] + grid), h_img); v++){
                            //       pc_array_grid[v*w_img+u].push_back(thispoint);           
                            //  }
                            //save mask
                            m_thread.lock();
                            pc_manager.mask_win[cur_id].pc_masks_single.emplace_back(thispoint, grid);
                            m_thread.unlock();    
                                    
                        }
                    }
                }
                    
                //cout << "array size: " << pc_array.size() << endl;
                //ROS_INFO_STREAM("point_max.x :"<<point_max.x);
                //compare_rect = true;
                int ups = upsampling_pro(point_max, point_min, minmaxuv, w_img, h_img, c_img, i_pc_count);
                //  i_pc_count ++;
                //pc_size = 0;
                

                // point_max.x = 0;
                // point_max.y = 0;
                // point_max.z = 0;
                // point_min.x = 0;
                // point_min.y = 0;
                // point_min.z = 0;
                // minmaxuv.umax = w_img/2;
                // minmaxuv.umin = 900;
                // minmaxuv.vmax = h_img/2;
                // minmaxuv.vmin = 900;
                cloud.clear();
            }catch(std::bad_alloc){
               ROS_ERROR("terminate called after throwing an instance of 'std::bad_alloc'     continue.");
               continue;
          }
     }
     
}
bool pc_first_call = true;
// void pc2Callback(const sensor_msgs::PointCloud2ConstPtr& pc_msg){
void pc2Callback(const livox_ros_driver::CustomMsg::ConstPtr &pc_msg){
    pc_time_last = pc_time_cur;
    pc_time_cur = pc_msg->header.stamp.toSec();
    //时间出现跳变，需要重置
    if((pc_time_cur-pc_time_last>1.0 || pc_time_cur < pc_time_last) && pc_time_last != 0){
        pc_manager.reset_init();
        pc_first_call = true;
    }
    m_buf.lock();
    if(pc_first_call){
        p_origin = p_drone;
        q_origin = q_drone;
        
        T_origin.block<3,3>(0,0) = q_origin.toRotationMatrix().inverse();
        T_origin.block<3,1>(0,3) = q_origin.inverse() * -p_origin;
        pc_first_call = false;
    }
    img_buffer.push(imgrgb);
    cvMat_buffer.push(img);
    pc_buffer.push(pc_msg);
    p_drone_cur = p_drone;
    q_drone_cur = q_drone;
    cur_timestamp = imgrgb.header.stamp.toNSec();//yolo的时间戳是和图像一致的
    m_buf.unlock();
    //update feature point synchronized with pc timestamp
    m_feature.lock();
    Vector3d rect_uav_pos_world = Vector3d(rect_feat_point[0], rect_feat_point[1], rect_feat_point[2]);
    rect_uav_pos_world = K_in*(R_cam_imu*(q_drone_cur.inverse()*rect_uav_pos_world - q_drone_cur.inverse()*p_drone_cur) + t_cam_imu);
    circle_center = cv::Point2d(feat_point[0],feat_point[1]);
    rect_circle_center = cv::Point2d(rect_uav_pos_world.x()/rect_uav_pos_world.z(), rect_uav_pos_world.y()/rect_uav_pos_world.z());
    u0 = feat_point[0];
    v0 = feat_point[1];
    m_feature.unlock();

    con.notify_one();
    //ROS_INFO("push pc into puffer");
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
     //ROS_DEBUG("image width: %d, height: %d, channels: %d.", w_img, h_img, c_img);
	//}
}

Vector3d calculate_yolo_depth(vector<Vector3d> &array_pc, vector<Vector2d> &uv, double u0, double v0, int grid_z);
void PC_EKF_uodate(Vector3d measurements);
void calculate_yolo_depth_init(vector<Vector3d> &array_pc, vector<Vector2d> &uv, vector<Vector4d> &x_depth, double u0, double v0, int grid_z, double perception, double depth_threshold_down, double depth_threshold_up);