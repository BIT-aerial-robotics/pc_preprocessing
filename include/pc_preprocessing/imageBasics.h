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


using namespace std;
using namespace Eigen;

int w_img = 672, h_img = 376, c_img =3;
const int WINDOW_SIZE = 4;//0.4s
Matrix3d R_cam_lidar, K_in;
Vector3d t_cam_lidar;

//controll image saving frequency
double cur_setfre = 0;
double last_setfre = 0;


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
        u_down = max( (int)(point.u_px- grid), 0);
        u_up = min( (int)(point.u_px + grid), w_img);
        v_down = max( (int)( point.v_px- grid), 0);
        v_up = min ( (int)(point.v_px + grid), h_img);
        //valid_pixles = (u_up-u_down)*(v_up-v_down); 
    }
    int u_up, u_down, v_up, v_down;//[down,up)
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

double pc_time_cur = 0;
double pc_time_last = 0;
queue<cv::Mat> cvMat_buffer;
queue<sensor_msgs::Image> img_buffer;
std::mutex m_buf, m_feature, m_thread;
std::condition_variable con;

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
Quaterniond q_drone, q_drone_cur, q_origin;
//T_b0_w
Matrix4d T_origin = Matrix4d::Identity();
sensor_msgs::Image img1;
double feat_point[2] = {300,150}; //the feature position in the pixel frame, detected by the detector
vector<pointcoordinate> pc_array_feature; //put the feature points in the array
Eigen::Vector3d x_k_k;
Eigen::Matrix3d P_k_k = Eigen::Matrix3d::Identity();
int flag_int_xkk = 0;
int flag_int_xkk_last = 0;
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


bool image_save = false;
bool compare_rect = false;
bool first_loop = false;
/******************************************/

void Preprocess(){
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

    Matrix3d R_Imu_Lidar, R_cam_imu, R_imu_lidar;
    Vector3d t_imu_lidar;
    Vector3d t_Imu_Lidar = Vector3d(-0.352103, -0.057517, -0.087161);
    Vector3d t_cam_imu = Vector3d(0.05813448, -0.10000966, -0.09331424);

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
    while(true){

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
            m_buf.unlock();
            
            
            all_time.tic();
            ROS_INFO("pc received");
            ROS_DEBUG_STREAM("sum_pc_i: "<<sum_pc_i);
            pc_manager.UpdateMask(sum_pc_i);
            int cur_id = pc_manager.current_id;
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
            pc_manager.mask_win[cur_id].q_wb = q_drone_cur;
            pc_manager.mask_win[cur_id].t_wb = p_drone_cur;
            //  pc_manager.mask_win[pc_manager.current_id].ResetMaskSize(cloud.points.size());
            if(!pc_manager.init) continue;

            //int grid = 5;
           
            
            

            
            int num_outliner = 0;

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
            

            //omp_set_num_threads(4);
            

            
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
                
                // Matrix3d R_lidar0_lidar = R_imu_lidar.inverse()*R_b0_lidar;
                // Vector3d t_lidar0_lidar = R_imu_lidar.inverse()*(t_b0_lidar)-R_imu_lidar.inverse()*t_imu_lidar;
                Matrix3d R_cam0_lidar = R_cam_imu*R_b0_lidar;
                Vector3d t_cam0_lidar = (R_cam_imu*(t_b0_lidar)+t_cam_imu);
                Matrix3d R_uv0_lidar = K_in*R_cam0_lidar;
                Vector3d t_uv0_lidar = K_in*t_cam0_lidar;
                #pragma omp parallel for
                for (int i=0; i< pc_size; i++){
                    
                    Eigen::Vector3d pix_pc;
                    Eigen::Vector3d pc_cam;
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
                    pc_cam = R_cam0_lidar*pc_i + t_cam0_lidar;
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
                        thispoint.x_3d = pc_cam.x();
                        thispoint.y_3d = pc_cam.y();
                        thispoint.z_3d = pc_cam.z();
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
                        pc_manager.mask4.pc_masks_single.emplace_back(thispoint, grid);
                        m_thread.unlock();    
                                
                    }
                }
            }
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
            compare_rect = true;
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
    m_buf.unlock();
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