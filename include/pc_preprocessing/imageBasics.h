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
#include <pcl/filters/statistical_outlier_removal.h>
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

 
const int original_freq = 100;
const int cur_freq = 100;
const int WINDOW_SIZE = 40;//0.4s*original_freq
const int Add_n = original_freq/cur_freq;
int cur_add_n = 1;
Matrix3d R_cam_lidar, K_in;
Vector3d t_cam_lidar;
Matrix3d R_Imu_Lidar, R_cam_imu, R_imu_lidar;
Vector3d t_imu_lidar;
Vector3d t_cam_imu;

//scan number depending on sacn freq
//const int N_scan = 4;

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
    Mask_pc(pointcoordinate _point){
        point = _point;
        //valid_pixles = (u_up-u_down)*(v_up-v_down); 
    }
    void SetMask(pointcoordinate _point,int _grid){
        point = _point;
        grid = _grid;
        u_down =  round(point.u_px)- grid;
        u_up = round(point.u_px) + grid;
        v_down = round( point.v_px)- grid;
        v_up = round(point.v_px) + grid;
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
        if(ima3d == NULL){
            ROS_ERROR("malloc for ima3d failed!!!! retry.");
            free(ima3d);
            malloc_ok = false;
            return;
        }
        if(ima3d_ == NULL){
            ROS_ERROR("malloc for ima3d_ failed!!!! retry.");
            free(ima3d_);
            malloc_ok = false;
            return;
        }
        SetZeros();
        malloc_ok = true;
        //ROS_INFO_STREAM("ID_MASK recalled!!!!!");
        //pc_masks = (Mask_pc*)malloc(sizeof(Mask_pc)*(1));
    };
    ~ID_MASK(){
        free(ima3d);
        free(ima3d_);
        // free(pc_masks);
        
    };
    // void ResetMaskSize(const int size);
    void SetZeros();
    // vector<Mask_pc> pc_masks[4];//多线程
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
    bool malloc_ok = true;
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
    





    // ID_MASK mask_win[WINDOW_SIZE];//mask for no motion compensation
    ID_MASK mask_no_rect;
    ID_MASK maskn;
    bool init;
    double Maxima3d[3];
    int current_id;
    double timestamp;//ns
    livox_ros_driver::CustomMsg::ConstPtr pc_win_buffer[WINDOW_SIZE];
    Eigen::Quaterniond q_wb[WINDOW_SIZE];
    Eigen::Vector3d t_wb[WINDOW_SIZE]; 
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
int upsampling_pro(pcl::PointXYZ &maxxyz, pcl::PointXYZ &minxyz, minmaxuv_ &minmaxuv, int w, int h, int c, int nof, double grid_param[]);
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
    // vector<livox_ros_driver::CustomMsg::ConstPtr> pc_lidar;
    long long timestamp;
    Eigen::Quaterniond q_wb;
    Eigen::Vector3d t_wb;

};
typedef shared_ptr< Pc_Vector > Pc_Vector_Ptr;

bool comp(livox_ros_driver::CustomPoint& a, livox_ros_driver::CustomPoint& b){
        return a.x<b.x;
}


void fromROS2pcl(pcl::PointCloud<pcl::PointXYZINormal>::Ptr &pcl, livox_ros_driver::CustomMsg::ConstPtr &msg){
    int pc_size = msg->point_num;
    pcl::PointXYZINormal point;
    for(int i = 0; i<pc_size; i++){
        point.x = msg->points[i].x;
        point.y = msg->points[i].y;
        point.z = msg->points[i].z;
        point.intensity = msg->points[i].reflectivity;
        point.curvature =
                msg->points[i].offset_time / float(1000000); //use curvature as time of each laser points
        pcl->push_back(point);
    }
}



#endif 



/******************************************************/

//int w_img = 1280, h_img = 720, c_img =3;
 //1105 dataset
int i_pc_count = 0;
int i_img_count = 0;
int sum_pc = WINDOW_SIZE;
int sum_pc_i = 0;
double sum_x = 0;

//calculate ave time
int i_n = 0;
double ave_lock = 0;
double ave_lock_i = 0;
double ave_mask_process = 0;
double ave_mask_process_i = 0;
double ave_mask_cal = 0;
double ave_mask_cal_i = 0;
double ave_total = 0;



pcl::PointCloud<pcl::PointXYZ> cloud;
bool pc_first_call = true;
queue<livox_ros_driver::CustomMsg::ConstPtr> pc_buffer;
queue<darknet_ros_msgs::BoundingBoxes::ConstPtr> box_buffer;
queue<Pc_Vector> yolo_pc_buffer;
double pc_time_cur = 0;
double pc_time_last = 0;
queue<cv::Mat> cvMat_buffer;
queue<sensor_msgs::ImageConstPtr> img_buffer;
// lock when ekf state is freshing
std::mutex m_ekf;
// control the box buffer, wake up yolo_update thread when there are box datas.
std::mutex m_yolo;
//control of yolo_depth vector
std::mutex m_visualization;
std::mutex m_yolo_match;
std::mutex m_buf, m_feature, m_thread, m_state;
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
Vector3d p_drone, p_drone_cur, p_origin, p_drone_last;
long long cur_timestamp = 0;
long long pose_timestamp = 0;
Quaterniond q_drone, q_drone_cur, q_origin, q_drone_last;
queue<Quaterniond> q_drone_buffer;
queue<Vector3d> p_drone_buffer;
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

queue<Pc_Vector_Ptr> yolo_depth;
int up_size = 15;//队列大小

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
sensor_msgs::ImageConstPtr imgrgb;
sensor_msgs::ImageConstPtr imgrgb_cur;
float sigma_feature[2]={0,0}; //the uncertainty of feature in pixel frame
int ifdetection = 0 ;
Quaterniond q_bc = Quaterniond(-0.5, 0.5, -0.5, 0.5);
Vector3d t_bc = Vector3d::Zero();

int grid = 5;
Eigen::VectorXd pc_i(4);
Eigen::MatrixXd T_pc_ima(4,4);
Eigen::MatrixXd T_cam_lidar(4,4);
pthread_t tids[4];
//points clond threshold
double x_threshold = 30;
double y_threshold = 20;
double z_threshold = 20;

//save image for train
bool image_save = false;
//visualization
bool visualization = false;
bool show_image = true;
bool compare_rect = true;
bool compare_upsampling = true;
bool first_loop = false;
bool time_compare = true;
/******************************************/

void Preprocess(){
    const int param_n = grid*grid*4;
    double grid_param[param_n];
    // double 
    for(int j = -grid; j<grid; j++){
        for(int i = -grid; i<grid; i++){
            int index = (i+grid)+(j+grid)*grid*2;

            if(i == 0 && j==0){
                grid_param[index] = 10000;
            }else if(abs(i) == 1 && abs(j) ==1){
                grid_param[index] = 100;
            }else{
                grid_param[index] = 1.0/sqrt(i*i+j*j);
            }
            // cout<<grid_param[index]<<"  ";
        }
        // cout<<endl;
    }
    while(true){
            try{
                std::unique_lock<std::mutex> lk(m_buf);
                con.wait(lk,[&]
                    {
                            return (!img_buffer.empty());
                    });
                TicToc lock_t;
                /*start: update pc_buffer, make sure pc in buffer is consecutive*/     
                std::queue<livox_ros_driver::CustomMsgConstPtr> tmp_pc_buffer;
                pc_manager.pc_win_buffer[sum_pc_i] = pc_buffer.back();
                // ROS_INFO_STREAM("mat size = "<<cvMat_buffer.size());
                // img_cur = cvMat_buffer.back();
                imgrgb_cur = img_buffer.back();
                //todo: check timestamp, pop out data
                // while(!cvMat_buffer.empty())
                    // cvMat_buffer.pop();
                while(!img_buffer.empty())
                    img_buffer.pop();
                // ROS_INFO_STREAM("mat size = "<<cvMat_buffer.size());
                /*start: check timestamp***********************************************************/
                pc_time_last = pc_time_cur;
                pc_time_cur = pc_manager.pc_win_buffer[sum_pc_i]->header.stamp.toSec();
                //时间出现跳变，需要重置
                if((pc_time_cur-pc_time_last>1.0 || pc_time_cur < pc_time_last) && pc_time_last != 0
                        || !pc_manager.maskn.malloc_ok || !pc_manager.mask_no_rect.malloc_ok){
                    pc_manager.reset_init();
                    pc_first_call = true;
                    flag_int_xkk = 0;
                    flag_int_xkk_last = 0;
                    ifdetection = 0;
                    sum_pc_i = 0;
                    last_setfre = 0;
                    while(!yolo_depth.empty())
                        yolo_depth.pop();
                    while(!pc_buffer.empty()){
                        pc_buffer.pop();
                    }
                    continue;
                }
                /*end: check timestamp***********************************************************/
                //
                int i_buffer = 0;
                while(!pc_buffer.empty()){        
                    if(i_buffer != sum_pc_i){
                        pc_manager.pc_win_buffer[i_buffer] = pc_buffer.front();
                    }
                    i_buffer ++;
                    tmp_pc_buffer.push(pc_buffer.front());
                    pc_buffer.pop();                             
                }
                while(!tmp_pc_buffer.empty()){
                    pc_buffer.push(tmp_pc_buffer.front());
                    tmp_pc_buffer.pop();
                }
                
                
                /*end: update pc_buffer, make sure pc in buffer is consecutive*/

                
               
                pc_manager.UpdateMask(sum_pc_i);
                int cur_id = pc_manager.current_id;
                pc_manager.q_wb[cur_id] = q_drone_cur;
                pc_manager.t_wb[cur_id] = p_drone_cur;
                pc_manager.timestamp = cur_timestamp;
                pc_manager.maskn.timestamp = cur_timestamp;
                /*update pose*/
                std::queue<Quaterniond> tmp_q_drone;
                std::queue<Vector3d> tmp_p_drone;
                i_buffer = 0;
                // ROS_INFO_STREAM("q_drone_buffer size = "<<q_drone_buffer.size());
                while(!q_drone_buffer.empty()){         
                    if(i_buffer != cur_id){
                        pc_manager.q_wb[i_buffer] = q_drone_buffer.front();;
                    }
                    // ROS_INFO_STREAM("ibuffer = "<<i_buffer);
                    i_buffer ++;
                    tmp_q_drone.push(q_drone_buffer.front());
                    q_drone_buffer.pop();                             
                }
                while(!tmp_q_drone.empty()){
                    q_drone_buffer.push(tmp_q_drone.front());
                    tmp_q_drone.pop();
                }
                i_buffer = 0;
                while(!p_drone_buffer.empty()){        
                    if(i_buffer != cur_id){
                        pc_manager.t_wb[i_buffer] = p_drone_buffer.front();;
                    }
                    i_buffer ++;
                    tmp_p_drone.push(p_drone_buffer.front());
                    p_drone_buffer.pop();                             
                }
                while(!tmp_p_drone.empty()){
                    p_drone_buffer.push(tmp_p_drone.front());
                    tmp_p_drone.pop();
                }
                

                
                m_buf.unlock();
                i_n ++;
                ROS_DEBUG_STREAM("lock time = "<<lock_t.toc()<<" ms"); 
                ave_lock = (ave_lock*(i_n-1) + lock_t.toc())/i_n;
                ROS_DEBUG_STREAM("ave lock time: "<<ave_lock<<"ms");     
                all_time.tic();
                // ROS_INFO("pc received");
                ROS_DEBUG_STREAM("sum_pc_i: "<<sum_pc_i);
                
                //pcl::fromROSMsg (pc_msg, cloud);
                
                sum_pc_i ++;
                if (sum_pc_i == sum_pc){
                    sum_pc_i = 0;
                    // pc_manager.set_init();
                }
                

            
                
                

                
                

               
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
                int n_skip = 3;
                Pc_Vector_Ptr pc_vector (new Pc_Vector());
                pc_vector->timestamp = imgrgb_cur->header.stamp.toNSec();//这里不能用cur_timestamp，因为这里已经解锁了，所以这个变量可能已近更新了
                pc_vector->q_wb = pc_manager.q_wb[cur_id];
                pc_vector->t_wb = pc_manager.t_wb[cur_id];
                int total_size = 0;
                for(int i = 0; i<WINDOW_SIZE; i++){
                    total_size += pc_manager.pc_win_buffer[i]->points.size();
                }
                pc_vector->pc_lidar_3d.reserve(total_size/n_skip+2);
                pc_vector->pc_uv.reserve(total_size/n_skip+2);
                pc_manager.maskn.pc_masks_single.clear();
                pc_manager.maskn.pc_masks_single.reserve(total_size/n_skip+2);
                //compare time consumer of pc_vector insert
                if(time_compare){
                    TicToc start_i;
                    //#pragma omp parallel for
                    for(int k =0; k<WINDOW_SIZE; k++){
                    
                        int pc_size = pc_manager.pc_win_buffer[k]->points.size();
                        livox_ros_driver::CustomMsg::ConstPtr pc_msg = pc_manager.pc_win_buffer[k];
                        /*********************************************************************************/
                        //pcl
                        // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
                        // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
                        // pcl::PointCloud<pcl::PointXYZINormal>::Ptr pl_surf (new pcl::PointCloud<pcl::PointXYZINormal>);
                        // pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZINormal>);
                        // pl_surf->clear();
                        // pl_surf->reserve(pc_msg->point_num+1);
                        // cloud_filtered->clear();
                        // cloud_filtered->reserve(pc_msg->point_num+1);
                        // fromROS2pcl(pl_surf, pc_msg);
                        // pcl::StatisticalOutlierRemoval<pcl::PointXYZINormal> sor;
                        // sor.setInputCloud (pl_surf);
                        // sor.setMeanK (50);
                        // sor.setStddevMulThresh (1.0);
                        // sor.filter (*cloud_filtered);
                        /************************************************************************************/


                        //sort(pc_msg.points.begin(), pc_msg.points.end(), comp);
                        //Matrix4d T_cam0_L1 =  pc_manager.mask_win[pc_manager.current_id].T_bw * pc_manager.mask_win[pc_manager.current_id].T_wb * T_imu_lidar;
                        Quaterniond q_b0_b1 = pc_manager.q_wb[cur_id].inverse()*pc_manager.q_wb[k];
                        Vector3d t_b0_b1 = pc_manager.q_wb[cur_id].inverse()*(pc_manager.t_wb[k]-pc_manager.t_wb[cur_id]);
                        // ROS_INFO_STREAM("relative p = ("<<t_b0_b1.x()<<", "<<t_b0_b1.y()<<", "<<t_b0_b1.z()<<")");
                        // ROS_INFO_STREAM("relative q = \n"<<q_b0_b1.toRotationMatrix().matrix());
                        Matrix3d R_b0_lidar = q_b0_b1*R_imu_lidar;
                        // Vector3d t_b0_lidar = pc_manager.mask_win[cur_id].q_wb.inverse()*(pc_manager.mask_win[k].q_wb*t_imu_lidar) + t_b0_b1;
                        Vector3d t_b0_lidar = q_b0_b1*t_imu_lidar + t_b0_b1;

                        Matrix3d R_lidar0_lidar = R_imu_lidar.inverse()*R_b0_lidar;
                        Vector3d t_lidar0_lidar = R_imu_lidar.inverse()*(t_b0_lidar)-R_imu_lidar.inverse()*t_imu_lidar;
                        Matrix3d R_cam0_lidar = R_cam_imu*R_b0_lidar;
                        Vector3d t_cam0_lidar = (R_cam_imu*(t_b0_lidar)+t_cam_imu);
                        Matrix3d R_uv0_lidar = K_in*R_cam0_lidar;
                        Vector3d t_uv0_lidar = K_in*t_cam0_lidar;
                        Vector3d cam_offset = Vector3d(0,0,0);
                        /*********************************************************************/
                        // pointcoordinate zerothispoint
                        // for(int i = 0; i<pc_size; i=i+2){
                        //     // m_thread.lock();
                        //     pc_manager.maskn.pc_masks_single.emplace_back(zerothispoint, grid);
                        //     pc_vector.pc_lidar_3d.emplace_back(0,0,0);
                        //     pc_vector.pc_uv.emplace_back(0, 0);
                        //     // pc_vector.pc_3d_uv.emplace_back(pc_lidar.x(), pc_lidar.y(), pc_lidar.z(), pu, pv);
                        //     // m_thread.unlock();    
                        // }
                        /***************************************************************/



                        #pragma omp parallel for//会导致每组数据的处理时间有一些不稳定性存在，但是整体时间上是缩短的，如果不用并行，全部处理的平均时间大概在100ms
                        for (int i=0; i< pc_size; i=i+n_skip){
                            
                            Eigen::Vector3d pix_pc;
                            Eigen::Vector3d pc_lidar;
                            Eigen::Vector3d pc_i;
                            //if(i%2 == 0) continue;//如果不筛一半，不并行的话，处理时间大概在180ms
                            // pc_i<< cloud_filtered-> , pc_msg->points[i].y, pc_msg->points[i].z;
                            pc_i<< pc_msg->points[i].x, pc_msg->points[i].y, pc_msg->points[i].z;
                            // ROS_INFO_STREAM("time: "<<pc_msg->points[i].offset_time);
                            // ROS_INFO_STREAM("pc_i.x = "<<pc_i.x());
                            //如果没有，会导致上采样后的图像出现一些黑点
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
                            // pc_lidar = pc_i;
                            //pix_pc = T_pc_ima.block<3,3>(0,0)*pc_i + T_pc_ima.block<3,1>(0,3);
                            // pix_pc = T_pc_ima.block<3,3>(0,0)*pc_i + T_pc_ima.block<3,1>(0,3);
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

                                if  (thispoint.u_px > minmaxuv.umax) {minmaxuv.umax = thispoint.u_px;}
                                if  (thispoint.u_px < minmaxuv.umin) {minmaxuv.umin = thispoint.u_px; }
                                if  (thispoint.v_px > minmaxuv.vmax) {minmaxuv.vmax = thispoint.v_px;}
                                if  (thispoint.v_px < minmaxuv.vmin) {minmaxuv.vmin = thispoint.v_px;}
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
                                // if(dy<0.0001){
                                // dy = 0.0001;
                                // //ROS_INFO_STREAM("dy = "<<dy);
                                // }
                                // if(dx<0.0001){
                                // dx = 0.0001;
                                // //ROS_INFO_STREAM("dy = "<<dy);
                                // }
                                // if(dz<0.0001){
                                // dz = 0.0001;
                                // //ROS_INFO_STREAM("dy = "<<dy);
                                // }
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
                                pc_manager.maskn.pc_masks_single.emplace_back(thispoint);
                                // pc_vector.pc_lidar_3d.emplace_back(pc_lidar);
                                // pc_vector.pc_uv.emplace_back(pu, pv);
                                // pc_vector.pc_3d_uv.emplace_back(pc_lidar.x(), pc_lidar.y(), pc_lidar.z(), pu, pv);
                                m_thread.unlock();    
                                        
                            }
                        }
                    }
                    ROS_DEBUG_STREAM("mask processing_i: "<<start_i.toc()<<"ms");
                    ave_mask_process_i = (ave_mask_process_i*(i_n-1) + start_i.toc())/i_n;
                    ROS_DEBUG_STREAM("ave mask processing_i: "<<ave_mask_process_i<<"ms");
                    pc_manager.maskn.pc_masks_single.clear();
                }
                TicToc start;
                #pragma omp parallel for
                for(int k =0; k<WINDOW_SIZE; k++){
                
                    int pc_size = pc_manager.pc_win_buffer[k]->points.size();
                    livox_ros_driver::CustomMsg::ConstPtr pc_msg = pc_manager.pc_win_buffer[k];
                    /*********************************************************************************/
                    //pcl
                    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
                    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
                    // pcl::PointCloud<pcl::PointXYZINormal>::Ptr pl_surf (new pcl::PointCloud<pcl::PointXYZINormal>);
                    // pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZINormal>);
                    // pl_surf->clear();
                    // pl_surf->reserve(pc_msg->point_num+1);
                    // cloud_filtered->clear();
                    // cloud_filtered->reserve(pc_msg->point_num+1);
                    // fromROS2pcl(pl_surf, pc_msg);
                    // pcl::StatisticalOutlierRemoval<pcl::PointXYZINormal> sor;
                    // sor.setInputCloud (pl_surf);
                    // sor.setMeanK (50);
                    // sor.setStddevMulThresh (1.0);
                    // sor.filter (*cloud_filtered);
                    /************************************************************************************/


                    //sort(pc_msg.points.begin(), pc_msg.points.end(), comp);
                    //Matrix4d T_cam0_L1 =  pc_manager.mask_win[pc_manager.current_id].T_bw * pc_manager.mask_win[pc_manager.current_id].T_wb * T_imu_lidar;
                    Quaterniond q_b0_b1 = pc_manager.q_wb[cur_id].inverse()*pc_manager.q_wb[k];
                    Vector3d t_b0_b1 = pc_manager.q_wb[cur_id].inverse()*(pc_manager.t_wb[k]-pc_manager.t_wb[cur_id]);
                    // ROS_INFO_STREAM("relative p = ("<<t_b0_b1.x()<<", "<<t_b0_b1.y()<<", "<<t_b0_b1.z()<<")");
                    // ROS_INFO_STREAM("relative q = \n"<<q_b0_b1.toRotationMatrix().matrix());
                    Matrix3d R_b0_lidar = q_b0_b1*R_imu_lidar;
                    // Vector3d t_b0_lidar = pc_manager.mask_win[cur_id].q_wb.inverse()*(pc_manager.mask_win[k].q_wb*t_imu_lidar) + t_b0_b1;
                    Vector3d t_b0_lidar = q_b0_b1*t_imu_lidar + t_b0_b1;

                    Matrix3d R_lidar0_lidar = R_imu_lidar.inverse()*R_b0_lidar;
                    Vector3d t_lidar0_lidar = R_imu_lidar.inverse()*(t_b0_lidar)-R_imu_lidar.inverse()*t_imu_lidar;
                    Matrix3d R_cam0_lidar = R_cam_imu*R_b0_lidar;
                    Vector3d t_cam0_lidar = (R_cam_imu*(t_b0_lidar)+t_cam_imu);
                    Matrix3d R_uv0_lidar = K_in*R_cam0_lidar;
                    Vector3d t_uv0_lidar = K_in*t_cam0_lidar;
                    Vector3d cam_offset = Vector3d(0,0,0);
                    /*********************************************************************/
                    // pointcoordinate zerothispoint
                    // for(int i = 0; i<pc_size; i=i+2){
                    //     // m_thread.lock();
                    //     pc_manager.maskn.pc_masks_single.emplace_back(zerothispoint, grid);
                    //     pc_vector.pc_lidar_3d.emplace_back(0,0,0);
                    //     pc_vector.pc_uv.emplace_back(0, 0);
                    //     // pc_vector.pc_3d_uv.emplace_back(pc_lidar.x(), pc_lidar.y(), pc_lidar.z(), pu, pv);
                    //     // m_thread.unlock();    
                    // }
                    /***************************************************************/



                    // #pragma omp parallel for//会导致每组数据的处理时间有一些不稳定性存在，但是整体时间上是缩短的，如果不用并行，全部处理的平均时间大概在100ms
                    for (int i=0; i< pc_size; i=i+n_skip){
                        
                        Eigen::Vector3d pix_pc;
                        Eigen::Vector3d pc_lidar;
                        Eigen::Vector3d pc_i;
                        //if(i%2 == 0) continue;//如果不筛一半，不并行的话，处理时间大概在180ms
                        // pc_i<< cloud_filtered-> , pc_msg->points[i].y, pc_msg->points[i].z;
                        pc_i<< pc_msg->points[i].x, pc_msg->points[i].y, pc_msg->points[i].z;
                        // ROS_INFO_STREAM("time: "<<pc_msg->points[i].offset_time);
                        // ROS_INFO_STREAM("pc_i.x = "<<pc_i.x());
                        //如果没有，会导致上采样后的图像出现一些黑点
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
                        // pc_lidar = pc_i;
                        //pix_pc = T_pc_ima.block<3,3>(0,0)*pc_i + T_pc_ima.block<3,1>(0,3);
                        // pix_pc = T_pc_ima.block<3,3>(0,0)*pc_i + T_pc_ima.block<3,1>(0,3);
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

                            if  (thispoint.u_px > minmaxuv.umax) {minmaxuv.umax = thispoint.u_px;}
                            if  (thispoint.u_px < minmaxuv.umin) {minmaxuv.umin = thispoint.u_px; }
                            if  (thispoint.v_px > minmaxuv.vmax) {minmaxuv.vmax = thispoint.v_px;}
                            if  (thispoint.v_px < minmaxuv.vmin) {minmaxuv.vmin = thispoint.v_px;}
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
                            // if(dy<0.0001){
                            // dy = 0.0001;
                            // //ROS_INFO_STREAM("dy = "<<dy);
                            // }
                            // if(dx<0.0001){
                            // dx = 0.0001;
                            // //ROS_INFO_STREAM("dy = "<<dy);
                            // }
                            // if(dz<0.0001){
                            // dz = 0.0001;
                            // //ROS_INFO_STREAM("dy = "<<dy);
                            // }
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
                            pc_manager.maskn.pc_masks_single.emplace_back(thispoint, grid);
                            pc_vector->pc_lidar_3d.emplace_back(pc_lidar);
                            pc_vector->pc_uv.emplace_back(pu, pv);
                            // pc_vector.pc_3d_uv.emplace_back(pc_lidar.x(), pc_lidar.y(), pc_lidar.z(), pu, pv);
                            m_thread.unlock();    
                                    
                        }
                    }
                }
                m_yolo_match.lock();
                if(yolo_depth.size()>up_size)
                {
                    yolo_depth.pop();
                }
                yolo_depth.push(pc_vector);
                m_yolo_match.unlock();
                // pc_vector->pc_lidar_3d.clear();

                //ROS_INFO_STREAM("sizeof: "<<yolo_depth.back().pc_cam_3d.size());
                //ROS_INFO_STREAM("num_outliner = "<<num_outliner);
                // ROS_DEBUG_STREAM("pickout number = "<<pickout_n);
                // ROS_DEBUG_STREAM("out = "<<pickout_n_100);
                ROS_DEBUG_STREAM("valid = "<<valid_num);
                ROS_DEBUG_STREAM("mask processing: "<<start.toc()<<"ms");
                ave_mask_process = (ave_mask_process*(i_n-1) + start.toc())/i_n;
                ROS_DEBUG_STREAM("ave mask processing: "<<ave_mask_process<<"ms");
                // ROS_DEBUG_STREAM("min = ("<<point_min.x<<", "<<point_min.y<<", "<<point_min.z<<")");
                // ROS_DEBUG_STREAM("max = ("<<point_max.x<<", "<<point_max.y<<", "<<point_max.z<<")");
                // ROS_INFO_STREAM("u ("<<minmaxuv.umax<<", "<<minmaxuv.umin<<")    v ("<<minmaxuv.vmax<<", "<<minmaxuv.vmin<<")");
                
                if(compare_rect){
                    
                    #pragma omp parallel for
                    for(int k =0; k<WINDOW_SIZE; k++){
                    
                        int pc_size = pc_manager.pc_win_buffer[k]->points.size();
                        livox_ros_driver::CustomMsg::ConstPtr pc_msg = pc_manager.pc_win_buffer[k];
                        
                        Quaterniond q_b0_b1 = pc_manager.q_wb[cur_id].inverse()*pc_manager.q_wb[k];
                        Vector3d t_b0_b1 = pc_manager.q_wb[cur_id].inverse()*(pc_manager.t_wb[k]-pc_manager.t_wb[cur_id]);
                        // ROS_INFO_STREAM("relative p = ("<<t_b0_b1.x()<<", "<<t_b0_b1.y()<<", "<<t_b0_b1.z()<<")");
                        // ROS_INFO_STREAM("relative q = \n"<<q_b0_b1.toRotationMatrix().matrix());
                        Matrix3d R_b0_lidar = q_b0_b1*R_imu_lidar;
                        // Vector3d t_b0_lidar = pc_manager.mask_win[cur_id].q_wb.inverse()*(pc_manager.mask_win[k].q_wb*t_imu_lidar) + t_b0_b1;
                        Vector3d t_b0_lidar = q_b0_b1*t_imu_lidar + t_b0_b1;

                        Matrix3d R_lidar0_lidar = R_imu_lidar.inverse()*R_b0_lidar;
                        Vector3d t_lidar0_lidar = R_imu_lidar.inverse()*(t_b0_lidar)-R_imu_lidar.inverse()*t_imu_lidar;
                        Matrix3d R_cam0_lidar = R_cam_imu*R_b0_lidar;
                        Vector3d t_cam0_lidar = (R_cam_imu*(t_b0_lidar)+t_cam_imu);
                        Matrix3d R_uv0_lidar = K_in*R_cam0_lidar;
                        Vector3d t_uv0_lidar = K_in*t_cam0_lidar;
                        Vector3d cam_offset = Vector3d(0,0,0);




                        // #pragma omp parallel for//会导致每组数据的处理时间有一些不稳定性存在，但是整体时间上是缩短的，如果不用并行，全部处理的平均时间大概在100ms
                        for (int i=0; i< pc_size; i=i+n_skip){
                            
                            Eigen::Vector3d pix_pc;
                            Eigen::Vector3d pc_lidar;
                            Eigen::Vector3d pc_i;
                            //if(i%2 == 0) continue;//如果不筛一半，不并行的话，处理时间大概在180ms
                            // pc_i<< cloud_filtered-> , pc_msg->points[i].y, pc_msg->points[i].z;
                            pc_i<< pc_msg->points[i].x, pc_msg->points[i].y, pc_msg->points[i].z;
                            // ROS_INFO_STREAM("time: "<<pc_msg->points[i].offset_time);
                            // ROS_INFO_STREAM("pc_i.x = "<<pc_i.x());
                            //如果没有，会导致上采样后的图像出现一些黑点
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
                            // pc_lidar = R_lidar0_lidar*pc_i + t_lidar0_lidar;
                            pc_lidar = pc_i;
                            //pix_pc = T_pc_ima.block<3,3>(0,0)*pc_i + T_pc_ima.block<3,1>(0,3);
                            pix_pc = T_pc_ima.block<3,3>(0,0)*pc_i + T_pc_ima.block<3,1>(0,3);
                            // pix_pc = R_uv0_lidar*pc_i + t_uv0_lidar;
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

                                
                                double pu = pix_pc[0];
                                double pv = pix_pc[1];
                                double dx = abs(pc_lidar.x());
                                double dy = abs(pc_lidar.y());
                                double dz = abs(pc_lidar.z());
                                
                                thispoint.Gr_x = 1.0/sqrt(dx);
                                thispoint.Gr_y = 1.0/sqrt(dy);
                                thispoint.Gr_z = 1.0/sqrt(dz);
                                thispoint.G_x = sqrt(dx);
                                thispoint.G_y = sqrt(dy);
                                thispoint.G_z = sqrt(dz);
                               
                                m_thread.lock();
                                pc_manager.mask_no_rect.pc_masks_single.emplace_back(thispoint, grid);
                               
                                m_thread.unlock();    
                                        
                            }
                        }
                    }
                }
                // ROS_DEBUG_STREAM("no_rect mask size = "<<pc_manager.mask_win[cur_id].pc_masks_single.size());
                    
                //cout << "array size: " << pc_array.size() << endl;
                //ROS_INFO_STREAM("point_max.x :"<<point_max.x);
                //compare_rect = true;
                int ups = upsampling_pro(point_max, point_min, minmaxuv, w_img, h_img, c_img, i_pc_count, grid_param);
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
                // ROS_INFO_STREAM("process thread t = "<<all_time.toc()<<" ms");
            }catch(std::bad_alloc){
               ROS_ERROR("terminate called after throwing an instance of 'std::bad_alloc'     in pc_processing part");

               continue;
          }
     }
     
}

// void pc2Callback(const sensor_msgs::PointCloud2ConstPtr& pc_msg){
long long time_pc_cur = 0;
long long time_pc_last = 0;
void pc2Callback(const livox_ros_driver::CustomMsg::ConstPtr &pc_msg){
    TicToc pc_callback_t;
    m_state.lock();
    p_drone_last = p_drone_cur;
    q_drone_last = q_drone_cur;
    p_drone_cur = p_drone;
    q_drone_cur = q_drone;
    // p_drone_cur = Eigen::Vector3d(0, 0, 0);
    // q_drone_cur = Eigen::Quaterniond::Identity();
    m_state.unlock();



    m_buf.lock();
    // if(cur_add_n = Add_n){
    if(pc_first_call){
        // p_origin = p_drone;
        // q_origin = q_drone;
        
        // T_origin.block<3,3>(0,0) = q_origin.toRotationMatrix().inverse();
        // T_origin.block<3,1>(0,3) = q_origin.inverse() * -p_origin;
        pc_first_call = false;
    }
    // //check timestamp
    // time_pc_last = time_pc_cur;
    // time_pc_cur = pc_msg->header.stamp.toNSec(); 
    // if(time_pc_cur<=time_pc_last){
    //     ROS_ERROR_STREAM("PC TIMESTAMP ERRO!  detail: "<<time_pc_cur<<" <= "<<time_pc_last);
    //     return;
    // }
    if(cur_add_n = Add_n){
        // p_drone_buffer.push(p_drone_cur);
        p_drone_buffer.push(p_drone_last);
        while(p_drone_buffer.size()>WINDOW_SIZE){
            p_drone_buffer.pop();
        }
        // q_drone_buffer.push(q_drone_cur);
        q_drone_buffer.push(q_drone_last);
        while(q_drone_buffer.size()>WINDOW_SIZE){
            q_drone_buffer.pop();
        }
        
        // ROS_INFO_STREAM("size of cvMat = "<<cvMat_buffer.size())
        
        //reset cur_add_n
        cur_add_n = 0;
    }
    cur_add_n++;
    // img_buffer.push(imgrgb);
    // cvMat_buffer.push(img);
    pc_buffer.push(pc_msg);
    
    /*keep pc_buffer.size == WINDOW_SIZE*/
    while(pc_buffer.size()>WINDOW_SIZE){
        pc_buffer.pop();
    }
    // int pc_buffer_size = pc_buffer.size();
    // cur_timestamp = imgrgb.header.stamp.toNSec();//yolo的时间戳是和图像一致的
    m_buf.unlock();

    //update feature point synchronized with pc timestamp
    // m_feature.lock();
    // Vector3d rect_uav_pos_world = Vector3d(rect_feat_point[0], rect_feat_point[1], rect_feat_point[2]);
    // rect_uav_pos_world = K_in*(R_cam_imu*(q_drone_cur.inverse()*rect_uav_pos_world - q_drone_cur.inverse()*p_drone_cur) + t_cam_imu);
    // circle_center = cv::Point2d(feat_point[0],feat_point[1]);
    // rect_circle_center = cv::Point2d(rect_uav_pos_world.x()/rect_uav_pos_world.z(), rect_uav_pos_world.y()/rect_uav_pos_world.z());
    // u0 = feat_point[0];
    // v0 = feat_point[1];
    // m_feature.unlock();
    // if(pc_buffer_size == WINDOW_SIZE){
    //     pc_manager.set_init();
    //     con.notify_one();
    // }
        
    //ROS_INFO("push pc into puffer");
    // ROS_INFO_STREAM("PC_callback_time = "<<pc_callback_t.toc()<<" ms");
}
long long time_img_cur = 0;
long long time_img_last = 0;
void imgCallback(const  sensor_msgs::ImageConstPtr& msg)
{
	// cv_bridge::CvImagePtr cv_ptr;
	//cout << "encoding: " << msg->encoding << endl; //bgr8a
	//if(msg->encoding == "mono8" || msg->encoding == "bgr8" || msg->encoding == "rgb8"){
    //cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    // cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    //cv::Mat img  = cv_ptr -> image;
    // img  = cv_ptr -> image;
    // w_img = img.cols;
	// h_img = img.rows;
	// c_img = img.channels();
    imgrgb = msg;
    // time_img_last = time_img_cur;
    // time_img_cur = msg->header.stamp.toNSec();
    // if(time_img_cur <= time_img_last){
    //     ROS_ERROR("IMAGE TIMESTAMP ERROR!");
    //     return;
    // }
/**********************************************/


    m_buf.lock();
    img_buffer.push(imgrgb); 
    int pc_buffer_size = pc_buffer.size();
    cur_timestamp = imgrgb->header.stamp.toNSec();//yolo的时间戳是和图像一致的
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
    if(pc_buffer_size == WINDOW_SIZE){
        pc_manager.set_init();
        con.notify_one();
    }
        
    //ROS_INFO("push pc into puffer");
    // ROS_INFO_STREAM("PC_callback_time = "<<pc_callback_t.toc()<<" ms");
	

}

Vector3d calculate_yolo_depth(vector<Vector3d> &array_pc, vector<Vector2d> &uv, double u0, double v0, int grid_z);
void PC_EKF_uodate(Vector3d measurements);
void calculate_yolo_depth_init(vector<Vector3d> &array_pc, vector<Vector2d> &uv, vector<Vector4d> &x_depth, double u0, double v0, int grid_z, double perception, double depth_threshold_down, double depth_threshold_up);
void calculate_yolo_depth_2(vector<Vector3d> &array_pc, vector<Vector2d> &uv, vector<vector<Vector4d>> &grid_depth, double yolo_u_min, double yolo_u_max, double yolo_v_min, double yolo_v_max, 
                                        int grid_z, double perception, double depth_threshold_down, double depth_threshold_up);
