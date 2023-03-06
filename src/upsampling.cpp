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

#include "imageBasics.h"

using namespace std;


int x_depth_png = 0;
int search_box_yolo = 5;
double threshold_depth = 1.5;
/**
 * @brief calculate uav's 3D position in camera frame
 * 
 * @param array_pc all pc in camera frame
 * @param uv 2d pixle coordinates vector
 * @param u0 pixle coordinates of feature
 * @param v0
 * @param grid_z search radius
 * @return Vector3d: uav's 3D position in camera frame
 */

void calculate_yolo_depth_init(vector<Vector3d> &array_pc, vector<Vector2d> &uv, vector<Vector4d> &x_depth, double u0, double v0, int grid_z, double perception, double depth_threshold_down, double depth_threshold_up){
  // int grid_z = 5; 
  try{
    int grid_ave = 1;
    if(grid_z !=5){
      search_box_yolo = grid_z;
    }
    // vector<Vector3d> pc_array_yolo;
    x_depth.clear();
    //depth picture
    int lenth_of_vector = (depth_threshold_up-depth_threshold_down)/perception + 5;
    x_depth.reserve(lenth_of_vector+1);
    // ROS_INFO_STREAM("lenth_of_vector = "<<lenth_of_vector);
    
    // ROS_INFO_STREAM("x_depth capacity = "<<x_depth.capacity());
    // vector<int> x_label;
    if(x_depth.capacity() == 0){
      ROS_ERROR("unknow error, vector capacity become zero!!!!!!!!");
      return;
    }
    for(int i = 0;i<lenth_of_vector;i++){
        x_depth.emplace_back(0,0,0,0);
        // x_label.emplace_back(i);
    }
    
    

    int array_size = array_pc.size();
    //ROS_INFO_STREAM("u,v = "<<u0<<", "<<v0);
    int len_in = grid_z+grid_ave;
    int len_in2 = len_in*len_in;
    


    for(int i = 0; i < array_size; i++){
      int eu = uv[i].x() - u0;
      int ev = uv[i].y() - v0;
      int d2 = eu*eu + ev*ev;
      //ROS_INFO_STREAM("d2="<<d2);
      if(d2<=len_in2){
        //pc_array_feature.push_back(pc_masks[i].point);
        Vector3d in_box =Vector3d(array_pc[i].x(), array_pc[i].y(), array_pc[i].z()); 
        // pc_array_yolo.emplace_back(in_box);
        if(array_pc[i].x()<depth_threshold_up && array_pc[i].x()>depth_threshold_down)//lidar frame
        {
          int index = int((array_pc[i].x() - depth_threshold_down)/perception)+1;
          if(index < x_depth.size()){
            x_depth[index].x() += in_box.x();
            x_depth[index].y() += in_box.y();
            x_depth[index].z() += in_box.z();
            x_depth[index].w() ++ ;
          }else{
              ROS_ERROR("index error! there is a bug in calculate_yolo_depth_init().");
              return;

          }
        }
          

      }
    }
    
    //visualization
    for(int i=0;i<lenth_of_vector;i++){
      if(x_depth[i].w()<10) {
        x_depth[i].w() = 0;
        x_depth[i].x() = 0;
        x_depth[i].y() = 0;
        x_depth[i].z() = 0;
        // cout <<" "<<" ";
      }else{
        // cout <<x_depth[i].w()<<" ";
      }
    }
    // cout<<endl;
    return;
  }
  catch(std::bad_alloc){
    ROS_ERROR("terminate called after throwing an instance of 'std::bad_alloc'     in calculate_yolo_depth_init.");
    return;
  }
}

/**
 * @brief calculate uav's 3D position in camera frame
 * 
 * @param array_pc all pc in camera frame
 * @param uv 2d pixle coordinates vector
 * @param u0 pixle coordinates of feature
 * @param v0
 * @param grid_z search radius
 * @return Vector3d: uav's 3D position in camera frame
 */

/// @brief 
/// @param array_pc 
/// @param uv 
/// @param grid_depth 
/// @param yolo_u_min 
/// @param yolo_u_max 
/// @param yolo_v_min 
/// @param yolo_v_max 
/// @param grid_z 
/// @param perception 
/// @param depth_threshold_down 
/// @param depth_threshold_up 
void calculate_yolo_depth_2(vector<Vector3d> &array_pc, vector<Vector2d> &uv, vector<vector<Vector4d>> &grid_depth, double yolo_u_min, double yolo_u_max, double yolo_v_min, double yolo_v_max, 
                                        int grid_z, double perception, double depth_threshold_down, double depth_threshold_up){
  try{
    double uij,vij;
    //set zero
    int lenth_of_vector = (depth_threshold_up-depth_threshold_down)/perception+2;
    vector<Vector4d> x_depth;      
    x_depth.reserve(lenth_of_vector+1);
    if(x_depth.capacity() == 0){
      ROS_ERROR("unknow error, vector capacity become zero!!!!!!!!");
      return;
    }
    for(int i = 0;i<lenth_of_vector;i++){
        x_depth.emplace_back(0,0,0,0);
    }
    
    for(int i=yolo_u_min; i<yolo_u_max; i = i+grid_z*2){
      for(int j=yolo_v_min; j<yolo_v_max; j = j+grid_z*2){
        // uij = i + grid_z;
        // vij = j + grid_z;
        grid_depth.emplace_back(x_depth); 
      }
    }

     
    int grid_ave = 1;
    int array_size = array_pc.size();
    //ROS_INFO_STREAM("u,v = "<<u0<<", "<<v0);
    int len_in = grid_z+grid_ave;
    int len_in2 = len_in*len_in;
    #pragma omp parallel for
    for(int pc_id = 0; pc_id < array_size; pc_id++){
      //pixle coordinate
      int eu = uv[pc_id].x();
      int ev = uv[pc_id].y();
      if(eu>yolo_u_min && eu<yolo_u_max && ev>yolo_v_min && ev<yolo_v_max){
        int vector_id = 0;
        for(int i=yolo_u_min; i<yolo_u_max; i = i+grid_z*2){
          for(int j=yolo_v_min; j<yolo_v_max; j = j+grid_z*2){
            if(eu>=i && eu<i+grid_z*2 && ev>=j && ev>j+grid_z*2){
              //ROS_INFO_STREAM("d2="<<d2);
              //pc_array_feature.push_back(pc_masks[i].point);
              Vector3d in_box =Vector3d(array_pc[pc_id].x(), array_pc[pc_id].y(), array_pc[pc_id].z()); 
              // pc_array_yolo.emplace_back(in_box);
              if(array_pc[pc_id].x()<depth_threshold_up && array_pc[pc_id].x()>depth_threshold_down)//lidar frame
              {
                int index = int((array_pc[pc_id].x() - depth_threshold_down)/perception)+1;
                if(index < lenth_of_vector){
                  vector<Vector4d> &x_depth_i = grid_depth[vector_id];
                  x_depth_i[index].x() += in_box.x();
                  x_depth_i[index].y() += in_box.y();
                  x_depth_i[index].z() += in_box.z();
                  x_depth_i[index].w() ++ ;
                }else{
                    ROS_ERROR("index error! there is a bug in calculate_yolo_depth_2.");
                    continue;
                }
              }
    
            }
            vector_id++;

          }
        }  
      }
    }
    
    //visualization
    for(int i=0; i<grid_depth.size(); i++){
      vector<Vector4d> &x_depth_i = grid_depth[i];
      for(int j=0; j<x_depth_i.size(); j++){
      
        if(x_depth_i[j].w()<grid_z*10/3) {
          x_depth_i[j].w() = 0;
          x_depth_i[j].x() = 0;
          x_depth_i[j].y() = 0;
          x_depth_i[j].z() = 0;
          // cout <<" "<<" ";
        }else{
          // cout <<x_depth[i].w()<<" ";
        }
      }
      // cout<<"========================================================================"<<endl;
    }
    // for(int i=0;i<lenth_of_vector;i++){
    //   if(x_depth[i].w()<10) {
    //     x_depth[i].w() = 0;
    //     x_depth[i].x() = 0;
    //     x_depth[i].y() = 0;
    //     x_depth[i].z() = 0;
    //     // cout <<" "<<" ";
    //   }else{
    //     // cout <<x_depth[i].w()<<" ";
    //   }
    // }
    // cout<<endl;
    return;
  }
  catch(std::bad_alloc){
    ROS_ERROR("terminate called after throwing an instance of 'std::bad_alloc'     in calculate_yolo_depth_init.");
    return;
  }
}






/**
 * @brief calculate uav's 3D position in camera frame
 * 
 * @param array_pc all pc in camera frame
 * @param uv 2d pixle coordinates vector
 * @param u0 pixle coordinates of feature
 * @param v0
 * @param grid_z search radius
 * @return Vector3d: uav's 3D position in camera frame
 */

Vector3d calculate_yolo_depth(vector<Vector3d> &array_pc, vector<Vector2d> &uv, double u0, double v0, int grid_z = 5){
  // int grid_z = 5; 
  int grid_ave = 1;
  if(grid_z !=5){
    search_box_yolo = grid_z;
  }
  vector<Vector3d> pc_array_yolo;

  //depth picture
  vector<int> x_depth;
  vector<int> x_label;
  for(int i = 0;i<100;i++){
    x_depth.emplace_back(0);
    x_label.emplace_back(i);
  }

  int array_size = array_pc.size();
  //ROS_INFO_STREAM("u,v = "<<u0<<", "<<v0);
  int len_in = grid_z+grid_ave;
  int len_in2 = len_in*len_in;
  
  for(int i = 0; i < array_size; i++){
    int eu = uv[i].x() - u0;
    int ev = uv[i].y() - v0;
    int d2 = eu*eu + ev*ev;
    //ROS_INFO_STREAM("d2="<<d2);
    if(d2<=len_in2){
      //pc_array_feature.push_back(pc_masks[i].point);
      Vector3d in_box =Vector3d(array_pc[i].x(), array_pc[i].y(), array_pc[i].z()); 
      //depend on uav's last estimated position
     
      pc_array_yolo.push_back(in_box);
      //没有点云ekf
      //pc_array_yolo.push_back(in_box);

      //depth picture
      // if(array_pc[i].z()<50)
      // x_depth[int(array_pc[i].z())-1] = x_depth[int(array_pc[i].z())-1] + 1 ;
      
    }
  }
  //plot the depth picture
  // char depth_png[100];
  // sprintf(depth_png, "/home/mao/Pictures/depth/%d.png", x_depth_png);
  // plt::plot(x_label, x_depth);
  // plt::title("x_depth");
  // // plt::show();
  // plt::save(depth_png);
  // plt::close();
  // // x_depth.clear();
  // x_depth_png++;


  m_ekf.lock();
  double ave_x_last = x_k_k.x();
  ROS_INFO_STREAM("x_K_k = "<<x_k_k.x());
  double ave_y_last = x_k_k.y();
  double ave_z_last = x_k_k.z();
  m_ekf.unlock();
  ROS_INFO_STREAM("size of box = "<<pc_array_yolo.size());
  int size_in_box = pc_array_yolo.size();
  Vector3d pc_i;
  for(int j=4;j>0;j--){
    double ave_x = 0;
    double ave_y = 0;
    double ave_z = 0;
    double val_n = 0;
    //#pragma omp parallel for reduction(+: ave_x, ave_y, ave_z)
    for(int i = 0;i<size_in_box;i++){
      pc_i = pc_array_yolo[i];
      if(ave_x_last != 0 && ave_y_last != 0 && ave_z_last != 0){
        if((abs(pc_i.x()-ave_x_last)>bottle*j/2.0 || abs(pc_i.y()-ave_y_last)>bottle*j/2.0 || abs(pc_i.z()-ave_z_last)>bottle*j/2.0)){
          continue;
        }
      }
      val_n ++;
      ave_x += pc_i.x();
      ave_y += pc_i.y();
      ave_z += pc_i.z();
      
    }
    if(val_n == 0) break;
    ave_x_last = ave_x/val_n;
    ave_y_last = ave_y/val_n;
    ave_z_last = ave_z/val_n;
    // ROS_INFO_STREAM("val_n = "<<val_n);
    // ROS_INFO_STREAM("ave:("<<ave_x_last<<", "<<ave_y_last<<", "<<ave_z_last<<")");
  }
  
  Vector3d out = Vector3d(ave_x_last,ave_y_last,ave_z_last);

  return out;
}


void PC_EKF_uodate(Vector3d measurements){
  double sigma2 = 0.1;
  Matrix3d V_pc = sigma2 * Matrix3d::Identity();//noise covariance of measurents;
  Matrix3d H = Matrix3d::Identity();
  m_ekf.lock();
  Matrix3d K = P_k_k*H.transpose()*(H*P_k_k*H.transpose() + V_pc).inverse();
  x_k_k = x_k_k + K*(measurements - x_k_k);
  P_k_k = (eye3-K*H)*P_k_k*(eye3-K*H).transpose() + K*V_pc*K.transpose();
  ROS_INFO_STREAM("xyz_depth_estimate = "<<x_k_k.x()<<", "<<x_k_k.y()<<", "<<x_k_k.z());
  m_ekf.unlock();
  return;
}





//单线程处理
int upsampling_pro( pcl::PointXYZ &maxxyz, pcl::PointXYZ &minxyz, minmaxuv_ &minmaxuv, int w, int h, int c, int nof) {
	// Use std::chrono to time the algorithm
  double S_x=0, S_y=0, S_z=0, Y_x=0, Y_y=0, Y_z=0;
  //double mr_x=maxxyz.x, mr_y=maxxyz.y, mr_z=maxxyz.z;

  int minrow = floor(minmaxuv.vmin) + 1;
  int maxrow = (int)minmaxuv.vmax +1;
  //ROS_INFO_STREAM("v(max, min) = "<<maxrow<<", "<<minrow);
  //ROS_INFO_STREAM("u(max, min) = "<<minmaxuv.umax<<", "<<minmaxuv.umin);
  cv::Mat image_upsample = cv::Mat(h, w, CV_8UC3, cv::Scalar(0, 0, 0)); //initialize the mat variable according to the size of image
  cv::Mat image_upsample_no_rect = cv::Mat(h, w, CV_8UC3, cv::Scalar(0, 0, 0));
  cv::Mat image_upsample_original = cv::Mat(h, w, CV_8UC3, cv::Scalar(0, 0, 0));
  cv::Mat image_upsample_1 = cv::Mat(h, w, CV_8UC3, cv::Scalar(0, 0, 0));
  cv::Mat image_upsample_5 = cv::Mat(h, w, CV_8UC3, cv::Scalar(0, 0, 0));
  cv::Mat image_time_cmp = cv::Mat(h, w, CV_8UC3, cv::Scalar(0, 0, 0));
  int size_3c = image_upsample.rows*image_upsample.cols*c;
  //TicToc allocate;
  // double* ima3d = (double*)malloc(sizeof(double)*(size_3c));
  // double* ima3d_ = (double*)malloc(sizeof(double)*(size_3c));
  int mask_id = pc_manager.current_id;
  
  // ROS_INFO_STREAM("id :"<<mask_id);
  //vector<make_pair<int, double>> ima3d;
  //vector<make_pair<int, double>> ima3d_;
  vector<Mask_pc>& pc_masks_no_rect = pc_manager.mask_no_rect.pc_masks_single;
  vector<Mask_pc>& pc_masks = pc_manager.maskn.pc_masks_single;
  int mask_size = pc_masks.size();
  // ROS_INFO_STREAM("mask size: "<< mask_size);
  double maxima3d[3];
  double minima3d[3];
  minima3d[0]=-1;
  minima3d[1]=-1;
  minima3d[2]=-1;
  maxima3d[0]=0;
  maxima3d[1]=0;
  maxima3d[2]=0;
  
  double maxima3d_no_rect[3];

  maxima3d_no_rect[0]=0;
  maxima3d_no_rect[1]=0;
  maxima3d_no_rect[2]=0;


 
  //Wp_x = 0, Wp_y=0, Wp_z=0, Gr_x=0, Gr_y=0, Gr_z=0, Gs=0;
  //through mask
  S_x=0; Y_x=0;
  S_y=0; Y_y=0;
  S_z=0; Y_z=0;

  TicToc mask_method_i;//计算消耗主要在计算部分
  /*through masks and save numerator and denominator in image-sized map */
  double* ima3d = pc_manager.maskn.ima3d;
  double* ima3d_ = pc_manager.maskn.ima3d_;
  
  //#pragma omp parallel for 
  //ROS_INFO_STREAM("pc_manager.maskn.pc_masks_single"<<pc_masks[0].point.G_x);
  int col_c = image_upsample.cols*c;
  for (int i_g = 0; i_g < mask_size; i_g ++){
  
    double Gr_x = pc_masks[i_g].point.Gr_x;
    double Gr_y = pc_masks[i_g].point.Gr_y;
    double Gr_z = pc_masks[i_g].point.Gr_z;
    double Gs = 1;
    double G_x = pc_masks[i_g].point.G_x;
    double G_y = pc_masks[i_g].point.G_y;
    double G_z = pc_masks[i_g].point.G_z;

    double pu = pc_masks[i_g].point.u_px;
    double pv = pc_masks[i_g].point.v_px;

    double d2;
    int grid_index = 0;
    int list_index = int((pu-int(pu))/m_split) + int((pv-int(pv))/m_split)*n_split;
    vector<double>& grid_map = pc_manager.grid_param[list_index]; 
    for(int v = pc_masks[i_g].v_down; v<pc_masks[i_g].v_up; v++){
      for(int u =  pc_masks[i_g].u_down; u<pc_masks[i_g].u_up; u++){
        // d2 = sqrt((u - pu)*(u - pu) + (v-pv)*(v-pv));
        // if(d2<0.0001){
        //   //ROS_INFO_STREAM("d2 = "<<d2);
        //   d2 = 0.0001;
          
        // }
        // //ROS_INFO_STREAM("d2 = "<<d2);
        // Gs = 1.0/d2;
        
        
        Gs = grid_map[grid_index];
        grid_index++;
        int index = v*col_c + u*c;
        double x0 = Gs*Gr_x;
        double x1 = Gs*Gr_y;
        double x2 = Gs*Gr_z;
        double y0 = Gs*G_x;
        double y1 = Gs*G_y;
        double y2 = Gs*G_z;
        //m_thread.lock();
        if(ima3d == NULL || ima3d_ == NULL){
          ROS_ERROR("ima3d or ima3d_ pointer is NULL! restart!");
          pc_manager.maskn.malloc_ok = false;
          return 0;
        }
        ima3d[index] += x0;
        ima3d[index +1] += x1;
        ima3d[index +2] += x2;
        ima3d_[index] += y0;
        ima3d_[index +1] += y1;
        ima3d_[index +2] += y2;
        // m_thread.unlock();
      }
    }
  }
  ROS_DEBUG_STREAM("mask calculate_grid: "<<mask_method_i.toc()<<"ms");
  ave_mask_cal_i= (ave_mask_cal_i*(i_n-1) + mask_method_i.toc())/i_n;
  ROS_DEBUG_STREAM("ave mask calculate_grid: "<<ave_mask_cal_i<<"ms");
  //compare with original upsampling
  int image_size = w*h*c;
  double* ima3do = (double*)malloc(sizeof(double)*(image_size));
  double* ima3do_ = (double*)malloc(sizeof(double)*(image_size));
  double* ima3d5 = (double*)malloc(sizeof(double)*(image_size));
  double* ima3d5_ = (double*)malloc(sizeof(double)*(image_size));
  double* ima3d1 = (double*)malloc(sizeof(double)*(image_size));
  double* ima3d1_ = (double*)malloc(sizeof(double)*(image_size));
  double* ima3d_time = (double*)malloc(sizeof(double)*(image_size));
  double* ima3d_time_ = (double*)malloc(sizeof(double)*(image_size));
  if(compare_upsampling){
    //set zero
    for(int i = 0; i<image_size; i++){
      *(ima3do + i) = 0;
      *(ima3do_ + i) = 0;
      
    }
  }
  if(resolution_cmp){
    //set zero
    for(int i = 0; i<image_size; i++){
      *(ima3d1 + i) = 0;
      *(ima3d1_ + i) = 0;
      *(ima3d5 + i) = 0;
      *(ima3d5_ + i) = 0;
      
    }
  }
  if(original_cmp){
    for(int i = 0; i<image_size; i++){
      *(ima3d_time + i) = 0;
      *(ima3d_time_ + i) = 0;
      
    }
  }
  
  //compare time cost of calculating pixel distance 
  if(compare_upsampling){
    TicToc mask_method;//计算消耗主要在计算部分
    
    for (int i_g = 0; i_g < mask_size; i_g ++){
    
      double Gr_x = pc_masks[i_g].point.Gr_x;
      double Gr_y = pc_masks[i_g].point.Gr_y;
      double Gr_z = pc_masks[i_g].point.Gr_z;
      double Gs = 0;
      double G_x = pc_masks[i_g].point.G_x;
      double G_y = pc_masks[i_g].point.G_y;
      double G_z = pc_masks[i_g].point.G_z;

      double pu = pc_masks[i_g].point.u_px;
      double pv = pc_masks[i_g].point.v_px;
      double d2;
      for(int v = pc_masks[i_g].v_down; v<pc_masks[i_g].v_up; v++){
        for(int u =  pc_masks[i_g].u_down; u<pc_masks[i_g].u_up; u++){
          d2 = sqrt((u - pu)*(u - pu) + (v-pv)*(v-pv));
          if(d2<0.0001){
            //ROS_INFO_STREAM("d2 = "<<d2);
            d2 = 0.0001;
            
          }
          //ROS_INFO_STREAM("d2 = "<<d2);
          Gs = 1.0/d2;
          int index = v*image_upsample.cols*c + u*c;
          double x0 = Gs*Gr_x;
          double x1 = Gs*Gr_y;
          double x2 = Gs*Gr_z;
          double y0 = Gs*G_x;
          double y1 = Gs*G_y;
          double y2 = Gs*G_z;
          //m_thread.lock();
          if(ima3d == NULL || ima3d_ == NULL){
            ROS_ERROR("ima3d or ima3d_ pointer is NULL! restart!");
            pc_manager.maskn.malloc_ok = false;
            return 0;
          }
          ima3do[index] += x0;
          ima3do[index +1] += x1;
          ima3do[index +2] += x2;
          ima3do_[index] += y0;
          ima3do_[index +1] += y1;
          ima3do_[index +2] += y2;
          // m_thread.unlock();
        }
      }
    }
    ROS_DEBUG_STREAM("mask calculate_original: "<<mask_method.toc()<<"ms");
    ave_mask_cal= (ave_mask_cal*(i_n-1) + mask_method.toc())/i_n;
    ROS_DEBUG_STREAM("ave mask calculate_original: "<<ave_mask_cal<<"ms");
  }
  if(resolution_cmp){
    for (int i_g = 0; i_g < mask_size; i_g ++){
    
      double Gr_x = pc_masks[i_g].point.Gr_x;
      double Gr_y = pc_masks[i_g].point.Gr_y;
      double Gr_z = pc_masks[i_g].point.Gr_z;
      double Gs = 0;
      double G_x = pc_masks[i_g].point.G_x;
      double G_y = pc_masks[i_g].point.G_y;
      double G_z = pc_masks[i_g].point.G_z;

      double pu = pc_masks[i_g].point.u_px;
      double pv = pc_masks[i_g].point.v_px;
      double d2;
      int n_split1 = 1;
      double m_split1 = 1.0/n_split1;
      int n_split5 = 5;
      double m_split5 = 1.0/n_split5;
      int grid_index = 0;
      int list_index1 = int((pu-int(pu))/m_split1) + int((pv-int(pv))/m_split1)*n_split1;
      int list_index5 = int((pu-int(pu))/m_split5) + int((pv-int(pv))/m_split5)*n_split5;
      vector<double>& grid_map1 = pc_manager.grid_param1[list_index1];
      vector<double>& grid_map5 = pc_manager.grid_param5[list_index5]; 
      for(int v = pc_masks[i_g].v_down; v<pc_masks[i_g].v_up; v++){
        for(int u =  pc_masks[i_g].u_down; u<pc_masks[i_g].u_up; u++){
          d2 = sqrt((u - pu)*(u - pu) + (v-pv)*(v-pv));
          if(d2<0.0001){
            //ROS_INFO_STREAM("d2 = "<<d2);
            d2 = 0.0001;
            
          }
          //ROS_INFO_STREAM("d2 = "<<d2);
          Gs = grid_map1[grid_index];
          int index = v*image_upsample.cols*c + u*c;
          double x0 = Gs*Gr_x;
          double x1 = Gs*Gr_y;
          double x2 = Gs*Gr_z;
          double y0 = Gs*G_x;
          double y1 = Gs*G_y;
          double y2 = Gs*G_z;
          //m_thread.lock();
          if(ima3d == NULL || ima3d_ == NULL){
            ROS_ERROR("ima3d or ima3d_ pointer is NULL! restart!");
            pc_manager.maskn.malloc_ok = false;
            return 0;
          }
          ima3d1[index] += x0;
          ima3d1[index +1] += x1;
          ima3d1[index +2] += x2;
          ima3d1_[index] += y0;
          ima3d1_[index +1] += y1;
          ima3d1_[index +2] += y2;

          Gs = grid_map5[grid_index];
          grid_index++;
          x0 = Gs*Gr_x;
          x1 = Gs*Gr_y;
          x2 = Gs*Gr_z;
          y0 = Gs*G_x;
          y1 = Gs*G_y;
          y2 = Gs*G_z;
          ima3d5[index] += x0;
          ima3d5[index +1] += x1;
          ima3d5[index +2] += x2;
          ima3d5_[index] += y0;
          ima3d5_[index +1] += y1;
          ima3d5_[index +2] += y2;
          // m_thread.unlock();
        }
      }
    }
  }


  pc_masks.clear();
  if(compare_rect){
    double* ima3d = pc_manager.mask_no_rect.ima3d;
    double* ima3d_ = pc_manager.mask_no_rect.ima3d_;
    int mask_size_no_rect = pc_masks_no_rect.size();
    // ROS_INFO_STREAM("mask size = "<<mask_size_no_rect);
    for (int i_g = 0; i_g < mask_size_no_rect; i_g ++){
      double Gr_x = pc_masks_no_rect[i_g].point.Gr_x;
      double Gr_y = pc_masks_no_rect[i_g].point.Gr_y;
      double Gr_z = pc_masks_no_rect[i_g].point.Gr_z;
      double Gs = 1;
      double G_x = pc_masks_no_rect[i_g].point.G_x;
      double G_y = pc_masks_no_rect[i_g].point.G_y;
      double G_z = pc_masks_no_rect[i_g].point.G_z;

      double pu = pc_masks_no_rect[i_g].point.u_px;
      double pv = pc_masks_no_rect[i_g].point.v_px;
      double d2;
      // int grid_index = 0;
      for(int v = pc_masks_no_rect[i_g].v_down; v<pc_masks_no_rect[i_g].v_up; v++){
        for(int u =  pc_masks_no_rect[i_g].u_down; u<pc_masks_no_rect[i_g].u_up; u++){
          d2 = sqrt((u - pu)*(u - pu) + (v-pv)*(v-pv));
          if(d2<0.0001){
            //ROS_INFO_STREAM("d2 = "<<d2);
            d2 = 0.0001;
            
          }
          // //ROS_INFO_STREAM("d2 = "<<d2);
          Gs = 1.0/d2;
          // Gs = pc_manager.grid_param[0][grid_index];
          // grid_index++;
          int index = v*col_c + u*c;
          double x0 = Gs*Gr_x;
          double x1 = Gs*Gr_y;
          double x2 = Gs*Gr_z;
          double y0 = Gs*G_x;
          double y1 = Gs*G_y;
          double y2 = Gs*G_z;
          //m_thread.lock();
          if(ima3d == NULL || ima3d_ == NULL){
            ROS_ERROR("ima3d or ima3d_ pointer is NULL! restart!");
            pc_manager.maskn.malloc_ok = false;
            return 0;
          }
          ima3d[index] += x0;
          ima3d[index +1] += x1;
          ima3d[index +2] += x2;
          ima3d_[index] += y0;
          ima3d_[index +1] += y1;
          ima3d_[index +2] += y2;
          // m_thread.unlock();
        }
      }
    }
    pc_masks_no_rect.clear();
  }
  //ROS_INFO("what happened???");

  /*save the max denominator in all 4 serials data*/
  if(pc_manager.init){
    
    //4mask process
    TicToc comp_max;
    double Dx_i;
    double Dy_i;
    double Dz_i;
    for(int vali = minrow; vali < maxrow; vali++){
        for(int uali = (int)minmaxuv.umin; uali < (int)minmaxuv.umax; uali++){
        unsigned char *row_ptr = image_upsample.ptr<unsigned char>(vali);  // row_ptr is the pointer pointing to row vali
        unsigned char *data_ptr = &row_ptr[uali * image_upsample.channels()]; // data_ptr points to the pixel data to be accessed
        //notice the order is B,G,R in opencv, and R,G,B in matlab
        int index = vali*image_upsample.cols*c + uali*c;
        /*calculating denominator and set to 1 with 0 case*/
        
        double denominator_x = pc_manager.maskn.ima3d[index];
        double denominator_y = pc_manager.maskn.ima3d[index+1];
        double denominator_z = pc_manager.maskn.ima3d[index+2];
        if(denominator_x == 0) denominator_x = 1;
        if(denominator_y == 0) denominator_y = 1;
        if(denominator_z == 0) denominator_z = 1;
        if(pc_manager.maskn.ima3d_[index] == 0){
          
          continue;
        }
        /*calculating depth map(befor normalized) and save the max one*/
        Dx_i = (pc_manager.maskn.ima3d_[index])/denominator_x;
        Dy_i = (pc_manager.maskn.ima3d_[index+1])/denominator_y;
        Dz_i = (pc_manager.maskn.ima3d_[index+2])/denominator_z;
        if(maxima3d[0] < Dx_i ) (maxima3d[0] = Dx_i); 
        if(maxima3d[1] < Dy_i ) (maxima3d[1] = Dy_i) ;
        if(maxima3d[2] < Dz_i ) (maxima3d[2] = Dz_i) ;
        // if(minima3d[0] > Dx_i || minima3d[0]<0) (minima3d[0] = Dx_i); 
        // if(minima3d[1] > Dy_i || minima3d[1]<0) (minima3d[1] = Dy_i) ;
        // if(minima3d[2] > Dz_i || minima3d[2]<0) (minima3d[2] = Dz_i) ;
      }
    }
    ROS_DEBUG_STREAM("comp_max:"<<comp_max.toc()<<"ms");
    
    for(int vali = minrow; vali < maxrow; vali++)
      for(int uali = (int)minmaxuv.umin; uali < (int)minmaxuv.umax; uali++){
        unsigned char *row_ptr = image_upsample.ptr<unsigned char>(vali);  // row_ptr is the pointer pointing to row vali
        unsigned char *data_ptr = &row_ptr[uali * image_upsample.channels()]; // data_ptr points to the pixel data to be accessed
        //notice the order is B,G,R in opencv, and R,G,B in matlab
        int index = vali*image_upsample.cols*c + uali*c;
        //compare with the original upsampling algorithm
        
        if(compare_upsampling){
          unsigned char *row_ptr_o = image_upsample_original.ptr<unsigned char>(vali);  // row_ptr is the pointer pointing to row vali
          unsigned char *data_ptr_o = &row_ptr_o[uali * image_upsample_original.channels()]; 
          data_ptr_o[2] = (unsigned char)(255.0*(( ima3do_[index])/(ima3do[index]))/ maxima3d[0]);
          data_ptr_o[0] = (unsigned char)(255.0*(( ima3do_[index+1])/(ima3do[index+1]))/maxima3d[1]);
          data_ptr_o[1] = (unsigned char)(255.0*(( ima3do_[index+2])/(ima3do[index+2]))/maxima3d[2]);
        }
        if(resolution_cmp){
          unsigned char *row_ptr_1 = image_upsample_1.ptr<unsigned char>(vali);  // row_ptr is the pointer pointing to row vali
          unsigned char *data_ptr_1 = &row_ptr_1[uali * image_upsample_1.channels()]; 
          unsigned char *row_ptr_5 = image_upsample_5.ptr<unsigned char>(vali);  // row_ptr is the pointer pointing to row vali
          unsigned char *data_ptr_5 = &row_ptr_5[uali * image_upsample_5.channels()];
          data_ptr_1[2] = (unsigned char)(255.0*(( ima3d1_[index])/(ima3d1[index]))/ maxima3d[0]);
          data_ptr_1[0] = (unsigned char)(255.0*(( ima3d1_[index+1])/(ima3d1[index+1]))/maxima3d[1]);
          data_ptr_1[1] = (unsigned char)(255.0*(( ima3d1_[index+2])/(ima3d1[index+2]))/maxima3d[2]);

          data_ptr_5[2] = (unsigned char)(255.0*(( ima3d5_[index])/(ima3d5[index]))/ maxima3d[0]);
          data_ptr_5[0] = (unsigned char)(255.0*(( ima3d5_[index+1])/(ima3d5[index+1]))/maxima3d[1]);
          data_ptr_5[1] = (unsigned char)(255.0*(( ima3d5_[index+2])/(ima3d5[index+2]))/maxima3d[2]);
        }
        if(compare_rect){
          unsigned char *row_ptr_no_rect = image_upsample_no_rect.ptr<unsigned char>(vali);  // row_ptr is the pointer pointing to row vali
          unsigned char *data_ptr_no_rect = &row_ptr_no_rect[uali * image_upsample_original.channels()]; 
          data_ptr_no_rect[2] = (unsigned char)(255.0*(( pc_manager.mask_no_rect.ima3d_[index])/(pc_manager.mask_no_rect.ima3d[index]))/ maxima3d[0]);
          data_ptr_no_rect[0] = (unsigned char)(255.0*(( pc_manager.mask_no_rect.ima3d_[index+1])/(pc_manager.mask_no_rect.ima3d[index+1]))/maxima3d[1]);
          data_ptr_no_rect[1] = (unsigned char)(255.0*(( pc_manager.mask_no_rect.ima3d_[index+2])/(pc_manager.mask_no_rect.ima3d[index+2]))/maxima3d[2]);
        }

        if(original_cmp){
          unsigned char *row_ptr_o = image_time_cmp.ptr<unsigned char>(vali);  // row_ptr is the pointer pointing to row vali
          unsigned char *data_ptr_o = &row_ptr_o[uali * image_time_cmp.channels()]; 
          data_ptr_o[2] = (unsigned char)(255.0*( ima3d_time[index])/ maxima3d[0]);
          data_ptr_o[0] = (unsigned char)(255.0*( ima3d_time[index+1])/maxima3d[1]);
          data_ptr_o[1] = (unsigned char)(255.0*( ima3d_time[index+2])/maxima3d[2]);
        }
        data_ptr[2] = (unsigned char)(255.0*( (pc_manager.maskn.ima3d_[index])/(pc_manager.maskn.ima3d[index]))/ maxima3d[0]);
        data_ptr[0] = (unsigned char)(255.0*( (pc_manager.maskn.ima3d_[index+1])/(pc_manager.maskn.ima3d[index+1]))/maxima3d[1]);
        data_ptr[1] = (unsigned char)(255.0*( (pc_manager.maskn.ima3d_[index+2])/(pc_manager.maskn.ima3d[index+2]))/maxima3d[2]);
 
    }
    
    ROS_DEBUG_STREAM("get map: "<<comp_max.toc()<<"ms");
    ROS_DEBUG_STREAM("mask method: "<<mask_method_i.toc()<<"ms");
    
  

    //cv::Mat image_upsample2 = cv::Mat::zeros(h, w, CV_8UC(3)); //initialize the mat variable according to the size of image

    cv_bridge::CvImage out_msg;
    out_msg.header   = imgrgb_cur->header; // Same timestamp and tf frame as input image
    // ROS_INFO_STREAM("timestamp_img = "<<out_msg.header.stamp.toNSec());
    out_msg.encoding = sensor_msgs::image_encodings::BGR8; //gbr8
    out_msg.image    = image_upsample; // Your cv::Mat
    
    pubimg.publish(imgrgb_cur);
    pubimg_upsample.publish(out_msg.toImageMsg());
    // pubimg.publish(out_msg.toImageMsg());
    // pubimg_upsample.publish(imgrgb_cur);






    //ROS_DEBUG("image published.");
    ROS_DEBUG_STREAM("processing and pub 0.4s data: "<<all_time.toc()<<"ms");
    ave_total = (ave_total*(i_n-1) + all_time.toc())/i_n;
    ROS_DEBUG_STREAM("ave total: "<<ave_total<<"ms\n");
    

    //compare time with the original method
    if(original_cmp){
      double Dx_i;
      double Dy_i;
      double Dz_i;
      TicToc cmp_t;//计算消耗主要在计算部分
      for (int v=0; v< maxrow - minrow; v=v+1){
        for (int u= (int)minmaxuv.umin; u< (int)minmaxuv.umax; u=u+1){
          long int gridno = (v+minrow)*w+u;
          int s_g = pc_manager.pc_array_grid[gridno].size();
          double x0 = 0;
          double x1 = 0;
          double x2 = 0;
          double y0 = 0;
          double y1 = 0;
          double y2 = 0;
          for (int i_g = 0; i_g < s_g; i_g ++){
            double pu = pc_manager.pc_array_grid[gridno][i_g].u_px;
            double pv = pc_manager.pc_array_grid[gridno][i_g].v_px;
            double dx = pc_manager.pc_array_grid[gridno][i_g].x_3d;
            double dy = abs(pc_manager.pc_array_grid[gridno][i_g].y_3d);
            double dz = abs(pc_manager.pc_array_grid[gridno][i_g].z_3d);
            double d2 = sqrt((u - pu)*(u - pu) + (v-pv)*(v-pv));
            if(d2<0.0001){
              //ROS_INFO_STREAM("d2 = "<<d2);
              d2 = 0.0001;    
            }
            double Gr_x = pc_manager.pc_array_grid[gridno][i_g].Gr_x;
            double Gr_y = pc_manager.pc_array_grid[gridno][i_g].Gr_y;
            double Gr_z = pc_manager.pc_array_grid[gridno][i_g].Gr_z;
            double Gs = 1.0/d2;
            double G_x = pc_manager.pc_array_grid[gridno][i_g].G_x;
            double G_y = pc_manager.pc_array_grid[gridno][i_g].G_y;
            double G_z = pc_manager.pc_array_grid[gridno][i_g].G_z;
            x0 += Gs*Gr_x;
            x1 += Gs*Gr_y;
            x2 += Gs*Gr_z;
            y0 += Gs*G_x;
            y1 += Gs*G_y;
            y2 += Gs*G_z;
          }

          pc_manager.pc_array_grid[gridno].clear();

          if (x0==0) {x0=1;}
          if (x1==0) {x1=1;}
          if (x2==0) {x2=1;}

          Dx_i = y0/x0;
          Dy_i = y1/x1;
          Dz_i = y2/x2;
          ima3d_time[(v+minrow)*image_upsample.cols*c + u*c] = Dx_i;
          ima3d_time[(v+minrow)*image_upsample.cols*c + u*c +1] = Dy_i;
          ima3d_time[(v+minrow)*image_upsample.cols*c + u*c +2] = Dz_i;
                    
        }
      }
      
    
      for(int vali = minrow; vali < maxrow; vali++)
        for(int uali = (int)minmaxuv.umin; uali < (int)minmaxuv.umax; uali++){
          unsigned char *row_ptr = image_upsample.ptr<unsigned char>(vali);  // row_ptr is the pointer pointing to row vali
          unsigned char *data_ptr = &row_ptr[uali * image_upsample.channels()]; // data_ptr points to the pixel data to be accessed
          //notice the order is B,G,R in opencv, and R,G,B in matlab
          int index = vali*image_upsample.cols*c + uali*c;
          //compare with the original upsampling algorithm
          
        
        if(original_cmp){
          unsigned char *row_ptr_o = image_time_cmp.ptr<unsigned char>(vali);  // row_ptr is the pointer pointing to row vali
          unsigned char *data_ptr_o = &row_ptr_o[uali * image_time_cmp.channels()]; 
          data_ptr_o[2] = (unsigned char)(255.0*( ima3d_time[index])/ maxima3d[0]);
          data_ptr_o[0] = (unsigned char)(255.0*( ima3d_time[index+1])/maxima3d[1]);
          data_ptr_o[1] = (unsigned char)(255.0*( ima3d_time[index+2])/maxima3d[2]);
        }
        
      }
      b_n++;
      ROS_DEBUG_STREAM("original_step2: "<<cmp_t.toc()<<"ms");
      ave_original_2= (ave_original_2*(b_n-1) + cmp_t.toc())/b_n;
      ROS_DEBUG_STREAM("ave original_step2: "<<ave_original_2<<"ms");
    }
    
    
    
    if(image_save){
      double hz = 0;
      cur_setfre = imgrgb_cur->header.stamp.toSec();
      if(last_setfre == 0){
        last_setfre = cur_setfre;
        ROS_INFO("return");
        return 0;
      }
      if(cur_setfre - last_setfre ==0){
        ROS_ERROR("TIMEINSTANCE ERROR: cur_setfre - last_setfre == 0.");
        return 0 ;
      }
      hz = 1.0/(cur_setfre - last_setfre);
      ROS_DEBUG_STREAM("hz = "<<hz);
      if(hz<2.1 && hz>0){
        last_setfre = cur_setfre;
        ROS_DEBUG_STREAM("save iamge");
        
        char png_name_upsampling[200];
        char png_name[200];
        sprintf(png_name_upsampling, "/media/mao/PortableSSD/Dataset/mpc_dataset/dataset_20230201_lowlight/%05dimg_b_2.png", i_pc_count);
        sprintf(png_name, "/media/mao/PortableSSD/Dataset/mpc_dataset/dataset_20230201_lowlight/%05dimg_b.png", i_pc_count);
        i_pc_count ++;
        
        //cv::cvtColor(image_upsample, image_upsample_grey, cv::COLOR_BGR2GRAY);
        //cv::imshow("image_upsample", image_upsample_grey);
        //cv::namedWindow("image_upsample", cv::WINDOW_NORMAL);
        //cv::imshow("image_upsample", image_upsample);
        //cv::namedWindow("image", cv::WINDOW_NORMAL);
        //cv::imshow("image", img_cur);
        cv::imwrite(png_name_upsampling, image_upsample); //save the image
        cv::imwrite(png_name, img_cur); 
        //cv::waitKey(1);
        // cv::Mat channel[3];
        // cv::split(image_upsample, channel);//split into three channels
        
        // char pic1[50];
        // char pic2[50];
        // char pic3[50];
        // sprintf(pic1, "/tmp/%02dupsamplesave_0.png",nof);
        // sprintf(pic2, "/tmp/%02dupsamplesave_1.png",nof);
        // sprintf(pic3, "/tmp/%02dupsamplesave_2.png",nof);
        
        // cv::imshow("x of image_upsample", channel[0]);
        //cv::imwrite(pic1, channel[0]); //save the image
        // cv::imshow("y of image_upsample", channel[1]);
        //cv::imwrite(pic2, channel[1]); //save the image
        // cv::imshow("z of image_upsample", channel[2]);
        //cv::imwrite(pic3, channel[2]); //save the image
        //cv::destroyAllWindows();
      }
        
    }
    // search_box_yolo
    // // if(ifdetection==1){
      
      // // ROS_INFO_STREAM("box_grid_points.size() = "<<box_grid_points.size());
      m_visualization.lock();
      // for(int i = 0;i < box_grid_points.size();i++){
      //   cv::circle(img_cur, box_grid_points[i], 1, cv::Scalar(0, 255, 0));
      //   cv::circle(image_upsample, box_grid_points[i], 1, cv::Scalar(0, 255, 0));
      // }
      box_grid_points.clear();
      m_visualization.unlock();
      // ifdetection = 0;
    // }
    
    if(show_image){
      if(original_cmp){
        cv::imshow("XYZ_original", image_time_cmp);
      }

      if(compare_rect){
        cv::imshow("XYZ_no_motion_compensation", image_upsample_no_rect);
      
      }
      if(compare_upsampling){
        cv::imshow("XYZ_continue_method", image_upsample_original);
      }
      if(resolution_cmp){
        cv::imshow("XYZ_resolution_1.0", image_upsample_1);
        cv::imshow("XYZ_resolution_0.2", image_upsample_5);
      }
      //均值化
      // cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
      // cv::Mat img_normalized, img_cur_split[3];
      // vector<cv::Mat> img_normalized_split(3);
      // cv::split(image_upsample, img_cur_split);
      // for(int i = 0; i<3; i++){
      //   clahe->apply(img_cur_split[i], img_normalized_split[i]);
      // }
      // cv::merge(img_normalized_split, img_normalized);

      // cv::imshow("img_normalized", img_normalized);


      //xyz
      // cv::Mat channel[3];
      // cv::split(image_upsample, channel);//split into three channels
      // cv::imshow("x of image_upsample", channel[2]);
      // cv::imshow("y of image_upsample", channel[0]);
      // cv::imshow("z of image_upsample", channel[1]);
      cv_bridge::CvImagePtr cv_ptr;
      cv_ptr = cv_bridge::toCvCopy(imgrgb_cur, sensor_msgs::image_encodings::BGR8);
      img_cur  = cv_ptr -> image;
      cv::circle(image_upsample, circle_center, search_box_yolo, cv::Scalar(0, 255, 0));
      cv::circle(img_cur, circle_center, search_box_yolo, cv::Scalar(0, 255, 0));
      cv::circle(image_upsample, rect_circle_center, search_box_yolo, cv::Scalar(0, 0, 255));
      cv::circle(img_cur, rect_circle_center, search_box_yolo, cv::Scalar(0, 0, 255));
      cv::imshow("XYZ_resolution_0.1", image_upsample);
      cv::imshow("RGB", img_cur);
      cv::waitKey(1);
    }
  }else{
    //intialization error
    cur_setfre = 0;
    last_setfre = 0;
    ROS_INFO("Reset setfre value.");
    
  }
  free(ima3do);
  free(ima3do_);
  free(ima3d1);
  free(ima3d1_);
  free(ima3d5);
  free(ima3d5_);
  free(ima3d_time);
  free(ima3d_time_);
  return 0;
}


bool compare_pc_v(const pointcoordinate& left,const pointcoordinate& right){
    return left.v_px<right.v_px; //ascending sort
}




void PC_Wrapper::UpdateMask(int id){
  current_id = id;
  // mask_win[id].mask_id = id;
  // mask_win[id].SetZeros();
  maskn.SetZeros();
  mask_no_rect.SetZeros();
}

// void PC_Wrapper::UpdateMax3d(){
  
//   for(int i = 0; i < WINDOW_SIZE; i++){
//     if(Maxima3d[0] < mask_win[i].maxima3d[0]) Maxima3d[0] = mask_win[i].maxima3d[0];
//     if(Maxima3d[1] < mask_win[i].maxima3d[1]) Maxima3d[1] = mask_win[i].maxima3d[1];
//     if(Maxima3d[2] < mask_win[i].maxima3d[2]) Maxima3d[2] = mask_win[i].maxima3d[2];
//   }
  
// }


void ID_MASK::SetZeros(){
  for(int i = 0; i<image_size; i++){
    *(ima3d + i) = 0;
    *(ima3d_ + i) = 0;
  }
  //ROS_INFO_STREAM("mask "<<this->mask_id<<" recalled SetZero.");
}

// void ID_MASK::ResetMaskSize(const int size){
//   free(pc_masks);
//   pc_masks = (Mask_pc*)malloc(sizeof(Mask_pc)*(size));
//   masksize = size;
// }