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
//extern vector<pointcoordinate> pc_array_grid[252672];



// extern double feat_point[2]; //the feature position in the pixel frame, detected by the detector
// extern vector<pointcoordinate> pc_array_feature; //put the feature points in the array
// extern ros::Publisher pubimg_upsample;
// extern sensor_msgs::Image imgrgb;
// extern double sum_x;

// extern PC_Wrapper pc_manager;
// extern std::mutex m_feature;
// extern int sum_pc_i;
// extern TicToc all_time;





// int upsampling_pro( pcl::PointXYZ &maxxyz, pcl::PointXYZ &minxyz, minmaxuv_ &minmaxuv, int w, int h, int c, int nof) {
// 	// Use std::chrono to time the algorithm
//   double S_x=0, S_y=0, S_z=0, Y_x=0, Y_y=0, Y_z=0;
//   double mr_x=maxxyz.x, mr_y=maxxyz.y, mr_z=maxxyz.z;

//   int minrow = floor(minmaxuv.vmin) + 1;
//   int maxrow = (int)minmaxuv.vmax +1;
//   cv::Mat image_upsample = cv::Mat(h, w, CV_8UC3, cv::Scalar(0, 0, 0)); //initialize the mat variable according to the size of image
//   int size_3c = image_upsample.rows*image_upsample.cols*c;
//   //TicToc allocate;
//   //double* ima3d = (double*)malloc(sizeof(double)*(size_3c));
//   //double* ima3d_ = (double*)malloc(sizeof(double)*(size_3c));
//   double* ima3d = pc_manager.mask_win[sum_pc_i].ima3d;
//   double* ima3d_ = pc_manager.mask_win[sum_pc_i].ima3d_;
//   //vector<make_pair<int, double>> ima3d;
//   //vector<make_pair<int, double>> ima3d_;
//   vector<Mask_pc>& pc_masks = pc_manager.mask_win[sum_pc_i].pc_masks;
//   //ROS_INFO_STREAM("   "<<ima3d[5]);
//   /*
//   for(int i = 0; i<size_3c; i++){
//     *(ima3d + i) = 0;
//     *(ima3d_ + i) = 0;
//   }
//   */
//   //ROS_DEBUG_STREAM("allocate: "<<allocate.toc()<<"ms");
//   double maxima3d[3];

//   maxima3d[0]=0;
//   maxima3d[1]=0;
//   maxima3d[2]=0;

//   double Dx_i;
//   double Dy_i;
//   double Dz_i;

//   int kin = 0;
//   int grid = 5;

  
 
//   int grid_ave = 1;

//   TicToc center_t;
//   m_feature.lock();
//   pc_array_feature.clear();
//   int u0 = feat_point[0];
//   int v0 = feat_point[1];
//   //ROS_INFO_STREAM("u,v = "<<u0<<", "<<v0);
//   int len_in = grid+grid_ave;
//   int len_in2 = len_in*len_in;
//   for (int i = 0; i < pc_masks.size(); i++){
//     int eu = pc_masks[i].point.u_px - u0;
//     int ev = pc_masks[i].point.v_px - v0;
//     int d2 = eu*eu + ev*ev;
//     //ROS_INFO_STREAM("d2="<<d2);
//     if(d2<=len_in2){
//       pc_array_feature.push_back(pc_masks[i].point);
//     }
//   }
//   //   pc_array_feature.clear();
  
//   // for (int u =   (int)(feat_point[0]- grid_ave); u <   (int)(feat_point[0]+ grid_ave); u++)
// 	//       for (int v =   (int)(feat_point[1]- grid_ave); v < (int)(feat_point[1] + grid_ave); v++)
// 	//     	  for (int i_pc =  0; i_pc < pc_array_grid[v*w+u].size(); i_pc++){
// 	//     		  pc_array_feature.push_back(pc_array_grid[v*w+u][i_pc]);
// 	//     	  }
//   ROS_DEBUG_STREAM("array size = "<<pc_array_feature.size());
//   m_feature.unlock();
//   ROS_DEBUG_STREAM("process array : "<<center_t.toc()<<"ms");

//   double Wp_x = 0, Wp_y=0, Wp_z=0, Gr_x=0, Gr_y=0, Gr_z=0, Gs=0;
//   double G_x = 0, G_y = 0, G_z=0;
//   /*
//   TicToc oldone;
//   for (int v=0; v< maxrow - minrow; v=v+1)
//   {
//     for (int u= (int)minmaxuv.umin; u< (int)minmaxuv.umax; u=u+1)
//     {
//       S_x=0; Y_x=0;
//       S_y=0; Y_y=0;
//       S_z=0; Y_z=0;

//       long int gridno = (v+minrow)*w+u;
//       int s_g = pc_array_grid[gridno].size();

      
//       for (int i_g = 0; i_g < s_g; i_g ++){
//         double pu = pc_array_grid[gridno][i_g].u_px;
//         double pv = pc_array_grid[gridno][i_g].v_px;
//         double dx = pc_array_grid[gridno][i_g].x_3d;
//         double dy = abs(pc_array_grid[gridno][i_g].y_3d);
//         double dz = abs(pc_array_grid[gridno][i_g].z_3d);
// //         double dy = pc_array_grid[gridno][i_g].y_3d;
// //         double dz = pc_array_grid[gridno][i_g].z_3d;

//         Gr_x = dx/mr_x;
//         Gr_y = dy/mr_y;
//         Gr_z = dz/mr_z;
//         Gs =  ( (u - pu)*(u - pu) + (v+ minrow-pv)*(v+ minrow-pv) );
//         Wp_x = 1/sqrt(Gs*Gr_x);
//         Wp_y = 1/sqrt(Gs*Gr_y);
//         Wp_z = 1/sqrt(Gs*Gr_z);
//         S_x = S_x + Wp_x;
//         S_y = S_y + Wp_y;
//         S_z = S_z + Wp_z;
//         Y_x = Y_x + Wp_x*dx;
//         Y_y = Y_y + Wp_y*dy;
//         Y_z = Y_z + Wp_z*dz;


//       }

//       pc_array_grid[gridno].clear();
//       //vector <pointcoordinate>().swap(pc_array_grid[gridno]);

//       if (S_x==0) {S_x=1;}
//       if (S_y==0) {S_y=1;}
//       if (S_z==0) {S_z=1;}

//       Dx_i = Y_x/S_x;
//       Dy_i = Y_y/S_y;
//       Dz_i = Y_z/S_z;

//       if(maxima3d[0] < Dx_i ) (maxima3d[0] = Dx_i) ;
//       if(maxima3d[1] < Dy_i ) (maxima3d[1] = Dy_i) ;
//       if(maxima3d[2] < Dz_i ) (maxima3d[2] = Dz_i) ;

//       ima3d[(v+minrow)*image_upsample.cols*c + u*c] = Dx_i;
//       ima3d[(v+minrow)*image_upsample.cols*c + u*c +1] = Dy_i;
//       ima3d[(v+minrow)*image_upsample.cols*c + u*c +2] = Dz_i;
//    }
//   }
//   ROS_DEBUG_STREAM("calculate: "<<oldone.toc()<<"ms");
//   for(int vali = minrow; vali < maxrow; vali++)
//     for(int uali = (int)minmaxuv.umin; uali < (int)minmaxuv.umax; uali++){
//       unsigned char *row_ptr = image_upsample.ptr<unsigned char>(vali);  // row_ptr is the pointer pointing to row vali
//       unsigned char *data_ptr = &row_ptr[uali * image_upsample.channels()]; // data_ptr points to the pixel data to be accessed
//       //notice the order is B,G,R in opencv, and R,G,B in matlab
//       data_ptr[2] = (unsigned char)(255.0*ima3d[vali*image_upsample.cols*c + uali*c]/maxima3d[0]);
//       data_ptr[1] = (unsigned char)(255.0*ima3d[vali*image_upsample.cols*c + uali*c + 1]/maxima3d[1]);
//       data_ptr[0] = (unsigned char)(255.0*ima3d[vali*image_upsample.cols*c + uali*c + 2]/maxima3d[2]);
//     }
//   ROS_DEBUG_STREAM("oldone_total: "<<oldone.toc()<<"ms");

//   free(ima3d);
//   free(ima3d_);
 
//   ima3d = (double*)malloc(sizeof(double)*(image_upsample.rows*image_upsample.cols*c));
//   for(int i = 0; i<size_3c; i++){
//     *(ima3d + i) = 0;
//   }
//   */
 
//   //Wp_x = 0, Wp_y=0, Wp_z=0, Gr_x=0, Gr_y=0, Gr_z=0, Gs=0;
//   //through mask
//   S_x=0; Y_x=0;
//   S_y=0; Y_y=0;
//   S_z=0; Y_z=0;

//   TicToc mask_method;
//   int mask_size =pc_masks.size();    
//   for (int i_g = 0; i_g < mask_size; i_g ++){
//     double pu = pc_masks[i_g].point.u_px;
//     double pv = pc_masks[i_g].point.v_px;
//     double dx = pc_masks[i_g].point.x_3d;
//     double dy = abs(pc_masks[i_g].point.y_3d);
//     double dz = abs(pc_masks[i_g].point.z_3d);
//     //double px_num = pc_masks[i_g].valid_pixles;
//   //         double dy = pc_array_grid[gridno][i_g].y_3d;
//   //         double dz = pc_array_grid[gridno][i_g].z_3d;

//     Gr_x = sqrt(mr_x/dx);
//     Gr_y = sqrt(mr_y/dy);
//     Gr_z = sqrt(mr_z/dz);
//     G_x = Gr_x*dx;
//     G_y = Gr_y*dy;
//     G_z = Gr_z*dz;
//     for(int u =  pc_masks[i_g].u_down; u<pc_masks[i_g].u_up; u++){
//       for(int v = pc_masks[i_g].v_down; v<pc_masks[i_g].v_up; v++){
//         Gs = 1.0/sqrt((u - pu)*(u - pu) + (v-pv)*(v-pv));
//         int index = v*image_upsample.cols*c + u*c;
//         ima3d[index] += Gs*Gr_x;
//         ima3d[index +1] += Gs*Gr_y;
//         ima3d[index +2] += Gs*Gr_z;
//         ima3d_[index] += Gs*G_x;
//         ima3d_[index +1] += Gs*G_y;
//         ima3d_[index +2] += Gs*G_z;
//       }
//     }
//   }
//   pc_masks.clear();
//   if(maxima3d[0] < Dx_i ) (maxima3d[0] = Dx_i) ;
//   if(maxima3d[1] < Dy_i ) (maxima3d[1] = Dy_i) ;
//   if(maxima3d[2] < Dz_i ) (maxima3d[2] = Dz_i) ;
//   ROS_DEBUG_STREAM("mask calculate: "<<mask_method.toc()<<"ms");


//   TicToc comp_max;
//   for(int vali = minrow; vali < maxrow; vali++)
// 	  for(int uali = (int)minmaxuv.umin; uali < (int)minmaxuv.umax; uali++){
//       unsigned char *row_ptr = image_upsample.ptr<unsigned char>(vali);  // row_ptr is the pointer pointing to row vali
//       unsigned char *data_ptr = &row_ptr[uali * image_upsample.channels()]; // data_ptr points to the pixel data to be accessed
//       //notice the order is B,G,R in opencv, and R,G,B in matlab
//       int index = vali*image_upsample.cols*c + uali*c;
//       if (ima3d[index]==0) {ima3d[index]=1;}
//       if (ima3d[index+1]==0) {ima3d[index+1]=1;}
//       if (ima3d[index+2]==0) {ima3d[index+2]=1;}
//       Dx_i = ima3d_[index]/ima3d[index];
//       Dy_i = ima3d_[index + 1]/ima3d[index + 1];
//       Dz_i = ima3d_[index + 2]/ima3d[index + 2];
//       if(maxima3d[0] < Dx_i ) (maxima3d[0] = Dx_i); 
//       if(maxima3d[1] < Dy_i ) (maxima3d[1] = Dy_i) ;
//       if(maxima3d[2] < Dz_i ) (maxima3d[2] = Dz_i) ;
//       //data_ptr[2] = (unsigned char)(255.0*(ima3d_[index]/ima3d[index]));
//       //data_ptr[1] = (unsigned char)(255.0*(ima3d_[index + 1]/ima3d[index + 1]));
//       //data_ptr[0] = (unsigned char)(255.0*(ima3d_[index + 2]/ima3d[index + 2]));
// 	  }
//   ROS_DEBUG_STREAM("comp_max:"<<comp_max.toc()<<"ms");

//   for(int vali = minrow; vali < maxrow; vali++)
// 	  for(int uali = (int)minmaxuv.umin; uali < (int)minmaxuv.umax; uali++){
//       unsigned char *row_ptr = image_upsample.ptr<unsigned char>(vali);  // row_ptr is the pointer pointing to row vali
//       unsigned char *data_ptr = &row_ptr[uali * image_upsample.channels()]; // data_ptr points to the pixel data to be accessed
//       //notice the order is B,G,R in opencv, and R,G,B in matlab
//       int index = vali*image_upsample.cols*c + uali*c;
//       data_ptr[2] = (unsigned char)(255.0*(ima3d_[index]/ima3d[index])/maxima3d[0]);
//       data_ptr[1] = (unsigned char)(255.0*(ima3d_[index + 1]/ima3d[index + 1])/maxima3d[1]);
//       data_ptr[0] = (unsigned char)(255.0*(ima3d_[index + 2]/ima3d[index + 2])/maxima3d[2]);
// 	  }
//   ROS_DEBUG_STREAM("get map: "<<comp_max.toc()<<"ms");
//   ROS_DEBUG_STREAM("mask method: "<<mask_method.toc()<<"ms");
  
//   //free(ima3d);
//   //free(ima3d_);


//   //cv::Mat image_upsample2 = cv::Mat::zeros(h, w, CV_8UC(3)); //initialize the mat variable according to the size of image

//   cv_bridge::CvImage out_msg;
//   out_msg.header   = imgrgb.header; // Same timestamp and tf frame as input image
//   out_msg.encoding = sensor_msgs::image_encodings::BGR8; //gbr8
//   out_msg.image    = image_upsample; // Your cv::Mat

//   pubimg.publish(imgrgb);
//   ROS_DEBUG("rgb_timestamp: %.6fms",imgrgb.header.stamp.toNSec()*(1e-6));
//   pubimg_upsample.publish(out_msg.toImageMsg());
//   //ROS_DEBUG("image published.");

//   /*
//   char pic0[50];

//   sprintf(pic0, "/tmp/%02dimg_2.png", nof);
//   //cv::cvtColor(image_upsample, image_upsample_grey, cv::COLOR_BGR2GRAY);
//   //cv::imshow("image_upsample", image_upsample_grey);
//   //cv::imshow("image_upsample", image_upsample);
//   //cv::imwrite(pic0, image_upsample); //save the image
//   //cv::waitKey(1);
//   cv::Mat channel[3];
//   cv::split(image_upsample, channel);//split into three channels
  
//   char pic1[50];
//   char pic2[50];
//   char pic3[50];
//   sprintf(pic1, "/tmp/%02dupsamplesave_0.png",nof);
//   sprintf(pic2, "/tmp/%02dupsamplesave_1.png",nof);
//   sprintf(pic3, "/tmp/%02dupsamplesave_2.png",nof);
  
//   cv::imshow("x of image_upsample", channel[0]);
//  // cv::imwrite(pic1, channel[0]); //save the image
//   // cv::imshow("y of image_upsample", channel[1]);
//  // cv::imwrite(pic2, channel[1]); //save the image
//   // cv::imshow("z of image_upsample", channel[2]);
//  // cv::imwrite(pic3, channel[2]); //save the image

//   cv::waitKey(1);
//   //cv::destroyAllWindows();
//   */
//   return 0;
// }

//多线程
// int upsampling_pro( pcl::PointXYZ &maxxyz, pcl::PointXYZ &minxyz, minmaxuv_ &minmaxuv, int w, int h, int c, int nof) {
// 	// Use std::chrono to time the algorithm
//   double S_x=0, S_y=0, S_z=0, Y_x=0, Y_y=0, Y_z=0;
//   double mr_x=maxxyz.x, mr_y=maxxyz.y, mr_z=maxxyz.z;

//   int minrow = floor(minmaxuv.vmin) + 1;
//   int maxrow = (int)minmaxuv.vmax +1;
//   //ROS_INFO_STREAM("v(max, min) = "<<maxrow<<", "<<minrow);
//   //ROS_INFO_STREAM("u(max, min) = "<<minmaxuv.umax<<", "<<minmaxuv.umin);
//   cv::Mat image_upsample = cv::Mat(h, w, CV_8UC3, cv::Scalar(0, 0, 0)); //initialize the mat variable according to the size of image
//   int size_3c = image_upsample.rows*image_upsample.cols*c;
//   //TicToc allocate;
//   //double* ima3d = (double*)malloc(sizeof(double)*(size_3c));
//   //double* ima3d_ = (double*)malloc(sizeof(double)*(size_3c));
//   int mask_id = pc_manager.current_id;
//   double* ima3d = pc_manager.mask_win[mask_id].ima3d;
//   double* ima3d_ = pc_manager.mask_win[mask_id].ima3d_;
//   double maxima3d[3];

//   maxima3d[0]=0;
//   maxima3d[1]=0;
//   maxima3d[2]=0;

//   double Dx_i;
//   double Dy_i;
//   double Dz_i;

//   int kin = 0;
//   int grid = 5;

  
 
//   int grid_ave = 1;

//   TicToc center_t;
//   m_feature.lock();
//   pc_array_feature.clear();
//   int u0 = feat_point[0];
//   int v0 = feat_point[1];
//   //ROS_INFO_STREAM("u,v = "<<u0<<", "<<v0);
//   int len_in = grid+grid_ave;
//   int len_in2 = len_in*len_in;
//   for(int index = 0; index<4; index++){
//     vector<Mask_pc>& pc_masks = pc_manager.mask_win[mask_id].pc_masks[index];
//     int mask_size = pc_masks.size();
//     for (int i = 0; i < mask_size; i++){
//       int eu = pc_masks[i].point.u_px - u0;
//       int ev = pc_masks[i].point.v_px - v0;
//       int d2 = eu*eu + ev*ev;
//       //ROS_INFO_STREAM("d2="<<d2);
//       if(d2<=len_in2){
//         pc_array_feature.push_back(pc_masks[i].point);
//       }
//     }
//   }
//   ROS_DEBUG_STREAM("array size = "<<pc_array_feature.size());
//   m_feature.unlock();
//   ROS_DEBUG_STREAM("process array : "<<center_t.toc()<<"ms");

//   double Wp_x = 0, Wp_y=0, Wp_z=0, Gr_x=0, Gr_y=0, Gr_z=0, Gs=0;
//   double G_x = 0, G_y = 0, G_z=0;

//   S_x=0; Y_x=0;
//   S_y=0; Y_y=0;
//   S_z=0; Y_z=0;

//   TicToc mask_method;
//   for(int index = 0; index<4; index++){
//     vector<Mask_pc>& pc_masks = pc_manager.mask_win[mask_id].pc_masks[index];
//     int mask_size = pc_masks.size();
//     for (int i_g = 0; i_g < mask_size; i_g ++){
//       double pu = pc_masks[i_g].point.u_px;
//       double pv = pc_masks[i_g].point.v_px;
//       double dx = pc_masks[i_g].point.x_3d;
//       double dy = abs(pc_masks[i_g].point.y_3d);
//       double dz = abs(pc_masks[i_g].point.z_3d);

//       Gr_x = sqrt(1.0/dx);
//       Gr_y = sqrt(1.0/dy);
//       Gr_z = sqrt(1.0/dz);
//       G_x = Gr_x*dx;
//       G_y = Gr_y*dy;
//       G_z = Gr_z*dz;
//       for(int u =  pc_masks[i_g].u_down; u<pc_masks[i_g].u_up; u++){
//         for(int v = pc_masks[i_g].v_down; v<pc_masks[i_g].v_up; v++){
//           Gs = 1.0/sqrt((u - pu)*(u - pu) + (v-pv)*(v-pv));
//           int index = v*image_upsample.cols*c + u*c;
//           ima3d[index] += Gs*Gr_x;
//           ima3d[index +1] += Gs*Gr_y;
//           ima3d[index +2] += Gs*Gr_z;
//           ima3d_[index] += Gs*G_x;
//           ima3d_[index +1] += Gs*G_y;
//           ima3d_[index +2] += Gs*G_z;
//         }
//       }
//     }
//     pc_masks.clear();
//   }
//   //

//   ROS_DEBUG_STREAM("mask calculate: "<<mask_method.toc()<<"ms");

//   if(pc_manager.init){
//     TicToc comp_max;
//     for(int vali = minrow; vali < maxrow; vali++){
//       for(int uali = (int)minmaxuv.umin; uali < (int)minmaxuv.umax; uali++){
//         unsigned char *row_ptr = image_upsample.ptr<unsigned char>(vali);  // row_ptr is the pointer pointing to row vali
//         unsigned char *data_ptr = &row_ptr[uali * image_upsample.channels()]; // data_ptr points to the pixel data to be accessed
//         //notice the order is B,G,R in opencv, and R,G,B in matlab
//         int index = vali*image_upsample.cols*c + uali*c;
//         double denominator_x = pc_manager.mask_win[0].ima3d[index]+pc_manager.mask_win[1].ima3d[index]+pc_manager.mask_win[2].ima3d[index]+pc_manager.mask_win[3].ima3d[index];
//         double denominator_y = pc_manager.mask_win[0].ima3d[index+1]+pc_manager.mask_win[1].ima3d[index+1]+pc_manager.mask_win[2].ima3d[index+1]+pc_manager.mask_win[3].ima3d[index+1];
//         double denominator_z = pc_manager.mask_win[0].ima3d[index+2]+pc_manager.mask_win[1].ima3d[index+2]+pc_manager.mask_win[2].ima3d[index+2]+pc_manager.mask_win[3].ima3d[index+2];
//         if(denominator_x == 0) denominator_x = 1;
//         if(denominator_y == 0) denominator_y = 1;
//         if(denominator_z == 0) denominator_z = 1;

//         Dx_i = (pc_manager.mask_win[0].ima3d_[index]+pc_manager.mask_win[1].ima3d_[index]+pc_manager.mask_win[2].ima3d_[index]+pc_manager.mask_win[3].ima3d_[index])/denominator_x;
//         Dy_i = (pc_manager.mask_win[0].ima3d_[index+1]+pc_manager.mask_win[1].ima3d_[index+1]+pc_manager.mask_win[2].ima3d_[index+1]+pc_manager.mask_win[3].ima3d_[index+1])/denominator_y;
//         Dz_i = (pc_manager.mask_win[0].ima3d_[index+2]+pc_manager.mask_win[1].ima3d_[index+2]+pc_manager.mask_win[2].ima3d_[index+2]+pc_manager.mask_win[3].ima3d_[index+2])/denominator_z;
//         if(maxima3d[0] < Dx_i ) (maxima3d[0] = Dx_i); 
//         if(maxima3d[1] < Dy_i ) (maxima3d[1] = Dy_i) ;
//         if(maxima3d[2] < Dz_i ) (maxima3d[2] = Dz_i) ;
//       }
//     }

//     ROS_DEBUG_STREAM("comp_max:"<<comp_max.toc()<<"ms");
    
//     for(int vali = minrow; vali < maxrow; vali++)
//       for(int uali = (int)minmaxuv.umin; uali < (int)minmaxuv.umax; uali++){
//         unsigned char *row_ptr = image_upsample.ptr<unsigned char>(vali);  // row_ptr is the pointer pointing to row vali
//         unsigned char *data_ptr = &row_ptr[uali * image_upsample.channels()]; // data_ptr points to the pixel data to be accessed
//         //notice the order is B,G,R in opencv, and R,G,B in matlab
//         int index = vali*image_upsample.cols*c + uali*c;

//         data_ptr[2] = (unsigned char)(255.0*( (pc_manager.mask_win[0].ima3d_[index]+pc_manager.mask_win[1].ima3d_[index]+pc_manager.mask_win[2].ima3d_[index]+pc_manager.mask_win[3].ima3d_[index])/(pc_manager.mask_win[0].ima3d[index]+pc_manager.mask_win[1].ima3d[index]+pc_manager.mask_win[2].ima3d[index]+pc_manager.mask_win[3].ima3d[index]))/maxima3d[0]);
//         data_ptr[1] = (unsigned char)(255.0*( (pc_manager.mask_win[0].ima3d_[index+1]+pc_manager.mask_win[1].ima3d_[index+1]+pc_manager.mask_win[2].ima3d_[index+1]+pc_manager.mask_win[3].ima3d_[index+1])/(pc_manager.mask_win[0].ima3d[index+1]+pc_manager.mask_win[1].ima3d[index+1]+pc_manager.mask_win[2].ima3d[index+1]+pc_manager.mask_win[3].ima3d[index+1]))/maxima3d[1]);
//         data_ptr[0] = (unsigned char)(255.0*( (pc_manager.mask_win[0].ima3d_[index+2]+pc_manager.mask_win[1].ima3d_[index+2]+pc_manager.mask_win[2].ima3d_[index+2]+pc_manager.mask_win[3].ima3d_[index+2])/(pc_manager.mask_win[0].ima3d[index+2]+pc_manager.mask_win[1].ima3d[index+2]+pc_manager.mask_win[2].ima3d[index+2]+pc_manager.mask_win[3].ima3d[index+2]))/maxima3d[2]);

    
//     }
    
//     ROS_DEBUG_STREAM("get map: "<<comp_max.toc()<<"ms");
//     ROS_DEBUG_STREAM("mask method: "<<mask_method.toc()<<"ms");

//     //cv::Mat image_upsample2 = cv::Mat::zeros(h, w, CV_8UC(3)); //initialize the mat variable according to the size of image

//     cv_bridge::CvImage out_msg;
//     out_msg.header   = imgrgb.header; // Same timestamp and tf frame as input image
//     out_msg.encoding = sensor_msgs::image_encodings::BGR8; //gbr8
//     out_msg.image    = image_upsample; // Your cv::Mat

//     pubimg.publish(imgrgb);
//     //ROS_DEBUG("rgb_timestamp: %.6fms",imgrgb.header.stamp.toNSec()*(1e-6));
//     pubimg_upsample.publish(out_msg.toImageMsg());
//     //ROS_DEBUG("image published.");
//   }
//   ROS_DEBUG_STREAM("processing and pub 0.4s data: "<<all_time.toc()<<"ms");

//   char pic0[50];

//   sprintf(pic0, "/tmp/%02dimg_2.png", nof);
//   //cv::cvtColor(image_upsample, image_upsample_grey, cv::COLOR_BGR2GRAY);
//   //cv::imshow("image_upsample", image_upsample_grey);
//   //cv::imshow("image_upsample", image_upsample);
//   //cv::imwrite(pic0, image_upsample); //save the image
//   //cv::waitKey(1);
//   cv::Mat channel[3];
//   cv::split(image_upsample, channel);//split into three channels
  
//   char pic1[50];
//   char pic2[50];
//   char pic3[50];
//   sprintf(pic1, "/tmp/%02dupsamplesave_0.png",nof);
//   sprintf(pic2, "/tmp/%02dupsamplesave_1.png",nof);
//   sprintf(pic3, "/tmp/%02dupsamplesave_2.png",nof);
  
//   cv::imshow("x of image_upsample", channel[0]);
//  // cv::imwrite(pic1, channel[0]); //save the image
//   // cv::imshow("y of image_upsample", channel[1]);
//  // cv::imwrite(pic2, channel[1]); //save the image
//   // cv::imshow("z of image_upsample", channel[2]);
//  // cv::imwrite(pic3, channel[2]); //save the image

//   cv::waitKey(1);
//   //cv::destroyAllWindows();
  
//   return 0;
// }




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
  
  int grid_ave = 1;
  if(grid_z !=5){
    search_box_yolo = grid_z;
  }
  vector<Vector3d> pc_array_yolo;

  //depth picture
  int lenth_of_vector = (depth_threshold_up-depth_threshold_down)/perception + 5;
  // ROS_INFO_STREAM("lenth_of_vector = "<<lenth_of_vector);
  x_depth.clear();
  // vector<int> x_label;
 
  for(int i = 0;i<lenth_of_vector;i++){
      x_depth.emplace_back(Vector4d::Zero());
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
      pc_array_yolo.push_back(in_box);
      if(array_pc[i].x()<depth_threshold_up && array_pc[i].x()>depth_threshold_down)//lidar frame
      {
        int index = int((array_pc[i].x() - depth_threshold_down)/perception)+1;
        x_depth[index].x() += in_box.x();
        x_depth[index].y() += in_box.y();
        x_depth[index].z() += in_box.z();
        x_depth[index].w() ++ ;
      }
        

    }
  }
  
  //visualization
  for(int i=0;i<x_depth.size();i++){
    if(x_depth[i].w()<10) {
      x_depth[i].w() = 0;
      x_depth[i].x() = 0;
      x_depth[i].y() = 0;
      x_depth[i].z() = 0;
      // cout <<" "<<" ";
    }else{
      // cout <<x_depth[i]<<" ";
    }
  }
  // cout<<endl;
  return;
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
  int size_3c = image_upsample.rows*image_upsample.cols*c;
  //TicToc allocate;
  // double* ima3d = (double*)malloc(sizeof(double)*(size_3c));
  // double* ima3d_ = (double*)malloc(sizeof(double)*(size_3c));
  int mask_id = pc_manager.current_id;
  
  //ROS_INFO_STREAM("id :"<<mask_id);
  //vector<make_pair<int, double>> ima3d;
  //vector<make_pair<int, double>> ima3d_;
  vector<Mask_pc>& pc_masks_no_rect = pc_manager.mask_win[mask_id].pc_masks_single;
  vector<Mask_pc>& pc_masks = pc_manager.mask4.pc_masks_single;
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

  

  /*collect points in the target box for depth estimating in ekf part*/
  
  // if(flag_int_xkk_last == 1){
  //   TicToc center_t;
  //   // search center is read from feat_point
  //   Vector3d Yolo_p;
  //   ROS_INFO_STREAM("(u, v) = ("<<u0<<", "<<v0 );
  //   Yolo_p = calculate_yolo_depth(yolo_depth.back().pc_lidar_3d, yolo_depth.back().pc_uv, u0, v0, 30);
    
  //   //pc ekf update
  //   PC_EKF_uodate(Yolo_p);
    
  //   ROS_DEBUG_STREAM("process array : "<<center_t.toc()<<"ms");
  // } 
  

  

 
  //Wp_x = 0, Wp_y=0, Wp_z=0, Gr_x=0, Gr_y=0, Gr_z=0, Gs=0;
  //through mask
  S_x=0; Y_x=0;
  S_y=0; Y_y=0;
  S_z=0; Y_z=0;

  TicToc mask_method;//计算消耗主要在计算部分
  /*through masks and save numerator and denominator in image-sized map */
  double* ima3d = pc_manager.mask4.ima3d;
  double* ima3d_ = pc_manager.mask4.ima3d_;
  //#pragma omp parallel for 
  //ROS_INFO_STREAM("pc_manager.mask4.pc_masks_single"<<pc_masks[0].point.G_x);
  for (int i_g = 0; i_g < mask_size; i_g ++){
  
    double Gr_x=pc_masks[i_g].point.Gr_x;
    double Gr_y=pc_masks[i_g].point.Gr_y;
    double Gr_z=pc_masks[i_g].point.Gr_z;
    double Gs=0;
    double G_x = pc_masks[i_g].point.G_x;
    double G_y = pc_masks[i_g].point.G_y;
    double G_z = pc_masks[i_g].point.G_z;

    double pu = pc_masks[i_g].point.u_px;
    double pv = pc_masks[i_g].point.v_px;
    // double dx = abs(pc_masks[i_g].point.x_3d);
    // double dy = abs(pc_masks[i_g].point.y_3d);
    // double dz = abs(pc_masks[i_g].point.z_3d);
    // if(dy<0.01){
    //   dy = 0.01;
    //   //ROS_INFO_STREAM("dy = "<<dy);
    // }
    // if(dx<0.01){
    //   dx = 0.01;
    //   //ROS_INFO_STREAM("dy = "<<dy);
    // }
    // Gr_x = 1.0/sqrt(dx);
    // Gr_y = 1.0/sqrt(dy);
    // Gr_z = 1.0/sqrt(dz);
    // G_x = sqrt(dx);
    // G_y = sqrt(dy);
    // G_z = sqrt(dz);
    // // if(pc_masks[i_g].point.y_3d < 0){
    // //   G_y = -G_y;
    // // }
    // // if(pc_masks[i_g].point.z_3d < 0){
    // //   G_z = -G_z;
    // // }
    double d2;
    for(int u =  pc_masks[i_g].u_down; u<pc_masks[i_g].u_up; u++){
      for(int v = pc_masks[i_g].v_down; v<pc_masks[i_g].v_up; v++){
        d2 = sqrt((u - pu)*(u - pu) + (v-pv)*(v-pv));
        if(d2<0.01){
          //ROS_INFO_STREAM("d2 = "<<d2);
          d2 = 0.01;
          
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
        ima3d[index] += x0;
        ima3d[index +1] += x1;
        ima3d[index +2] += x2;
        ima3d_[index] += y0;
        ima3d_[index +1] += y1;
        ima3d_[index +2] += y2;
        //m_thread.unlock();
      }
    }
  }
  ROS_DEBUG_STREAM("mask calculate: "<<mask_method.toc()<<"ms");
  pc_masks.clear();
  if(compare_rect){
    double* ima3d = pc_manager.mask_win[mask_id].ima3d;
    double* ima3d_ = pc_manager.mask_win[mask_id].ima3d_;
    int mask_size_no_rect = pc_masks_no_rect.size();
    ROS_INFO_STREAM("mask size = "<<mask_size_no_rect);
    for (int i_g = 0; i_g < mask_size_no_rect; i_g ++){
      double Gr_x=0, Gr_y=0, Gr_z=0, Gs=0;
      double G_x = 0, G_y = 0, G_z=0;
      double pu = pc_masks_no_rect[i_g].point.u_px;
      double pv = pc_masks_no_rect[i_g].point.v_px;
      double dx = abs(pc_masks_no_rect[i_g].point.x_3d);
      double dy = abs(pc_masks_no_rect[i_g].point.y_3d);
      double dz = abs(pc_masks_no_rect[i_g].point.z_3d);

      Gr_x = 1.0/sqrt(dx);
      Gr_y = 1.0/sqrt(dy);
      Gr_z = 1.0/sqrt(dz);
      G_x = Gr_x*dx;
      G_y = Gr_y*dy;
      G_z = Gr_z*dz;
      // if(pc_masks[i_g].point.y_3d < 0){
      //   G_y = -G_y;
      // }
      // if(pc_masks[i_g].point.z_3d < 0){
      //   G_z = -G_z;
      // }
      //ROS_INFO("what happened???");
      for(int u =  pc_masks_no_rect[i_g].u_down; u<pc_masks_no_rect[i_g].u_up; u++){
        for(int v = pc_masks_no_rect[i_g].v_down; v<pc_masks_no_rect[i_g].v_up; v++){
          Gs = 1.0/sqrt((u - pu)*(u - pu) + (v-pv)*(v-pv));
          int index = v*image_upsample.cols*c + u*c;
          double x0 = Gs*Gr_x;
          double x1 = Gs*Gr_y;
          double x2 = Gs*Gr_z;
          double y0 = Gs*G_x;
          double y1 = Gs*G_y;
          double y2 = Gs*G_z;
          //m_thread.lock();
          ima3d[index] += x0;
          ima3d[index +1] += x1;
          ima3d[index +2] += x2;
          ima3d_[index] += y0;
          ima3d_[index +1] += y1;
          ima3d_[index +2] += y2;
          //m_thread.unlock();
        }
      }
    }
    pc_masks_no_rect.clear();
  }
  //ROS_INFO("what happened???");

  /*save the max denominator in all 4 serials data*/
  if(pc_manager.init){
    
    if(compare_rect){
      double* ima3d = pc_manager.mask_win[mask_id].ima3d;
      double* ima3d_ = pc_manager.mask_win[mask_id].ima3d_;
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
          double denominator_x = pc_manager.mask_win[0].ima3d[index]+pc_manager.mask_win[1].ima3d[index]+pc_manager.mask_win[2].ima3d[index]+pc_manager.mask_win[3].ima3d[index];
          double denominator_y = pc_manager.mask_win[0].ima3d[index+1]+pc_manager.mask_win[1].ima3d[index+1]+pc_manager.mask_win[2].ima3d[index+1]+pc_manager.mask_win[3].ima3d[index+1];
          double denominator_z = pc_manager.mask_win[0].ima3d[index+2]+pc_manager.mask_win[1].ima3d[index+2]+pc_manager.mask_win[2].ima3d[index+2]+pc_manager.mask_win[3].ima3d[index+2];
          if(denominator_x == 0) denominator_x = 1;
          if(denominator_y == 0) denominator_y = 1;
          if(denominator_z == 0) denominator_z = 1;
          // if (ima3d[index]==0) {ima3d[index]=1;}
          // if (ima3d[index+1]==0) {ima3d[index+1]=1;}
          // if (ima3d[index+2]==0) {ima3d[index+2]=1;}
          // Dx_i = ima3d_[index]/ima3d[index];
          // Dy_i = ima3d_[index + 1]/ima3d[index + 1];
          // Dz_i = ima3d_[index + 2]/ima3d[index + 2];

          /*calculating depth map(befor normalized) and save the max one*/
          Dx_i = (pc_manager.mask_win[0].ima3d_[index]+pc_manager.mask_win[1].ima3d_[index]+pc_manager.mask_win[2].ima3d_[index]+pc_manager.mask_win[3].ima3d_[index])/denominator_x;
          Dy_i = (pc_manager.mask_win[0].ima3d_[index+1]+pc_manager.mask_win[1].ima3d_[index+1]+pc_manager.mask_win[2].ima3d_[index+1]+pc_manager.mask_win[3].ima3d_[index+1])/denominator_y;
          Dz_i = (pc_manager.mask_win[0].ima3d_[index+2]+pc_manager.mask_win[1].ima3d_[index+2]+pc_manager.mask_win[2].ima3d_[index+2]+pc_manager.mask_win[3].ima3d_[index+2])/denominator_z;
          if(maxima3d_no_rect[0] < Dx_i ) (maxima3d_no_rect[0] = Dx_i); 
          if(maxima3d_no_rect[1] < Dy_i ) (maxima3d_no_rect[1] = Dy_i) ;
          if(maxima3d_no_rect[2] < Dz_i ) (maxima3d_no_rect[2] = Dz_i) ;
        }
      }
      for(int vali = minrow; vali < maxrow; vali++)
        for(int uali = (int)minmaxuv.umin; uali < (int)minmaxuv.umax; uali++){
          unsigned char *row_ptr = image_upsample_no_rect.ptr<unsigned char>(vali);  // row_ptr is the pointer pointing to row vali
          unsigned char *data_ptr = &row_ptr[uali * image_upsample.channels()]; // data_ptr points to the pixel data to be accessed
          //notice the order is B,G,R in opencv, and R,G,B in matlab
          int index = vali*image_upsample.cols*c + uali*c;

          data_ptr[2] = (unsigned char)(255.0*( (pc_manager.mask_win[0].ima3d_[index]+pc_manager.mask_win[1].ima3d_[index]+pc_manager.mask_win[2].ima3d_[index]+pc_manager.mask_win[3].ima3d_[index])/(pc_manager.mask_win[0].ima3d[index]+pc_manager.mask_win[1].ima3d[index]+pc_manager.mask_win[2].ima3d[index]+pc_manager.mask_win[3].ima3d[index]))/maxima3d_no_rect[0]);
          data_ptr[1] = (unsigned char)(255.0*( (pc_manager.mask_win[0].ima3d_[index+1]+pc_manager.mask_win[1].ima3d_[index+1]+pc_manager.mask_win[2].ima3d_[index+1]+pc_manager.mask_win[3].ima3d_[index+1])/(pc_manager.mask_win[0].ima3d[index+1]+pc_manager.mask_win[1].ima3d[index+1]+pc_manager.mask_win[2].ima3d[index+1]+pc_manager.mask_win[3].ima3d[index+1]))/maxima3d_no_rect[1]);
          data_ptr[0] = (unsigned char)(255.0*( (pc_manager.mask_win[0].ima3d_[index+2]+pc_manager.mask_win[1].ima3d_[index+2]+pc_manager.mask_win[2].ima3d_[index+2]+pc_manager.mask_win[3].ima3d_[index+2])/(pc_manager.mask_win[0].ima3d[index+2]+pc_manager.mask_win[1].ima3d[index+2]+pc_manager.mask_win[2].ima3d[index+2]+pc_manager.mask_win[3].ima3d[index+2]))/maxima3d_no_rect[2]);
 
      }
    }

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
        
        double denominator_x = pc_manager.mask4.ima3d[index];
        double denominator_y = pc_manager.mask4.ima3d[index+1];
        double denominator_z = pc_manager.mask4.ima3d[index+2];
        if(denominator_x == 0) denominator_x = 1;
        if(denominator_y == 0) denominator_y = 1;
        if(denominator_z == 0) denominator_z = 1;
        if(pc_manager.mask4.ima3d_[index] == 0){
          
          continue;
        }
        /*calculating depth map(befor normalized) and save the max one*/
        Dx_i = (pc_manager.mask4.ima3d_[index])/denominator_x;
        Dy_i = (pc_manager.mask4.ima3d_[index+1])/denominator_y;
        Dz_i = (pc_manager.mask4.ima3d_[index+2])/denominator_z;
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
      //   data_ptr[2] = (unsigned char)(255.0*(ima3d_[index]/ima3d[index])/maxima3d[0]);
      //  data_ptr[1] = (unsigned char)(255.0*(ima3d_[index + 1]/ima3d[index + 1])/maxima3d[1]);
      //  data_ptr[0] = (unsigned char)(255.0*(ima3d_[index + 2]/ima3d[index + 2])/maxima3d[2]);
      /*get the final nomalized depth map */
      //2 1 0
        data_ptr[2] = (unsigned char)(255.0*( (pc_manager.mask4.ima3d_[index])/(pc_manager.mask4.ima3d[index]))/ maxima3d[0]);
        data_ptr[0] = (unsigned char)(255.0*( (pc_manager.mask4.ima3d_[index+1])/(pc_manager.mask4.ima3d[index+1]))/maxima3d[1]);
        data_ptr[1] = (unsigned char)(255.0*( (pc_manager.mask4.ima3d_[index+2])/(pc_manager.mask4.ima3d[index+2]))/maxima3d[2]);
 
    }
    
    ROS_DEBUG_STREAM("get map: "<<comp_max.toc()<<"ms");
    ROS_DEBUG_STREAM("mask method: "<<mask_method.toc()<<"ms");
  

    //cv::Mat image_upsample2 = cv::Mat::zeros(h, w, CV_8UC(3)); //initialize the mat variable according to the size of image

    cv_bridge::CvImage out_msg;
    out_msg.header   = imgrgb_cur.header; // Same timestamp and tf frame as input image
    // ROS_INFO_STREAM("timestamp_img = "<<out_msg.header.stamp.toNSec());
    out_msg.encoding = sensor_msgs::image_encodings::BGR8; //gbr8
    out_msg.image    = image_upsample; // Your cv::Mat
    // cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
    // cv::Mat img_normalized, img_cur_split[3];
    // vector<cv::Mat> img_normalized_split(3);
    // cv::split(img_cur, img_cur_split);
    // for(int i = 0; i<3; i++){
    //   clahe->apply(img_cur_split[i], img_normalized_split[i]);
    // }
    // cv::merge(img_normalized_split, img_normalized);
    pubimg.publish(imgrgb_cur);
    pubimg_upsample.publish(out_msg.toImageMsg());
    // pubimg.publish(out_msg.toImageMsg());
    // pubimg_upsample.publish(imgrgb_cur);






    //ROS_DEBUG("image published.");
    ROS_DEBUG_STREAM("processing and pub 0.4s data: "<<all_time.toc()<<"ms");
    // search_box_yolo
    // if(ifdetection==1){
      cv::circle(image_upsample, circle_center, search_box_yolo, cv::Scalar(0, 255, 0));
      cv::circle(img_cur, circle_center, search_box_yolo, cv::Scalar(0, 255, 0));
      cv::circle(image_upsample, rect_circle_center, search_box_yolo, cv::Scalar(0, 0, 255));
      cv::circle(img_cur, rect_circle_center, search_box_yolo, cv::Scalar(0, 0, 255));
      // ROS_INFO_STREAM("box_grid_points.size() = "<<box_grid_points.size());
      // for(int i = 0;i < box_grid_points.size();i++){
      //   cv::circle(img_cur, box_grid_points[i], 1, cv::Scalar(0, 255, 0));
      // }
      box_grid_points.clear();
      // ifdetection = 0;
    // }
    
    cv::imshow("image_upsample", image_upsample);
    cv::imshow("image", img_cur);
    // cv::imshow("img_normalized", img_normalized);
    cv::Mat channel[3];
    cv::split(image_upsample, channel);//split into three channels
    
    // char pic1[50];
    // char pic2[50];
    // char pic3[50];
    // sprintf(pic1, "/tmp/%02dupsamplesave_0.png",nof);
    // sprintf(pic2, "/tmp/%02dupsamplesave_1.png",nof);
    // sprintf(pic3, "/tmp/%02dupsamplesave_2.png",nof);
    
    // cv::imshow("x of image_upsample", channel[0]);
    // cv::imshow("y of image_upsample", channel[1]);
    // cv::imshow("z of image_upsample", channel[2]);
    if(compare_rect){
      cv::imshow("image_upsample_no_rect", image_upsample_no_rect);
      
    }

    cv::waitKey(1);
    if(image_save){
      double hz = 0;
      cur_setfre = imgrgb_cur.header.stamp.toSec();
      if(last_setfre == 0){
        last_setfre = cur_setfre;
        ROS_INFO("return");
        return 0;
      }
      hz = 1.0/(cur_setfre - last_setfre);
      ROS_DEBUG_STREAM("hz = "<<hz);
      if(hz<0.4 && hz>0){
        last_setfre = cur_setfre;
        ROS_DEBUG_STREAM("save iamge");
        
        char png_name_upsampling[100];
        char png_name[100];
        sprintf(png_name_upsampling, "/media/mao/PortableSSD/mpc_dataset/image/%05dimg_2.png", i_pc_count);
        sprintf(png_name, "/media/mao/PortableSSD/mpc_dataset/image/%05dimg.png", i_pc_count);
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
        cv::Mat channel[3];
        cv::split(image_upsample, channel);//split into three channels
        
        // char pic1[50];
        // char pic2[50];
        // char pic3[50];
        // sprintf(pic1, "/tmp/%02dupsamplesave_0.png",nof);
        // sprintf(pic2, "/tmp/%02dupsamplesave_1.png",nof);
        // sprintf(pic3, "/tmp/%02dupsamplesave_2.png",nof);
        
        cv::imshow("x of image_upsample", channel[0]);
        //cv::imwrite(pic1, channel[0]); //save the image
        cv::imshow("y of image_upsample", channel[1]);
        //cv::imwrite(pic2, channel[1]); //save the image
        cv::imshow("z of image_upsample", channel[2]);
        //cv::imwrite(pic3, channel[2]); //save the image
        //cv::destroyAllWindows();
      }
      
    }
    
    }else{
      cur_setfre = 0;
      last_setfre = 0;
      ROS_INFO("Reset setfre value.");
    }
  return 0;
}


bool compare_pc_v(const pointcoordinate& left,const pointcoordinate& right){
    return left.v_px<right.v_px; //ascending sort
}




void PC_Wrapper::UpdateMask(int id){
  current_id = id;
  mask_win[id].mask_id = id;
  mask_win[id].SetZeros();
  mask4.SetZeros();
}

void PC_Wrapper::UpdateMax3d(){
  
  for(int i = 0; i < WINDOW_SIZE; i++){
    if(Maxima3d[0] < mask_win[i].maxima3d[0]) Maxima3d[0] = mask_win[i].maxima3d[0];
    if(Maxima3d[1] < mask_win[i].maxima3d[1]) Maxima3d[1] = mask_win[i].maxima3d[1];
    if(Maxima3d[2] < mask_win[i].maxima3d[2]) Maxima3d[2] = mask_win[i].maxima3d[2];
  }
  
}


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