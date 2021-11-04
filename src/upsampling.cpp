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
extern vector<pointcoordinate> pc_array_grid[921600];

int upsampling_pro( pcl::PointXYZ &maxxyz, pcl::PointXYZ &minxyz, minmaxuv_ &minmaxuv, int w, int h, int c, int nof) {
	// Use std::chrono to time the algorithm
  chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
  double Wp_x, Wp_y, Wp_z, Gr_x, Gr_y, Gr_z, Gs, S_x=0, S_y=0, S_z=0, Y_x=0, Y_y=0, Y_z=0;
  double mr_x=maxxyz.x, mr_y=maxxyz.y, mr_z=maxxyz.z;

  /*cout << "minxyz, x:  " << minxyz.x << endl;
  cout << "minxyz, y:  " << minxyz.y << endl;
  cout << "minxyz, z:  " << minxyz.z << endl;*/

  //sort(pc_array.begin(),pc_array.end(),compare_pc_v); //time consuming

  //int minrow = floor(pc_array[0].v_px);  //the minimum v coordinate of the points
  int minrow = floor(minmaxuv.vmin) + 1;
  //cout << "minrow:" << minrow << endl;
  //cout << "size of pc: " << pc_array.size() << endl;

  //int maxrow = floor(pc_array.back().v_px) +1;  //the minimum v coordinate of the points
  int maxrow = (int)minmaxuv.vmax +1;
  //cout << "maxrow:" << maxrow << endl;

  //cv::Mat image_upsample  = image.clone();//clone original image used for upsampling
  cv::Mat image_upsample = cv::Mat(h, w, CV_8UC3, cv::Scalar(0, 0, 0)); //initialize the mat variable according to the size of image

  double* ima3d = (double*)malloc(sizeof(double)*(image_upsample.rows*image_upsample.cols*c));

  double maxima3d[3];
  //= {0.0f,0.0f,0.0f};
  maxima3d[0]=0;
  maxima3d[1]=0;
  maxima3d[2]=0;

  double Dx_i;
  double Dy_i;
  double Dz_i;

  int kin = 0;
  int grid = 5;
  //int sd = pc_array.size();

  for (int v=0; v< maxrow - minrow; v=v+1)
  {
   for (int u= (int)minmaxuv.umin; u< (int)minmaxuv.umax; u=u+1)
   {
	   S_x=0; Y_x=0;
	   S_y=0; Y_y=0;
	   S_z=0; Y_z=0;

	   long int gridno = (v+minrow)*w+u;
	   int s_g = pc_array_grid[gridno].size();
	   for (int i_g = 0; i_g < s_g; i_g ++){
         double pu = pc_array_grid[gridno][i_g].u_px;
         double pv = pc_array_grid[gridno][i_g].v_px;
         double dx = pc_array_grid[gridno][i_g].x_3d;
//         double dy = pc_array_grid[gridno][i_g].y_3d - minxyz.y;
//         double dz = pc_array_grid[gridno][i_g].z_3d - minxyz.z;
         double dy = pc_array_grid[gridno][i_g].y_3d;
         double dz = pc_array_grid[gridno][i_g].z_3d;

         Gr_x = dx/mr_x;
         Gr_y = dy/mr_y;
         Gr_z = dz/mr_z;
         Gs =  ( (u - pu)*(u - pu) + (v+ minrow-pv)*(v+ minrow-pv) );
         Wp_x = 1/sqrt(Gs*Gr_x);
         Wp_y = 1/sqrt(Gs*Gr_y);
         Wp_z = 1/sqrt(Gs*Gr_z);
         S_x = S_x + Wp_x;
         S_y = S_y + Wp_y;
         S_z = S_z + Wp_z;
         Y_x = Y_x + Wp_x*dx;
         Y_y = Y_y + Wp_y*dy;
         Y_z = Y_z + Wp_z*dz;
	  }
	  pc_array_grid[gridno].clear();

      if (S_x==0) {S_x=1;}
      if (S_y==0) {S_y=1;}
      if (S_z==0) {S_z=1;}

      Dx_i = Y_x/S_x;
      Dy_i = Y_y/S_y;
      Dz_i = Y_z/S_z;

      if(maxima3d[0] < Dx_i ) (maxima3d[0] = Dx_i) ;
      if(maxima3d[1] < Dy_i ) (maxima3d[1] = Dy_i) ;
      if(maxima3d[2] < Dz_i ) (maxima3d[2] = Dz_i) ;

      ima3d[(v+minrow)*image_upsample.cols*c + u*c] = Dx_i;
	  ima3d[(v+minrow)*image_upsample.cols*c + u*c +1] = Dy_i;
	  ima3d[(v+minrow)*image_upsample.cols*c + u*c +2] = Dz_i;
   }
  }

  for(int vali = minrow; vali < maxrow; vali++)
	  for(int uali = (int)minmaxuv.umin; uali < (int)minmaxuv.umax; uali++){
          unsigned char *row_ptr = image_upsample.ptr<unsigned char>(vali);  // row_ptr is the pointer pointing to row vali
          unsigned char *data_ptr = &row_ptr[uali * image_upsample.channels()]; // data_ptr points to the pixel data to be accessed
          //notice the order is B,G,R in opencv, and R,G,B in matlab
          data_ptr[2] = (unsigned char)(255.0*ima3d[vali*image_upsample.cols*c + uali*c]/maxima3d[0]);
          data_ptr[1] = (unsigned char)(255.0*ima3d[vali*image_upsample.cols*c + uali*c + 1]/maxima3d[1]);
          data_ptr[0] = (unsigned char)(255.0*ima3d[vali*image_upsample.cols*c + uali*c + 2]/maxima3d[2]);
	  }

  free(ima3d);

  chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
  chrono::duration<double> time_used = chrono::duration_cast < chrono::duration < double >> (t2 - t1);
  cout << "Total time in upsampling: " << time_used.count() << " s." << endl;

  char pic0[50];

  sprintf(pic0, "/tmp/%02dimage_upsample.png", nof);
  //cv::cvtColor(image_upsample, image_upsample_grey, cv::COLOR_BGR2GRAY);
  //cv::imshow("image_upsample", image_upsample_grey);
  cv::imshow("image_upsample", image_upsample);
  cv::imwrite(pic0, image_upsample); //save the image
  cv::waitKey(1);
  cv::Mat channel[3];
  cv::split(image_upsample, channel);//split into three channels

  char pic1[50];
  char pic2[50];
  char pic3[50];
  sprintf(pic1, "/tmp/%02dupsamplesave_0.png",nof);
  sprintf(pic2, "/tmp/%02dupsamplesave_1.png",nof);
  sprintf(pic3, "/tmp/%02dupsamplesave_2.png",nof);
  // cv::imshow("x of image_upsample", channel[0]);
  cv::imwrite(pic1, channel[0]); //save the image
  // cv::imshow("y of image_upsample", channel[1]);
  cv::imwrite(pic2, channel[1]); //save the image
  // cv::imshow("z of image_upsample", channel[2]);
  cv::imwrite(pic3, channel[2]); //save the image

  // cv::waitKey(1);
  cv::destroyAllWindows();
  return 0;
}


bool compare_pc_v(const pointcoordinate& left,const pointcoordinate& right){
    return left.v_px<right.v_px; //ascending sort
}

