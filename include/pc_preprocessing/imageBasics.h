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

class pointcoordinate
{
public:
    double x_3d{0.0};
    double y_3d{0.0};
    double z_3d{0.0};
    double u_px{0.0};
    double v_px{0.0};

    void print()
    {
        //std::cout << "3D coordinate of this point: [" << x_3d << "; " << y_3d << "; " << z_3d <<  "]" << std::endl;
        //std::cout << "2D coordinate of this point in pixel frame: [" << u_px << "; " << v_px <<  "]" << std::endl;
        
    
    
    }
};

bool compare_pc_v(const pointcoordinate& left,const pointcoordinate& right){
    return left.v_px<right.v_px; //ascending sort
}

#endif 

