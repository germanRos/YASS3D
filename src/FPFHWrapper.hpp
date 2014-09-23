/** Copyright 2013. All rights reserved.
* Author: German Ros (gros@cvc.uab.es)
*         Advanced Driver Assistance Systems (ADAS)
*         Computer Vision Center (CVC)
*	  Universitat Autònoma de Barcelona (UAB)
*
* This is free software; you can redistribute it and/or modify it under the
* terms of the GNU General Public License as published by the Free Software
* Foundation; either version 3 of the License, or any later version.
*
* This software is distributed in the hope that it will be useful, but WITHOUT ANY
* WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
* PARTICULAR PURPOSE. See the GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with
* this software; if not, write to the Free Software Foundation, Inc., 51 Franklin
* Street, Fifth Floor, Boston, MA 02110-1301, USA 
*
*/

#ifndef _FPFH_WRAPPER_HPP_
	#define _FPFH_WRAPPER_HPP_

	#include "LocalFeature.hpp"
	#include <pcl/features/fpfh.h>
	#include <pcl/features/normal_3d.h>

	class FPFHWrapper : public LocalFeature
	{
		public:
			FPFHWrapper(double radiusNormal=0.1, double radiusFPFH = 0.25)
			{
				this->radiusNormal = radiusNormal;
				this->radiusFPFH = radiusFPFH;
			}

			void computeFeature(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, vector< vector<double> >& features);

			void computeFeature(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,  pcl::PointCloud<pcl::Normal>::Ptr& normals, vector< vector<double> >& features);
			

		private:
			double radiusNormal;
			double radiusFPFH;

			pcl::FPFHEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::FPFHSignature33> fpfh_estimation;

	};
#endif
