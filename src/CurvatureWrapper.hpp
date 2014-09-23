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

#ifndef _CURVATURE_WRAPPER_HPP_
	#define _CURVATURE_WRAPPER_HPP_

	#include "LocalFeature.hpp"

	class CurvatureWrapper : public LocalFeature
	{
		public:
			CurvatureWrapper(double radiusNormal=0.03, double radiusCurvature=1.0)
			{
				this->radiusNormal = radiusNormal;
				this->radiusCurvature = radiusCurvature;
			}

			void computeFeature(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, vector< vector<double> >& features);
		
			void computeFeature(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,  pcl::PointCloud<pcl::Normal>::Ptr& normals, vector< vector<double> >& features);
			

		private:
			double radiusNormal;
			double radiusCurvature;

			pcl::PrincipalCurvaturesEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::PrincipalCurvatures> principal_curvatures_estimation;
	};
#endif
