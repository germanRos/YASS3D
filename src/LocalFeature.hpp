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

#ifndef _LOCAL_FEATURE_HPP_
	#define _LOCAL_FEATURE_HPP_

	#include <iostream>
	#include <vector>
	#include <pcl/io/pcd_io.h>
	#include <pcl/point_cloud.h>
	#include <pcl/point_types.h>
	#include <pcl/features/normal_3d.h>
	#include <pcl/features/principal_curvatures.h>

	using namespace std;
	using namespace pcl;

	class LocalFeature
	{
		public:
			virtual void computeFeature(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, vector< vector<double> >& features) = 0;
		
			virtual void computeFeature(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,  pcl::PointCloud<pcl::Normal>::Ptr& normals, vector< vector<double> >& features) = 0;

			void computeNormals(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, double radius, pcl::PointCloud<pcl::Normal>::Ptr& normals, pcl::search::KdTree<pcl::PointXYZRGB>::Ptr& tree)
			{

  				// Compute the normals
  				pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> normalEstimation;
  				normalEstimation.setInputCloud(cloud);
				normalEstimation.setSearchMethod(tree);
				normalEstimation.setRadiusSearch(radius);
				normalEstimation.compute(*normals);

				//for(int i=0; i<normals->size(); i++)
				//{
				//	pcl::Normal n = normals->at(i);
					//cout << n.normal_x << ", " << n.normal_y << ", " << n.normal_z << endl;
					//PointXYZRGB pt = cloud->points[i];
					//cout << "(" << pt.x << "," << pt.y << "," << pt.z << " | " << int(pt.r) << "," << int(pt.g) << "," << int(pt.b) << ")" << endl;				
				//}
			}
	};

#endif
