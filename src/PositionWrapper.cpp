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

#include "PositionWrapper.hpp"

void PositionWrapper::computeFeature(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, vector< vector<double> >& features)
{	
	//copy features to the output vector
	for(int i=0; i<cloud->points.size(); i++)
	{
		vector<double> desc;
		desc.push_back(cloud->points[i].x);
		desc.push_back(cloud->points[i].y);
		desc.push_back(cloud->points[i].z);
		features.push_back(desc);
	}
}

void PositionWrapper::computeFeature(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,  pcl::PointCloud<pcl::Normal>::Ptr& normals, vector< vector<double> >& features)
{
	//copy features to the output vector
	for(int i=0; i<cloud->points.size(); i++)
	{
		vector<double> desc;
		desc.push_back(cloud->points[i].x);
		desc.push_back(cloud->points[i].y);
		desc.push_back(cloud->points[i].z);
		features.push_back(desc);
	}
}
