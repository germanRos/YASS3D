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

#include "DONWrapper.hpp"

void DONWrapper::computeFeature(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, vector< vector<double> >& features)
{	
	normal_estimation.setInputCloud(cloud);
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
	normal_estimation.setSearchMethod(tree);

	// calculate normals with the small scale
	pcl::PointCloud<pcl::Normal>::Ptr normals_small_scale (new pcl::PointCloud<pcl::Normal>);
	normal_estimation.setRadiusSearch (radiusLow);
	normal_estimation.compute (*normals_small_scale);

	// calculate normals with the large scale
	pcl::PointCloud<pcl::Normal>::Ptr normals_large_scale (new pcl::PointCloud<pcl::Normal>);
	normal_estimation.setRadiusSearch (radiusLarge);
	normal_estimation.compute (*normals_large_scale);

	// Create output cloud for DoN results
	PointCloud<PointNormal>::Ptr doncloud(new pcl::PointCloud<PointNormal>);
	copyPointCloud<PointXYZRGB, PointNormal>(*cloud, *doncloud);

	pcl::PointCloud<PointNormal>::Ptr normals_small_scale1 (new pcl::PointCloud<PointNormal>);
	pcl::PointCloud<PointNormal>::Ptr normals_large_scale1 (new pcl::PointCloud<PointNormal>);

	copyPointCloud<Normal, PointNormal>(*normals_large_scale, *normals_large_scale1);
	copyPointCloud<Normal, PointNormal>(*normals_small_scale, *normals_small_scale1);

	pcl::DifferenceOfNormalsEstimation<pcl::PointXYZRGB, PointNormal, PointNormal> difference;
	difference.setInputCloud(cloud);
	difference.setNormalScaleLarge (normals_large_scale1);
	difference.setNormalScaleSmall (normals_small_scale1);
	difference.computeFeature(*doncloud);

	for(int v=0; v<doncloud->points.size(); v++)
	{
		vector<double> desc;
		desc.push_back(doncloud->at(v).normal_x);
		desc.push_back(doncloud->at(v).normal_y);
		desc.push_back(doncloud->at(v).normal_z);
		features.push_back(desc);
	}
}

void DONWrapper::computeFeature(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,  pcl::PointCloud<pcl::Normal>::Ptr& normals, vector< vector<double> >& features)
{

}
