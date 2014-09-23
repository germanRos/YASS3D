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

#include "FPFHWrapper.hpp"

void FPFHWrapper::computeFeature(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, vector< vector<double> >& features)
{
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
	computeNormals(cloud, radiusNormal, normals, tree);

	// Provide the original point cloud (without normals)
	fpfh_estimation.setInputCloud(cloud);
	// Provide the point cloud with normals
	fpfh_estimation.setInputNormals(normals);
	// fpfhEstimation.setInputWithNormals(cloud, cloudWithNormals); PFHEstimation does not have this function
	// Use the same KdTree from the normal estimation
	fpfh_estimation.setSearchMethod(tree);
	//pcl::PointCloud<pcl::FPFHSignature33>::Ptr pfh_features (new pcl::PointCloud<pcl::FPFHSignature33>);
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr pfh_features (new pcl::PointCloud<pcl::FPFHSignature33>);
	fpfh_estimation.setRadiusSearch (radiusFPFH);

	// Actually compute the spin images
	fpfh_estimation.compute (*pfh_features);

	for(int v=0; v<pfh_features->size(); v++)
	{
		vector<double> desc;
		for(int i=0; i<33; i++)
			desc.push_back(pfh_features->at(v).histogram[i]);
		features.push_back(desc);
	}
}
void FPFHWrapper::computeFeature(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,  pcl::PointCloud<pcl::Normal>::Ptr& normals, vector< vector<double> >& features)
{

}
