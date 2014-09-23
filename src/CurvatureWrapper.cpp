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

#include "CurvatureWrapper.hpp"

void CurvatureWrapper::computeFeature(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, vector< vector<double> >& features)
{	
	//compute normals
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
	computeNormals(cloud, radiusNormal, normals, tree);

	//compute 
	pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr principal_curvatures = pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr(new pcl::PointCloud<pcl::PrincipalCurvatures>);
	principal_curvatures_estimation = PrincipalCurvaturesEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::PrincipalCurvatures>();
	principal_curvatures_estimation.setInputCloud(cloud);
	principal_curvatures_estimation.setInputNormals(normals);

	principal_curvatures_estimation.setSearchMethod (tree);
	principal_curvatures_estimation.setRadiusSearch (radiusCurvature);
	principal_curvatures_estimation.compute (*principal_curvatures);

	//copy features to the output vector
	for(int i=0; i<principal_curvatures->points.size(); i++)
	{
		pcl::PrincipalCurvatures descriptor = principal_curvatures->points[i];

		//cout << "(" << descriptor.principal_curvature_x << ", " << descriptor.principal_curvature_y << ", " << descriptor.principal_curvature_z << ")" << endl;
		vector<double> desc;
		desc.push_back(descriptor.principal_curvature_x);
		desc.push_back(descriptor.principal_curvature_y);
		desc.push_back(descriptor.principal_curvature_z);
		features.push_back(desc);
	}
}

void CurvatureWrapper::computeFeature(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,  pcl::PointCloud<pcl::Normal>::Ptr& normals, vector< vector<double> >& features)
{
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);

	//compute 
	pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr principal_curvatures = pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr(new pcl::PointCloud<pcl::PrincipalCurvatures>);
	principal_curvatures_estimation = PrincipalCurvaturesEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::PrincipalCurvatures>();
	principal_curvatures_estimation.setInputCloud(cloud);
	principal_curvatures_estimation.setInputNormals (normals);

	principal_curvatures_estimation.setSearchMethod (tree);
	principal_curvatures_estimation.setRadiusSearch (radiusCurvature);
	principal_curvatures_estimation.compute (*principal_curvatures);

	//copy features to the output vector
	for(int i=0; i<principal_curvatures->points.size(); i++)
	{
		pcl::PrincipalCurvatures descriptor = principal_curvatures->points[i];
		vector<double> desc;
		desc.push_back(descriptor.principal_curvature_x);
		desc.push_back(descriptor.principal_curvature_y);
		desc.push_back(descriptor.principal_curvature_z);
		features.push_back(desc);
	}
}
