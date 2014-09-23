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

#include <iostream>
#include <fstream>
#include <string>
#include <algorithm>
#include <ctype.h>
#include <dirent.h>
#include <stdlib.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>

using namespace std;
using namespace pcl;

   static int ColorsLabel [12][3] = {
                                {128,   128,    0}, //Vegatation (1)
                                {0,       0,    192}, //Sidewalk (2)
                                {128,     0,    0}, //Bulding (3)
                                {64,     64,    128}, //Fence (4)
                                {128,    64,    128}, //Road (5)
                                {64,      0,    128}, //Car (6)
                                {128,   128,    128}, //Sky (7)
                                {192,   192,    128}, //Pole (8)
                                {192,   128,    128}, //Sign (9)
                                {64,    64,     0}, //Pedestrian (10)
                                {0,     128,    192}, //Cyclist (11)
                                {  0,     0,      0} //Black (12)
                              };

int rgb2Label(PointXYZRGB& pt)
{

	for(int i=0; i<12; i++)
	{
		if( (ColorsLabel[i][0] == pt.r) && (ColorsLabel[i][1] == pt.g) && (ColorsLabel[i][2] == pt.b) )
		{
		    i = i+1;
		    switch(i)
		    {
		        case 1:
		            i= 2;
		            break;
		        case 2:
		            i = 1;
		            break;
		        case 3:
		            i= 2;
		            break;
		        case 4:
		            i= 2;
		            break;
		        case 5:
		            i= 0;
		            break;
		        case 6:
		            i= 2;
		            break;
		        case 7:
		            i= -1;
		            break;
		        case 8:
		            i= 2;
		            break;
		        case 9:
		            i= 2;
		            break;
		        case 10:
		            i= 2;
		            break;
		        case 11:
		            i= 2;
		            break;
		        case 12:
		            i= -1;
		            break;
		    }
		    return i;
		}
	}
	return -1;
}

int main(int argc, char** argv)
{
	if(argc != 4)
	{
		cerr << "[main] Syntax error. Try with " << argv[0] << "rgbCloud.pcd gtCloud.pcd outputCloud.pcd" << endl;
		return -1; 
	}

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr labels (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGBL>::Ptr rgbl (new pcl::PointCloud<pcl::PointXYZRGBL>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr xyz (new pcl::PointCloud<pcl::PointXYZ>);

	pcl::io::loadPCDFile<pcl::PointXYZRGB> (argv[1], *rgb);
	pcl::io::loadPCDFile<pcl::PointXYZRGB> (argv[2], *labels);

	pcl::copyPointCloud(*rgb, *xyz);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(xyz);
	tree->setEpsilon(0.05);

	for(int i=0; i<labels->size(); i++)
	{
		PointXYZRGB pt = labels->points[i];
		PointXYZ pt2;
		pt2.x = pt.x;
		pt2.y = pt.y;
		pt2.z = pt.z;

		vector<int> k_indices;
		vector<float> k_sqr_distances;
		tree->nearestKSearch(pt2, 1, k_indices, k_sqr_distances);

		PointXYZRGB rgbpt = rgb->points[k_indices[0]];
		//rgb to label!
		int pt_label = rgb2Label(labels->points[i]);
		if(pt_label != -1)
		{
			PointXYZRGBL ptout;
			ptout.x = pt.x;
			ptout.y = pt.y;
			ptout.z = pt.z;
			ptout.r = rgbpt.r;
			ptout.g = rgbpt.g;
			ptout.b = rgbpt.b;
			ptout.label = pt_label;

			rgbl->push_back(ptout);
		}
	}

	pcl::io::savePCDFile<pcl::PointXYZRGBL> (argv[3], *rgbl);
	
	return 0;	
}
