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

#include "yass.hpp"
#include "CurvatureWrapper.hpp"
#include "PositionWrapper.hpp"
#include "ColorNWrapper.hpp"
#include "DONWrapper.hpp"
#include "FPFHWrapper.hpp"

using namespace std;
using namespace pcl;

struct mRGB
{
	int r, g, b;
	
	mRGB(int r, int g, int b) : r(r), g(g), b(b) {}
};

vector<string> readDir(string directory)
{
        vector<string> result;
        struct dirent **namelist;
        int n,i;

        n = scandir(directory.c_str(), &namelist, 0, versionsort);
        if (n < 0)
                cerr << "[readDir]: Error, no files found" << endl;
        else
        {
                for(i =0 ; i < n; ++i)
                {
                        if(namelist[i]->d_type == DT_REG)
                        {
                                //cout << namelist[i]->d_name << endl;
                                result.push_back(namelist[i]->d_name);
                                free(namelist[i]);
                        }
                }
                free(namelist);
        }

        return result;
}

void savingLabels(vector<int>& labels, string outputFile)
{
	ofstream fd(outputFile.c_str());
	
	for(int l=0; l<labels.size(); l++)
	{
		fd << labels[l] << endl;
	}

	fd.close();
}

void generateSemanticPointCloud(PointCloud<pcl::PointXYZRGB>::Ptr& cloud, vector<int>& labels, vector<mRGB>& colorScheme)
{
	for(int p=0; p<cloud->size(); p++)
	{
		mRGB c = colorScheme.at(labels[p]);
		cloud->points[p].r = c.r;
		cloud->points[p].g = c.g;
		cloud->points[p].b = c.b;
	}
}

int main(int argc, char** argv)
{
	int NLabels = 3;

	if(argc != 4)
	{
		cerr << "[main] Syntax error. Try with: " << argv[0] << " testing_pointcloud model outputCloud" << endl;
		return -1;
	}

	string testing = string(argv[1]);
	string modelFile = string(argv[2]);
	string outputCloud = string(argv[3]);
	
	// loading clouds
	pcl::PointCloud<pcl::PointXYZRGBL>::Ptr tempCloud (new pcl::PointCloud<pcl::PointXYZRGBL>);
	int result = pcl::io::loadPCDFile<pcl::PointXYZRGBL> (testing, *tempCloud);
		
	PointCloud<pcl::PointXYZRGB>::Ptr testCloud (new PointCloud<pcl::PointXYZRGB>);
	copyPointCloud<pcl::PointXYZRGBL, pcl::PointXYZRGB>(*tempCloud, *testCloud);

	//let's configure the features we want to use
	FeatureManager fManager;

	// FPFH...
	FPFHWrapper* fFPFH = new FPFHWrapper(0.1, 0.25);
	// curvatures...
	CurvatureWrapper* fCurvature = new CurvatureWrapper(0.1, 0.5);
	// 3D Position
	PositionWrapper* fPosition = new PositionWrapper();
	// Normalized colour
	ColorNWrapper* fColorN = new ColorNWrapper();
	// DoN...
	DONWrapper* fDON = new DONWrapper(0.1, 0.3);

	fManager.addFeatureExtractor(fFPFH);
	fManager.addFeatureExtractor(fCurvature);
	fManager.addFeatureExtractor(fPosition);
	fManager.addFeatureExtractor(fColorN);
	fManager.addFeatureExtractor(fDON);

	//train model
	YASS3D yass(fManager, NLabels);
	model* mymodel;	

	if((mymodel=load_model(modelFile.c_str()))==0)
	{
		cerr << "Error. Model " << modelFile << " couldn't be read" << endl;
		return -2;
	}
	
	//test model
	vector<mRGB> colors;
	colors.push_back(mRGB(128, 64, 128));
	colors.push_back(mRGB(0, 0, 192));
	colors.push_back(mRGB(128, 128, 0));
	
	cout << endl << endl << "\t *** Testing mode ***" << endl;
	
		
	vector<int> labels;
	yass.testVoxel(testCloud, mymodel, labels);

	//generate semantic point cloud
	generateSemanticPointCloud(testCloud, labels, colors);

	//saving labels and cloud
	pcl::io::savePCDFile<pcl::PointXYZRGB>(outputCloud.c_str(), *testCloud);



	return 0;
}
