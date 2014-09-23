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

#include "FeatureManager.hpp"

void FeatureManager::addFeatureExtractor(LocalFeature* lf)
{
	featureExtractors.push_back(lf);
}

void FeatureManager::extractFeatures(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, vector< vector< vector<double> > >& featuresTrainingClouds)
{
	// Dims = n_voxels x feature_size*N_features
	vector< vector<double> > features;
	//for each feature extractor...
	for(int f=0; f<featureExtractors.size(); f++)
	{
		// Dims = n_voxels x feature_size
		vector< vector<double> > featureF;
		LocalFeature* fExtractor = featureExtractors[f];
		fExtractor->computeFeature(cloud, featureF);

		//if it's the first time we initialize the structure
		if(features.empty())
		{
			for(int v=0; v<featureF.size(); v++)
				features.push_back(vector<double>());
		}
		
		//for each voxel we add the current feature
		for(int v=0; v<featureF.size(); v++)
			features[v].insert(features[v].end(), featureF[v].begin(), featureF[v].end());
	}

			
	featuresTrainingClouds.push_back(features);
}
