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

#include "yass.hpp"

model* YASS3D::trainModel(vector<pcl::PointCloud<pcl::PointXYZRGBL>::Ptr>& trainingClouds)
{
	for(int c=0; c<trainingClouds.size(); c++)
	{
		//convert cloud to XYZRGB
		PointCloud<pcl::PointXYZRGB>::Ptr cloudXYZRGBTemp (new PointCloud<pcl::PointXYZRGB>);
		PointCloud<pcl::PointXYZRGB>::Ptr cloudXYZRGB (new PointCloud<pcl::PointXYZRGB>);
		copyPointCloud<pcl::PointXYZRGBL, pcl::PointXYZRGB>(*trainingClouds[c], *cloudXYZRGBTemp);

		pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
		sor.setInputCloud(cloudXYZRGBTemp);
		sor.setMeanK(100);
		sor.setStddevMulThresh(0.3);
		sor.filter(*cloudXYZRGB);

		//Extract supervoxels
		superVoxelClustering(cloudXYZRGB,  SVTables);

		//Extract features
		fManager.extractFeatures(cloudXYZRGB, featuresTrainingClouds);

		//Aggregate features
		aggregateFeaturesTraining(trainingClouds[c], featuresTrainingClouds[c], SVTables[c], SVFeaturesTrainingClouds, SVLabelsTrainingClouds);

		cout << "--- Cloud-[" << c+1 << "] processed ---" << endl;
	}

	debugCheckInstancesPerClass();

	//prepare for Liblinear
	//createLibLinearProblem();
	//if(autoAdjustWeights)
	//{
	//	int maxv = 0;
	//	for(int i=0; i<instancesPerClass.size(); i++)
	//	{
	//		if(instancesPerClass[i] > maxv)
	//			maxv = instancesPerClass[i];
	//	}

	//	for(int i=0; i<instancesPerClass.size(); i++)
	//		paramsLiblinear->weight[i] = round(double(maxv) / double(paramsLiblinear->weight[i]));	
	//}

	//second approach
	selectSamples();
	createLibLinearProblemV2();

	const char* out = check_parameter(&problemLiblinear, paramsLiblinear);
	if(out != NULL)
	{
		cerr << "LIBLIBNEAR " << out << endl;
		return NULL;
	}

	//train the model
	struct model* linearModel = train(&problemLiblinear, paramsLiblinear);	
	cout << "Model generated!" << endl;

	return linearModel;
}

bool YASS3D::isNANFree(vector<double>& feature)
{
	for(int i=0; i<feature.size(); i++)
		if(std::isnan(feature[i]))
			return false;
	return true;
}

void YASS3D::selectSamples()
{
	vector<int> accumLabels(NumClasses, 0);
	vector< vector< pair<int, int> > > candidates(NumClasses, vector< pair<int,int> >());
	for(int c=0; c<SVFeaturesTrainingClouds.size(); c++)
	{
		for(int sv=0; sv<SVFeaturesTrainingClouds[c].size(); sv++)
		{
			if(isNANFree(SVFeaturesTrainingClouds[c][sv]))
			{
				accumLabels[SVLabelsTrainingClouds[c][sv]]++;
				candidates[SVLabelsTrainingClouds[c][sv]].push_back(make_pair(c, sv));
			}
				
		}
	}

	//find the class with the lowest number of instances
	int minInstances = INT_MAX;
	for(int i=0; i<NumClasses; i++)
		if(accumLabels[i] < minInstances)
			minInstances = accumLabels[i];

	//now we need to draw minInstances features from each class
	for(int i=0; i<NumClasses; i++)
	{
		//first we shuffle the vector
		std::random_shuffle(candidates[i].begin(), candidates[i].end());
		//now we take the first minInstances elements
		for(int j=0; j<minInstances; j++)
		{
			int c = candidates[i][j].first;
			int sv = candidates[i][j].second;

			SVFeaturesTrainingCloudsRef.push_back(SVFeaturesTrainingClouds[c][sv]);
			SVLabelsTrainingCloudsRef.push_back(SVLabelsTrainingClouds[c][sv]);
		}
	}
}

void YASS3D::testSuperVoxel(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, const model* lmodel, vector<int>& labels)
{
	//Extract supervoxels
	vector<SortedSuperVoxels> localSVTables;
	superVoxelClustering(cloud,  localSVTables);

	//Extract features
	vector < vector< vector<double> > > localFeaturesTrainingClouds;
	fManager.extractFeatures(cloud, localFeaturesTrainingClouds);

	//Aggregate features
	vector < vector< vector<double> > > localSVFeaturesTrainingClouds;
	aggregateFeaturesTest(localFeaturesTrainingClouds[0], localSVTables[0], localSVFeaturesTrainingClouds);

	//now classify!
	int n = localSVFeaturesTrainingClouds[0][0].size();
	int N = n + ((liblinearBias)? 1 : 0);
	struct feature_node* instance = (struct feature_node *)malloc((N + 1) * sizeof(struct feature_node));
	//for each super-voxel sv-th
	for(int sv=0; sv<localSVTables[0].getNSV(); sv++)
	{
		//fill instance
		for(int i=0; i<n; i++)
		{
			instance[i].index = i+1;
			instance[i].value = std::isnan(localSVFeaturesTrainingClouds[0][sv][i])? 0.0 : localSVFeaturesTrainingClouds[0][sv][i];
		}

		if(liblinearBias)
		{
			instance[n].index = n+1;
			instance[n].value = 1.0;
			instance[n+1].index = -1;
		}

		else
			instance[n].index = -1;

		labels.push_back(predict(lmodel, instance));
	}
}

void YASS3D::testVoxel(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, const model* lmodel, vector<int>& labels)
{
	//Extract features
	vector < vector< vector<double> > > localFeaturesTrainingClouds;
	fManager.extractFeatures(cloud, localFeaturesTrainingClouds);

	//now classify!
	int n = localFeaturesTrainingClouds[0][0].size();
	int N = n + ((liblinearBias)? 1 : 0);
	struct feature_node* instance = (struct feature_node *)malloc((N + 1) * sizeof(struct feature_node));
	//for each voxel v-th
	for(int v=0; v<localFeaturesTrainingClouds[0].size(); v++)
	{
		//fill instance
		for(int i=0; i<n; i++)
		{
			instance[i].index = i+1;
			instance[i].value = std::isnan(localFeaturesTrainingClouds[0][v][i])? 0.0 : localFeaturesTrainingClouds[0][v][i];
		}

		if(liblinearBias)
		{
			instance[n].index = n+1;
			instance[n].value = 1.0;
			instance[n+1].index = -1;
		}

		else
			instance[n].index = -1;

		labels.push_back(predict(lmodel, instance));
	}
}

void YASS3D::setLiblinearOptions(struct parameter* paramsLiblinear, bool bias, bool autoAdjustWeights)
{
	this->paramsLiblinear = paramsLiblinear;
	this->liblinearBias = bias;
	this->autoAdjustWeights = autoAdjustWeights;
}

void YASS3D::createLibLinearProblemV2()
{
	problemLiblinear.l = SVFeaturesTrainingCloudsRef.size();
	problemLiblinear.n = SVFeaturesTrainingCloudsRef[0].size() + ((liblinearBias)? 1 : 0);
	problemLiblinear.y = (double*)malloc(problemLiblinear.l * sizeof(double));
	problemLiblinear.x = (struct feature_node **)malloc(problemLiblinear.l * sizeof(struct feature_node *));

	int N = SVFeaturesTrainingCloudsRef[0].size();
	int idx = 0;

	//for each cloud c-th
	for(int sv=0; sv<SVFeaturesTrainingCloudsRef.size(); sv++)
	{
		problemLiblinear.x[idx] = (struct feature_node *)malloc((problemLiblinear.n + 1) * sizeof(struct feature_node));
		problemLiblinear.y[idx] = SVLabelsTrainingCloudsRef[sv];

		for(int i=0; i<N; i++)
		{
			problemLiblinear.x[idx][i].index = i+1;
			problemLiblinear.x[idx][i].value = SVFeaturesTrainingCloudsRef[sv][i];
		}

		if(liblinearBias)
		{
			problemLiblinear.x[idx][N].index = N+1;
			problemLiblinear.x[idx][N].value = 1;
			//this is just to respect the "sparse" codification
			//used by liblinear. It seems that they consider
			//prob.x as a sparse vector and the separation between
			//the i-th instante and the i+1-th instance is given by
			//a special "token" that has index = -1.
			problemLiblinear.x[idx][N+1].index = -1;
		}

		else
			problemLiblinear.x[idx][N].index = -1;
			
		idx++;
	}

	problemLiblinear.bias = liblinearBias;
}

void YASS3D::createLibLinearProblem()
{
	//From SVFeaturesTrainingClouds and SVLabelsTrainingClouds
	// to problemLiblinear (problem)
	problemLiblinear.l = 0;
	for(int c=0; c<SVTables.size(); c++)
		problemLiblinear.l += SVTables[c].getNSV();
	problemLiblinear.n = SVFeaturesTrainingClouds[0][0].size() + ((liblinearBias)? 1 : 0);
	problemLiblinear.y = (double*)malloc(problemLiblinear.l * sizeof(double));
	problemLiblinear.x = (struct feature_node **)malloc(problemLiblinear.l * sizeof(struct feature_node *));

	int N = SVFeaturesTrainingClouds[0][0].size();
	int idx = 0;

	//for each cloud c-th
	for(int c=0; c<SVTables.size(); c++)
	{
		//for each super-voxel sv-th
		for(int sv=0; sv<SVTables[c].getNSV(); sv++)
		{
			problemLiblinear.x[idx] = (struct feature_node *)malloc((problemLiblinear.n + 1) * sizeof(struct feature_node));
			problemLiblinear.y[idx] = SVLabelsTrainingClouds[c][sv];
			instancesPerClass[SVLabelsTrainingClouds[c][sv]]++;

			for(int i=0; i<N; i++)
			{
				problemLiblinear.x[idx][i].index = i+1;
				problemLiblinear.x[idx][i].value = std::isnan(SVFeaturesTrainingClouds[c][sv][i])? 0.0 : SVFeaturesTrainingClouds[c][sv][i];
			}

			if(liblinearBias)
			{
				problemLiblinear.x[idx][N].index = N+1;
				problemLiblinear.x[idx][N].value = 1;
				//this is just to respect the "sparse" codification
				//used by liblinear. It seems that they consider
				//prob.x as a sparse vector and the separation between
				//the i-th instante and the i+1-th instance is given by
				//a special "token" that has index = -1.
				problemLiblinear.x[idx][N+1].index = -1;
			}

			else
				problemLiblinear.x[idx][N].index = -1;
			
			idx++;
		}
	}

	problemLiblinear.bias = liblinearBias;
}

void YASS3D::aggregateFeaturesTest(const vector< vector<double> >& featuresTraining, SortedSuperVoxels& SVList, vector< vector< vector<double> > >& SVFeaturesTrainingClouds)
{

	//for each super voxel...
	vector< vector<double> > SVFeatures;
	vector<int> labels;
	for(int sv=0; sv<SVList.getNSV(); sv++)
	{

		vector<double> aggregatedFeature(featuresTraining[0].size(), 0.0);
		// retrieve all the voxels of super-voxel sv-th
		for(int v=0; v<SVList.getSVSize(sv); v++)
		{
			//look for the feature of the voxel v-th
			int idx = SVList.getVoxelIndex(sv, v);
			vector<double> featureV = featuresTraining[idx];
			for(int i=0; i<featureV.size(); i++)
				aggregatedFeature[i] = (aggregatedFeature[i] + featureV[i]) / double(SVList.getSVSize(sv));

		}

		//add the aggregated feature to the output vector
		SVFeatures.push_back(aggregatedFeature);
	}

	SVFeaturesTrainingClouds.push_back(SVFeatures);

	cout << "\t* Features aggregated!" << endl;
}

void YASS3D::aggregateFeaturesTraining(const pcl::PointCloud<pcl::PointXYZRGBL>::Ptr& cloudLabels, const vector< vector<double> >& featuresTraining, SortedSuperVoxels& SVList, vector< vector< vector<double> > >& SVFeaturesTrainingClouds, vector< vector<int> >& SVLabelsTrainingClouds)
{

	//for each super voxel...
	vector< vector<double> > SVFeatures;
	vector<int> labels;
	for(int sv=0; sv<SVList.getNSV(); sv++)
	{

		labelCount.reset();
		vector<double> aggregatedFeature(featuresTraining[0].size(), 0.0);
		// retrieve all the voxels of super-voxel sv-th
		for(int v=0; v<SVList.getSVSize(sv); v++)
		{
			//look for the feature of the voxel v-th
			int idx = SVList.getVoxelIndex(sv, v);
			vector<double> featureV = featuresTraining[idx];

			for(int i=0; i<featureV.size(); i++)
				aggregatedFeature[i] = (aggregatedFeature[i] + featureV[i]) / double(SVList.getSVSize(sv));
			int label = cloudLabels->points[idx].label;
			labelCount.countLabel(label);
		}

		//add the aggregated feature to the output vector
		SVFeatures.push_back(aggregatedFeature);
		//add the aggregated label
		labels.push_back(labelCount.getFinalLabel());
	}

	SVFeaturesTrainingClouds.push_back(SVFeatures);
	SVLabelsTrainingClouds.push_back(labels);

	cout << "\t* Features aggregated!" << endl;
}

void YASS3D::superVoxelClustering(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, vector<SortedSuperVoxels>& SVTables)
{
	PointCloud<pcl::PointXYZ>::Ptr cloudXYZ (new PointCloud<pcl::PointXYZ>);
	copyPointCloud<pcl::PointXYZRGB, pcl::PointXYZ>(*cloud, *cloudXYZ);

	bool use_transform = true;
	float voxel_resolution = 0.05f;
	float seed_resolution = 0.05f;
	float color_importance = 1.0f;
	float spatial_importance = 10.0f;
	float normal_importance = 1.0f;

	//supervoxel extractor
	pcl::SupervoxelClustering<pcl::PointXYZRGB> super(voxel_resolution, seed_resolution, use_transform);
	super.setInputCloud(cloud);
	super.setColorImportance(color_importance);
	super.setSpatialImportance(spatial_importance);
	super.setNormalImportance(normal_importance);

	std::map <uint32_t, pcl::Supervoxel<pcl::PointXYZRGB>::Ptr > supervoxel_clusters;
	super.extract(supervoxel_clusters);

	pcl::PointCloud<PointXYZL>::Ptr cloudSV = super.getLabeledCloud();
	int NSV = super.getMaxLabel() + 1;

	//This is an indexing structure
	//used to link SuperVoxels and their voxels!
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(cloudXYZ);
	tree->setEpsilon(0.05);
	SortedSuperVoxels sortedSV(NSV);

	cout << "\t* SuperVoxel extracted!" << endl;

  	for(int v=0; v<cloudSV->size(); v++)
  	{
		PointXYZL pt = cloudSV->points[v];
		PointXYZ pt2;
		pt2.x = pt.x;
		pt2.y = pt.y;
		pt2.z = pt.z;

		vector<int> k_indices;
		vector<float> k_sqr_distances;
		tree->nearestKSearch(pt2, 1, k_indices, k_sqr_distances);
      		sortedSV.addVoxel(int(pt.label), k_indices[0]);
  	}

	SVTables.push_back(sortedSV);	

	cout << "\t* Indexing structure created!" << endl;
}
