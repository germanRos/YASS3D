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

#ifndef _YASS3D_HPP_
	#define _YASS3D_HPP_

	#include <pcl/io/pcd_io.h>
	#include <pcl/point_cloud.h>
	#include <pcl/point_types.h>
	#include <pcl/segmentation/supervoxel_clustering.h>	
	#include "FeatureManager.hpp"
	#include "linear.h"
	#include <limits.h>
	#include <algorithm>
	#include <pcl/filters/statistical_outlier_removal.h>
	
	using namespace pcl;

	struct SortedSuperVoxels
	{
		vector< vector<int> > table;

		SortedSuperVoxels(int NSV)
		{
			table = vector< vector<int> >(NSV, vector<int>());
		}

		const int getNSV()
		{
			return table.size();
		}

		const int getSVSize(const int i)
		{
			return table[i].size();
		}

		const int getVoxelIndex(int sv, int i)
		{
			return table[sv][i];
		}

		void addVoxel(const int sv, int index)
		{
			table[sv].push_back(index);			
		}
	};

	struct LabelCounter
	{
		int NClasses;
		int* hist;

		LabelCounter() {};

		LabelCounter(int N)
		{
			NClasses = N;
			hist = new int[N];
		}

		void reset()
		{
			for(int i=0; i<NClasses; i++)
				hist[i] = 0;
		}

		void countLabel(int label)
		{
			if(label < NClasses)
				hist[label]++;
		}

		int getFinalLabel()
		{
			int maxIdx = -1;
			int maxValue = -1;

			for(int i=0; i<NClasses; i++)
			{
				if(hist[i] > maxValue)
				{
					maxValue = hist[i];
					maxIdx = i;
				}
			}

			return maxIdx;
		}
	};

	/**
	* Yet Another Semantic Segmentator in 3D
	*/
	class YASS3D
	{
		public:
			YASS3D(FeatureManager& manager, int NumClasses)
			{
				this->fManager = manager;
				this->NumClasses = NumClasses;
				this->labelCount = LabelCounter(NumClasses);
				this->liblinearBias = false;
				this->autoAdjustWeights = true;

				instancesPerClass = vector<int>(NumClasses, 0);
			}


			model* trainModel(vector<pcl::PointCloud<pcl::PointXYZRGBL>::Ptr>& trainingClouds);

			void testSuperVoxel(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, const model* lmodel, vector<int>& labels);

			void testVoxel(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, const model* lmodel, vector<int>& labels);

			void setLiblinearOptions(struct parameter* paramsLiblinear, bool bias, bool autoAdjustWeights);

		private:
			int NumClasses;
			FeatureManager fManager;
			vector<SortedSuperVoxels> SVTables;
			vector < vector< vector<double> > > featuresTrainingClouds;
			vector < vector< vector<double> > > SVFeaturesTrainingClouds;
			vector < vector<int> > SVLabelsTrainingClouds;
			vector<int> instancesPerClass;

			//these are special versions of the above structures
			//after a process of candidate selection
			vector< vector<double> > SVFeaturesTrainingCloudsRef;
			vector<int> SVLabelsTrainingCloudsRef;

			LabelCounter labelCount;
			struct problem problemLiblinear;
			struct parameter* paramsLiblinear;
			bool liblinearBias;
			bool autoAdjustWeights;


			void superVoxelClustering(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudRGB, vector<SortedSuperVoxels>& SVTable);

			void aggregateFeaturesTraining(const pcl::PointCloud<pcl::PointXYZRGBL>::Ptr& cloudLabels, const vector< vector<double> >& featuresTraining, SortedSuperVoxels& SVList, vector< vector< vector<double> > >& SVFeaturesTrainingClouds, vector< vector<int> >& SVLabelsTrainingClouds);

			void aggregateFeaturesTest(const vector< vector<double> >& featuresTraining, SortedSuperVoxels& SVList, vector< vector< vector<double> > >& SVFeaturesTrainingClouds);			

			void createLibLinearProblem();

			void createLibLinearProblemV2();

			bool isNANFree(vector<double>& feature);

			void selectSamples();

			void debugCheckInstancesPerClass()
			{
				vector<int> classes(NumClasses, 0);
				for(int c=0; c<SVLabelsTrainingClouds.size(); c++)
					for(int sv=0; sv<SVLabelsTrainingClouds[c].size(); sv++)
						classes[SVLabelsTrainingClouds[c][sv]]++;

				for(int i=0; i<classes.size(); i++)
					cout << "--- Class " << i << " with [" << classes[i] << "] instances" << endl; 
			}

			void debugPrint(SortedSuperVoxels& svTable, string outputFile)
			{
				ofstream fd(outputFile.c_str());
	
				for(int sv=0; sv<svTable.getNSV(); sv++)
				{
					for(int i=0; i<svTable.getSVSize(sv); i++)
						fd << svTable.getVoxelIndex(sv, i) << " ";
					fd << endl;
				}

				fd.close();
			}

			void debugPrintFeature(int cloudIdx, string outputFile)
			{
				ofstream fd(outputFile.c_str());
				
				int NVoxels = featuresTrainingClouds[cloudIdx].size();
				for(int v=0; v<NVoxels; v++)
				{
					for(int f=0; f<featuresTrainingClouds[cloudIdx][v].size(); f++)
						fd << featuresTrainingClouds[cloudIdx][v][f] << " ";
					fd << endl;
				}

				fd.close();
			}

			void debugPrintAggregatedFeature(int cloudIdx, string outputFile)
			{
				ofstream fd(outputFile.c_str());
				
				int NVoxels = SVFeaturesTrainingClouds[cloudIdx].size();
				for(int v=0; v<NVoxels; v++)
				{
					for(int f=0; f<SVFeaturesTrainingClouds[cloudIdx][v].size(); f++)
						fd << featuresTrainingClouds[cloudIdx][v][f] << " ";
					fd << endl;
				}

				fd.close();
			}

			void debugPrintAggregatedLabels(int cloudIdx, string outputFile)
			{
				ofstream fd(outputFile.c_str());
				
				int NVoxels = SVLabelsTrainingClouds[cloudIdx].size();
				for(int v=0; v<NVoxels; v++)
				{
					fd << SVLabelsTrainingClouds[cloudIdx][v] << endl;
				}

				fd.close();
			}
	};

	

#endif
