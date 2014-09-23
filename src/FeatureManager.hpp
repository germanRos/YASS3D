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

#ifndef _FEATURE_MANAGER_HPP_
	#define _FEATURE_MANAGER_HPP_

	#include "LocalFeature.hpp"

	class FeatureManager
	{
		public:
			void addFeatureExtractor(LocalFeature* lf);

			void extractFeatures(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, vector< vector< vector<double> > >& featuresTrainingClouds);

		private:
			vector<LocalFeature*> featureExtractors;
	};

#endif
