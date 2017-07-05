//
//  pcl.cpp
//  KinectMusic
//
//  Created by Mikhail Svetlov on 05/07/17.
//  Copyright © 2017 mgsvetlov. All rights reserved.
//
#include <iostream>
#include <algorithm>
#include <sstream>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>

#include "pcl.hpp"
#include "../../log/logs.h"

void fitPlane(Blob& blob){
    
    // initialize PointClouds
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    
    // populate our PointCloud with points
    cloud->width    = static_cast<unsigned int>(blob.lCells.size());
    cloud->height   = 1;
    cloud->is_dense = false;
    cloud->points.resize (cloud->width * cloud->height);
    
    auto it = blob.lCells.begin();
    int w(blob.matSize.width);
    for (size_t i = 0; i < cloud->points.size (); ++i, ++it) {
        int ind = it->ind;
        cloud->points[i].x = ind % w;
        cloud->points[i].y = (ind-cloud->points[i].x) / w;
        cloud->points[i].z =  it->val;
    }
    
    std::vector<int> inliers, model;
    Eigen::VectorXf model_coefficients;
    pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr
    model_p (new pcl::SampleConsensusModelPlane<pcl::PointXYZ> (cloud));
    pcl::RandomSampleConsensus<pcl::PointXYZ> ransac (model_p);
    ransac.setDistanceThreshold (10.);
    
    ransac.computeModel();
    ransac.getModelCoefficients(model_coefficients);
    ransac.getInliers(inliers);
    ransac.getModel(model);
    std::stringstream ss;
    ss << "model: ";
    //for(auto m : model)
    ss << model_coefficients << " ";
    Logs::writeLog("gestures", ss.str());
    typedef std::list<Cell>::iterator IterList;
    std::vector<Cell> vCells (std::move_iterator<IterList>(blob.lCells.begin()), std::move_iterator<IterList>(blob.lCells.end()));
    blob.lCells.clear();
    for(auto i : inliers){
        blob.lCells.emplace_back(vCells[i]);
    }

}
