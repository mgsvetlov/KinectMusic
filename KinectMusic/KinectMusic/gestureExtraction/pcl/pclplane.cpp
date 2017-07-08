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

#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>

#include "pclplane.hpp"
#include "pclutility.hpp"
#include "../../log/logs.h"


void PclPlane::fitPlane(Blob& blob, float& x, float& y, float& z, float& w){
    
    // initialize PointClouds
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);    
    PclUtility::blob2cloud(blob, cloud);
    fitPlane(cloud, x, y, z, w);
}

void PclPlane::fitPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float& x, float& y, float& z, float& w){
    //std::vector<int> inliers, model;
    Eigen::VectorXf model_coefficients;
    pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr
    model_p (new pcl::SampleConsensusModelPlane<pcl::PointXYZ> (cloud));
    pcl::RandomSampleConsensus<pcl::PointXYZ> ransac (model_p);
    ransac.setDistanceThreshold (10.);
    
    ransac.computeModel();
    ransac.getModelCoefficients(model_coefficients);
    //ransac.getInliers(inliers);
    //ransac.getModel(model);
    x = model_coefficients.x();
    y = model_coefficients.y();
    z = model_coefficients.z();
    w = model_coefficients.w();
}






