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
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include "pcl.hpp"
#include "../../log/logs.h"

std::list<Blob> Pcl::segmentation(Blob& blob){
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    blob2cloud(blob, cloud);
    // Creating the KdTree object for the search method of the extraction
    
    auto lClouds = Pcl::segmentation(cloud);
    std::list<Blob> lBlobs; //cloud2blob!
    return lBlobs;
}

std::list<pcl::PointCloud<pcl::PointXYZ>::Ptr> Pcl::segmentation(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){
    std::list<pcl::PointCloud<pcl::PointXYZ>::Ptr> lClouds;
    
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (cloud);
    
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance (40); // 2cm
    ec.setMinClusterSize (100);
    ec.setMaxClusterSize (25000);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud);
    ec.extract (cluster_indices);
    
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
        lClouds.emplace_back(new pcl::PointCloud<pcl::PointXYZ>);
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
            lClouds.back()->points.push_back(cloud->points[*pit]);
    }
    return lClouds;
}

void Pcl::fitPlane(Blob& blob, float& x, float& y, float& z, float& w){
    
    // initialize PointClouds
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);    
    blob2cloud(blob, cloud);
    fitPlane(cloud, x, y, z, w);
}

void Pcl::fitPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float& x, float& y, float& z, float& w){
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

void Pcl::blob2cloud(Blob& blob,  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){
    // populate our PointCloud with points
    auto& lCells = blob.getLCells();
    cloud->width    = static_cast<unsigned int>(lCells.size());
    cloud->height   = 1;
    cloud->is_dense = false;
    cloud->points.resize (cloud->width * cloud->height);
    
    auto it = lCells.begin();
    int w(blob.getMatSize().width);
    for (size_t i = 0; i < cloud->points.size (); ++i, ++it) {
        int ind = it->ind;
        cloud->points[i].x = ind % w;
        cloud->points[i].y = (ind-cloud->points[i].x) / w;
        cloud->points[i].z =  it->val;
    }
}


