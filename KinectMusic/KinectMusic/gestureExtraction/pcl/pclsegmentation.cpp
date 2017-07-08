//
//  pclsegmentation.cpp
//  KinectMusic
//
//  Created by Mikhail Svetlov on 08/07/17.
//  Copyright © 2017 mgsvetlov. All rights reserved.
//
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include "pclsegmentation.hpp"
#include "pclutility.hpp"

std::list<Blob> PclSegmentation::segmentation(Blob& blob){
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    PclUtility::blob2cloud(blob, cloud);
    // Creating the KdTree object for the search method of the extraction
    
    auto lClouds = segmentation(cloud);
    std::list<Blob> lBlobs; //cloud2blob!
    return lBlobs;
}

std::list<pcl::PointCloud<pcl::PointXYZ>::Ptr> PclSegmentation::segmentation(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){
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