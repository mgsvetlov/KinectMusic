//
//  pclnormals.cpp
//  KinectMusic
//
//  Created by Mikhail Svetlov on 08/07/17.
//  Copyright © 2017 mgsvetlov. All rights reserved.
//
#include <pcl/features/normal_3d.h>
#include "pclnormals.hpp"
#include "pclutility.hpp"
#include "../../log/logs.h"

void PclNormals::estimateNormals(Blob& blob){
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    PclUtility::blob2cloud(blob, cloud);
    // Create a set of indices to be used.
    int coeff(20);
    std::vector<int> indices (floor (cloud->points.size () / coeff));
    for (int i = 0; i < indices.size (); ++i) indices[i] = i * coeff;
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals = estimateNormals(cloud, indices);
    auto& cells = blob.getCells();
    auto itInd = indices.begin();
    for(auto& normal : *cloud_normals){
        next(cells.All().begin(), *itInd)->normal = cv::Vec4f(normal.normal_x, normal.normal_y, normal.normal_z, normal.curvature);
        ++itInd;
    }    
}

pcl::PointCloud<pcl::Normal>::Ptr  PclNormals::estimateNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const std::vector<int>& indices){
    
    // Create the normal estimation class, and pass the input dataset to it
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud (cloud);
    
    // Pass the indices
    boost::shared_ptr<std::vector<int> > indicesptr (new std::vector<int> (indices));
    ne.setIndices (indicesptr);
    
    // Create an empty kdtree representation, and pass it to the normal estimation object.
    // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
    ne.setSearchMethod (tree);
    
    // Output datasets
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
    
    // Use all neighbors in a sphere of radius 3cm
    ne.setRadiusSearch (10);
    
    // Compute the features
    ne.compute (*cloud_normals);
    
    // cloud_normals->points.size () should have the same size as the input cloud->points.size ()
    
    return cloud_normals;
}