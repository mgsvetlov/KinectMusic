//
//  utility.cpp
//  KinectMusic
//
//  Created by Mikhail Svetlov on 08/07/17.
//  Copyright © 2017 mgsvetlov. All rights reserved.
//

#include "pclutility.hpp"


void PclUtility::blob2cloud(Blob& blob,  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){
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

void PclUtility::cloud2blob(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, Blob& blob){
    auto& lCells = blob.getLCells();
    lCells.clear();
    int w(blob.getMatSize().width);
    for (size_t i = 0; i < cloud->points.size (); ++i){
        lCells.emplace_back(Cell(w * static_cast<int>(cloud->points[i].y) + cloud->points[i].x, cloud->points[i].z));
    }
}