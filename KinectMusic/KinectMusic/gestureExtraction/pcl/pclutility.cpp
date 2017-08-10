//
//  utility.cpp
//  KinectMusic
//
//  Created by Mikhail Svetlov on 08/07/17.
//  Copyright © 2017 mgsvetlov. All rights reserved.
//

#include "pclutility.h"

void PclUtility::points2cloud(std::list<cv::Point3i>& points,  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){
    cloud->width    = static_cast<unsigned int>(points.size());
    cloud->height   = 1;
    cloud->is_dense = false;
    cloud->points.resize (cloud->width * cloud->height);
    
    auto it = points.begin();
    for (size_t i = 0; i < cloud->points.size (); ++i, ++it) {
        cloud->points[i].x = it->x;
        cloud->points[i].y = it->y;
        cloud->points[i].z = it->z;
    }
}



