//
//  pclconvexhull.cpp
//  KinectMusic
//
//  Created by Mikhail Svetlov on 08/09/17.
//  Copyright © 2017 mgsvetlov. All rights reserved.
//

#include "pclutility.h"
#include "pclconvexhull.hpp"
#include <pcl/surface/convex_hull.h>

std::list<cv::Point3i> PclConvexHull::convecHull(std::list<cv::Point3i>& points){
     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    PclUtility::points2cloud(points, cloud);
    pcl::ConvexHull<pcl::PointXYZ> cHull;
    pcl::PointCloud<pcl::PointXYZ> cHull_points;
    cHull.setInputCloud(cloud);
    cHull.reconstruct (cHull_points);
    std::list<cv::Point3i> pointsConvexHull;

    for (size_t i = 0; i < cHull_points.points.size (); ++i) {
        pointsConvexHull.emplace_back(cHull_points.points[i].x, cHull_points.points[i].y, cHull_points.points[i].z);
    }
    return pointsConvexHull;
}