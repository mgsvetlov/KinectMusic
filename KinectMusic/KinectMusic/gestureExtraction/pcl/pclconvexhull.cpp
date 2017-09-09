//
//  pclconvexhull.cpp
//  KinectMusic
//
//  Created by Mikhail Svetlov on 08/09/17.
//  Copyright © 2017 mgsvetlov. All rights reserved.
//
#include "pthread.h"
#include "pclutility.h"
#include "pclconvexhull.hpp"
#include <pcl/surface/convex_hull.h>
//#include "../extractframedata.h"

pthread_mutex_t PclConvexHull::convexhullMutex = PTHREAD_MUTEX_INITIALIZER;
//std::list<cv::Point3i> PclConvexHull::input, PclConvexHull::output;

/*void *PclConvexHull::threadfunc(void *arg){
    while (!ExtractFrameData::die_gesture){
        pthread_mutex_lock(&convexhullMutex);
        if(input.empty()){
            pthread_mutex_unlock(&convexhullMutex);
            usleep(100);
            continue;
        }
        output = convecHull(input);
        pthread_mutex_unlock(&convexhullMutex);
    }
    return NULL;
}*/

std::vector<cv::Point3i> PclConvexHull::convecHull(const std::list<cv::Point3i>& points, std::vector<std::vector< uint32_t>>& polyInds){
    
     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    PclUtility::points2cloud(points, cloud);
    pcl::ConvexHull<pcl::PointXYZ> cHull;
    pcl::PointCloud<pcl::PointXYZ> cHull_points;
    cHull.setInputCloud(cloud);
    
    pcl::PolygonMesh output;
    
    pthread_mutex_lock(&convexhullMutex);
    cHull.reconstruct(output);
    pthread_mutex_unlock(&convexhullMutex);
    
    fromPCLPointCloud2 (output.cloud, cHull_points);
    std::vector<cv::Point3i> pointsConvexHull;
    for (size_t i = 0; i < cHull_points.points.size (); ++i) {
        pointsConvexHull.emplace_back(cHull_points.points[i].x, cHull_points.points[i].y, cHull_points.points[i].z);
    }
    
    for(auto& polygon : output.polygons){
        polyInds.push_back(polygon.vertices);
    }
    
    return pointsConvexHull;
}