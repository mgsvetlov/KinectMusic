//
//  pcl.hpp
//  KinectMusic
//
//  Created by Mikhail Svetlov on 05/07/17.
//  Copyright © 2017 mgsvetlov. All rights reserved.
//

#ifndef pcl_hpp
#define pcl_hpp

#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include "../blobs/blob.h"

class Pcl {
public:
static std::list<Blob> segmentation(Blob& blob);
static void fitPlane(Blob& blob, float& x, float& y, float& z, float& w);
    
private:
static std::list<pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentation(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
static void fitPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float& x, float& y, float& z, float& w);
static void blob2cloud(Blob& blob,  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
};

#endif /* pcl_hpp */
