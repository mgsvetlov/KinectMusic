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

class PclPlane {
public:
    static void fitPlane(Blob& blob, float& x, float& y, float& z, float& w);
    
private:
    static void fitPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float& x, float& y, float& z, float& w);
};

#endif /* pcl_hpp */
