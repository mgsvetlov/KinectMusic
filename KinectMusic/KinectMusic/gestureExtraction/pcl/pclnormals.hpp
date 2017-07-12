//
//  pclnormals.hpp
//  KinectMusic
//
//  Created by Mikhail Svetlov on 08/07/17.
//  Copyright © 2017 mgsvetlov. All rights reserved.
//

#ifndef pclnormals_hpp
#define pclnormals_hpp

#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include "../blobs/blob.h"

class PclNormals {
public:
    static void estimateNormals(Blob& blob);
    
private:
    static pcl::PointCloud<pcl::Normal>::Ptr  estimateNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const std::vector<int>& indices);
};

#endif /* pclnormals_hpp */
