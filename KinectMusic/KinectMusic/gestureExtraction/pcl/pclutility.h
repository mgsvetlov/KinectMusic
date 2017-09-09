//
//  utility.hpp
//  KinectMusic
//
//  Created by Mikhail Svetlov on 08/07/17.
//  Copyright © 2017 mgsvetlov. All rights reserved.
//

#ifndef utility_hpp
#define utility_hpp

#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include "../types.h"

class PclUtility {
public:
    static void points2cloud(const std::list<cv::Point3i>& points,  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
};

#endif /* utility_hpp */
