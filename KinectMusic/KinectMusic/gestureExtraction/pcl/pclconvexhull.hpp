#ifndef pclconvex_hull_hpp
#define pclconvex_hull_hpp

#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include "../types.h"

class PclConvexHull {
public:
    static std::list<cv::Point3i> convecHull(std::list<cv::Point3i>& points);
};

#endif /* pclconvex_hull_hpp */