#ifndef pclconvex_hull_hpp
#define pclconvex_hull_hpp

#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include "../types.h"

class PclConvexHull {
public:
    //static void *threadfunc(void *arg);
    static std::vector<cv::Point3i> convecHull(const std::list<cv::Point3i>& points, std::vector<std::vector< uint32_t >>& polyInds);
    static pthread_mutex_t convexhullMutex;
    //static std::list<cv::Point3i> input, output;
};

#endif /* pclconvex_hull_hpp */