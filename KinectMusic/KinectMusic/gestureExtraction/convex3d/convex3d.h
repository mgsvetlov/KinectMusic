//
//  handsheadextractor.hpp
//  KinectMusic
//
//  Created by Mikhail Svetlov on 22/10/16.
//  Copyright © 2016 mgsvetlov. All rights reserved.
//

#ifndef convex3d_h
#define convex3d_h

#include <stdio.h>
#include "../types.h"

class Convex3d {
public:
    static cv::Mat extractConvexities(cv::Mat mat, int filt_size, int filt_depth, int core_half_size, int count_false_percent, bool isZeroValid, std::list<int>& inds, bool extremumsOnly = false);
    static cv::Mat extractConvexitiesFine(cv::Mat mat, int start_dist, int end_dist, int dzThresh, int countGlobalMin, std::list<int>& inds);
    static int radius(cv::Mat mat, int ind, int start, int end, int dzThresh);
    static bool isBorderPoint(cv::Mat mat, int ind, int start, int end, int dzThresh);
private:
    static std::vector<std::pair<int,int>> neighbours;
};

#endif /* convex3d_h */
