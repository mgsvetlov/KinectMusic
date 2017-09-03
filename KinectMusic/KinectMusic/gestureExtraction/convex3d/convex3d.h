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
    static cv::Mat extractConvexities1(cv::Mat mat, int filt_size, int filt_depth, int core_half_size, int count_false_percent, bool isZeroValid, std::list<int>& inds, bool extremumsOnly = false);
private:
    static std::vector<std::pair<int,int>> neighbours;
};

#endif /* convex3d_h */
