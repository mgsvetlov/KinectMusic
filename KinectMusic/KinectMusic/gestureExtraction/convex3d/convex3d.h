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
    static cv::Mat extractConvexities(cv::Mat mat, int filt_size, int filt_depth, int core_half_size);
    static cv::Mat extractConvexities1(cv::Mat mat, std::list<int> inds, int filt_size, int filt_depth, int core_half_size);
};

#endif /* convex3d_h */
