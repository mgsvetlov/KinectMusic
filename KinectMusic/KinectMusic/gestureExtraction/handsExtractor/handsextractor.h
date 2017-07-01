//
//  handsheadextractor.hpp
//  KinectMusic
//
//  Created by Mikhail Svetlov on 22/10/16.
//  Copyright © 2016 mgsvetlov. All rights reserved.
//

#ifndef handsextractor_h
#define handsextractor_h

#include <stdio.h>
#include "../types.h"

class HandsExtractor {
public:
    static cv::Mat extractHands(cv::Mat mat, int filt_size, int filt_depth, int core_half_size);
};

#endif /* handsextractor_h */
