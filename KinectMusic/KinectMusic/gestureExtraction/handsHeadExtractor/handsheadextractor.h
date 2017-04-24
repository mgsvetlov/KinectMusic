//
//  handsheadextractor.hpp
//  KinectMusic
//
//  Created by Mikhail Svetlov on 22/10/16.
//  Copyright © 2016 mgsvetlov. All rights reserved.
//

#ifndef handsheadextractor_h
#define handsheadextractor_h

#include <stdio.h>
#include "../types.h"

class HandsHeadExtractor {
public:
    static cv::Mat extractHandsHead(cv::Mat mat, int filt_size, int filt_depth, int core_half_size);
};

#endif /* handsheadextractor_hpp */
