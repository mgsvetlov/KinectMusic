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
    HandsHeadExtractor(cv::Mat mat, int filt_size, int filt_depth, int iterCount);
    cv::Mat extractHandsHead();
private:
    cv::Mat mat;
    int filt_size, filt_depth, iterCount;
};

#endif /* handsheadextractor_hpp */
