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
    HandsHeadExtractor(cv::Mat mat, int filt_size1, int filt_depth1, int filt_size2, int filt_depth2);
    cv::Mat extractHandsHead();
private:
    cv::Mat mat;
    int filt_size1, filt_depth1, filt_size2, filt_depth2;
};

#endif /* handsheadextractor_hpp */
