//
//  types.h
//  KinectMusic
//
//  Created by Mikhail Svetlov on 01/10/16.
//  Copyright © 2016 mgsvetlov. All rights reserved.
//

#ifndef types_h
#define types_h

#define NO_DATA_VALUE -1

#include <vector>
#include <ctime>

#include "opencv/cv.h"
#include "opencv/highgui.h"
#include "opencv2/opencv.hpp"
#include <iostream>

/*#define START_GESTURE_VALUE -10
#define INSIDE_GESTURE_VALUE -100
#define END_GESTURE_VALUE -1000

//#define USE_CELL_NORMAL

struct HandData {
    cv::Point3i point;
    int phase = NO_DATA_VALUE;
    int angle = NO_DATA_VALUE;
    HandData() : point (NO_DATA_VALUE,NO_DATA_VALUE,NO_DATA_VALUE), phase(NO_DATA_VALUE), angle(NO_DATA_VALUE) {}
    HandData(const cv::Point3i& point, int phase, int angle) :
    point(point),
    phase(phase),
    angle(angle)
    {}
};

struct FrameData {
    int frameNum;
    int bodyDepth;
    std::vector<HandData> data;
};

*/
#endif /* types_h */
