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

extern int MAX_KINECT_VALUE;
extern int MAX_KINECT_DEPTH;
extern int BLOBS_RESIZE_POW;
extern int BLOB_MIN_SIZE;
extern int BLOB_MIN_SIZE_LAST;
extern int MAX_NEIGHB_DIFF_COARSE;
extern int MAX_NEIGHB_DIFF_FINE;

extern volatile int die_kinect;
extern volatile int die_gesture;

#define NO_DATA_VALUE -1
#define START_GESTURE_VALUE -10
#define INSIDE_GESTURE_VALUE -100
#define END_GESTURE_VALUE -1000

//#define VISUALIZATION

struct HandData {
    cv::Point3i point;
    int phase = NO_DATA_VALUE;
    HandData() : point (NO_DATA_VALUE,NO_DATA_VALUE,NO_DATA_VALUE), phase(NO_DATA_VALUE) {}
    HandData(const cv::Point3i& point, int phase) :
    point(point),
    phase(phase)
    {}
};

struct FrameData {
    int frameNum;
    int bodyDepth;
    std::vector<HandData> data;
};


#endif /* types_h */
