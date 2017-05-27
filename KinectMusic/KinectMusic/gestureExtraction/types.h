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
#include <fstream>
#include <ctime>
#include <sstream>

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

#define USE_CSOUND

#ifdef USE_CSOUND
extern volatile bool CSOUND_START;
#endif

extern volatile int die;

#define NO_DATA_VALUE -1
#define START_GESTURE_VALUE -10
#define INSIDE_GESTURE_VALUE -100
#define END_GESTURE_VALUE -1000

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
    std::vector<HandData> data;
};

extern std::ofstream gesturesLog;

std::string getCurrentTime();

#endif /* types_h */
