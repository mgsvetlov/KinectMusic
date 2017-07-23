//
//  extractframedata.h
//  KinectMusic
//
//  Created by Mikhail Svetlov on 17/09/16.
//  Copyright © 2016 mgsvetlov. All rights reserved.
//

#ifndef extractframedata_h
#define extractframedata_h

#include  <stdint.h>
#include <pthread.h>

struct ExtractFrameData {
static void *analyze_threadfunc(void *arg);
static pthread_mutex_t depth_mutex;
static volatile int die_kinect;
static volatile int die_gesture;
static volatile int MAX_KINECT_VALUE;
static volatile int MAX_NEIGHB_DIFF_COARSE;

};

//struct FrameData;
//void log(FrameData& frameData);

#endif /* extractframedata_h */
