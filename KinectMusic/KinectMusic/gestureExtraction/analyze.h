//
//  analyze.h
//  KinectMusic
//
//  Created by Mikhail Svetlov on 17/09/16.
//  Copyright © 2016 mgsvetlov. All rights reserved.
//

#ifndef analyze_h
#define analyze_h

#include  <stdint.h>
#include <pthread.h>


extern pthread_mutex_t depth_mutex;
extern pthread_mutex_t video_mutex;
extern pthread_mutex_t visualisation_mutex;

extern uint16_t * const depthAnalyze;

extern uint8_t *rgb_back, *rgb_mid, *rgb_front;

extern volatile bool newFrame;

extern int w, h;

extern volatile int frameNum;

void *analyze_threadfunc(void *arg);

struct FrameData;
void log(FrameData& frameData);

#endif /* analyze_h */
