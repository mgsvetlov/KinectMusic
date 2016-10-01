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

extern pthread_t freenect_thread;

extern pthread_mutex_t depth_mutex;
extern pthread_mutex_t video_mutex;

extern volatile int die;

extern uint16_t * const depthAnalyze;

extern uint8_t *rgb_back, *rgb_mid, *rgb_front;

extern int w, h;

void analyzeLoop();
#endif /* analyze_h */
