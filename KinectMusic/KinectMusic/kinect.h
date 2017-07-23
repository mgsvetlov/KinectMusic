//
//  kinect.h
//  KinectMusic
//
//  Created by Mikhail Svetlov on 20/09/15.
//  Copyright (c) 2015 mgsvetlov. All rights reserved.
//

#ifndef __KinectMusic__kinect__
#define __KinectMusic__kinect__

#include <stdio.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>

#include "libfreenect.h"

namespace Sensor {
extern int volatile frameNum;
extern uint16_t* pDepthMatrix;
}

extern uint8_t *rgb_back, *rgb_mid, *rgb_front;
extern uint8_t *depth_mid;
extern freenect_context *f_ctx;
extern freenect_device *f_dev;
extern int freenect_led;

extern freenect_video_format requested_format;
extern freenect_video_format current_format;
extern freenect_resolution requested_resolution;
extern freenect_resolution current_resolution;

extern uint16_t t_gamma[2048];

extern int got_depth;
extern int got_rgb;

void depth_cb(freenect_device *dev, void *v_depth, uint32_t timestamp);
void *freenect_threadfunc(void *arg);

#endif /* defined(__KinectMusic__kinect__) */
