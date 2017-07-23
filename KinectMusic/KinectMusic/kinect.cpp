//
//  kinect.c
//  KinectMusic
//
//  Created by Mikhail Svetlov on 20/09/15.
//  Copyright (c) 2015 mgsvetlov. All rights reserved.
//
#include <ctime>
#include "kinect.h"
#include "config/config.h"

namespace Sensor {
    int volatile frameNum = 0;
    uint16_t* pDepthMatrix = nullptr;
    pthread_mutex_t depthMutex = PTHREAD_MUTEX_INITIALIZER;
    volatile int die_kinect = 0;
}

uint8_t *rgb_back, *rgb_mid, *rgb_front;
freenect_context *f_ctx;
freenect_device *f_dev;

freenect_video_format requested_format = FREENECT_VIDEO_RGB;
freenect_video_format current_format = FREENECT_VIDEO_RGB;
//freenect_resolution requested_resolution = FREENECT_RESOLUTION_MEDIUM;//FREENECT_RESOLUTION_HIGH;
freenect_resolution current_resolution = FREENECT_RESOLUTION_MEDIUM;

uint16_t t_gamma[2048];

int got_depth = 0;
int got_rgb = 0;
int w = 640;
int h = 480;

void depth_cb(freenect_device *dev, void *v_depth, uint32_t timestamp)
{
    //static std::clock_t t = clock();

    pthread_mutex_lock(&Sensor::depthMutex);
    ++Sensor::frameNum;
    uint16_t *p_depth1 = (uint16_t*)v_depth;
    if(Sensor::pDepthMatrix == nullptr)
        Sensor::pDepthMatrix = new uint16_t[w*h];
    uint16_t *p_depth2 = Sensor::pDepthMatrix;
    for (int i = 0; i < w * h; i++) {
        *(p_depth2++) = *(p_depth1++);
    }
    got_depth++;
    pthread_mutex_unlock(&Sensor::depthMutex);
}

void video_cb(freenect_device *dev, void *rgb, uint32_t timestamp)
{
    // swap buffers
    assert (rgb_back == rgb);
    rgb_back = rgb_mid;
    freenect_set_video_buffer(dev, rgb_back);
    rgb_mid = (uint8_t*)rgb;
    
    got_rgb++;
}

void *freenect_threadfunc(void *arg)
{
    freenect_set_led(f_dev,LED_RED);
    freenect_set_depth_callback(f_dev, depth_cb);
    /*freenect_set_video_callback(f_dev, video_cb);
    freenect_set_video_mode(f_dev, freenect_find_video_mode(current_resolution, current_format));*/
    int depthFormatIndex = Config::instance()->getDepthFormat();
    freenect_depth_format depth_format = depthFormatIndex == 0 ?FREENECT_DEPTH_REGISTERED : FREENECT_DEPTH_11BIT;
    freenect_set_depth_mode(f_dev, freenect_find_depth_mode(current_resolution, depth_format));
    /*rgb_back = (uint8_t*)malloc(freenect_find_video_mode(current_resolution, current_format).bytes);
    rgb_mid = (uint8_t*)malloc(freenect_find_video_mode(current_resolution, current_format).bytes);
    rgb_front = (uint8_t*)malloc(freenect_find_video_mode(current_resolution, current_format).bytes);
    freenect_set_video_buffer(f_dev, rgb_back);*/
    
    freenect_start_depth(f_dev);
    //freenect_start_video(f_dev);
    
    int status = 0;
    
    while (!Sensor::die_kinect && status >= 0) {
        status = freenect_process_events(f_ctx);
        /*if (requested_format != current_format || requested_resolution != current_resolution) {
            freenect_stop_video(f_dev);
            freenect_set_video_mode(f_dev, freenect_find_video_mode(requested_resolution, requested_format));
            free(rgb_back);
            free(rgb_mid);
            free(rgb_front);
            rgb_back = (uint8_t*)malloc(freenect_find_video_mode(requested_resolution, requested_format).bytes);
            rgb_mid = (uint8_t*)malloc(freenect_find_video_mode(requested_resolution, requested_format).bytes);
            rgb_front = (uint8_t*)malloc(freenect_find_video_mode(requested_resolution, requested_format).bytes);
            current_format = requested_format;
            current_resolution = requested_resolution;
            freenect_set_video_buffer(f_dev, rgb_back);
            freenect_start_video(f_dev);
        }*/
    }
    
    if (status < 0) {
        printf("Something went terribly wrong.  Aborting.\n");
        return NULL;
    }
    
    printf("\nshutting down streams...\n");
    
    freenect_stop_depth(f_dev);
    freenect_stop_video(f_dev);
    
    freenect_close_device(f_dev);
    freenect_shutdown(f_ctx);
    
    delete[] Sensor::pDepthMatrix;
    
    printf("-- done!\n");
    return NULL;
}