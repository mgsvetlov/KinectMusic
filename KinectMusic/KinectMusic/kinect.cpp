//
//  kinect.c
//  KinectMusic
//
//  Created by Mikhail Svetlov on 20/09/15.
//  Copyright (c) 2015 mgsvetlov. All rights reserved.
//
#include <ctime>
#include "kinect.h"
#include  "gestureExtraction/analyze.h"
#include  "gestureExtraction/types.h"


uint8_t *rgb_back, *rgb_mid, *rgb_front;
//uint8_t *depth_mid;
freenect_context *f_ctx;
freenect_device *f_dev;


freenect_video_format requested_format = FREENECT_VIDEO_RGB;
freenect_video_format current_format = FREENECT_VIDEO_RGB;
freenect_resolution requested_resolution = FREENECT_RESOLUTION_MEDIUM;//FREENECT_RESOLUTION_HIGH;
freenect_resolution current_resolution = FREENECT_RESOLUTION_MEDIUM;//FREENECT_RESOLUTION_HIGH;

uint16_t t_gamma[2048];

int got_depth = 0;
int got_rgb = 0;


void depth_cb(freenect_device *dev, void *v_depth, uint32_t timestamp)
{
    static std::clock_t t = clock();
    /*if(frameNum % 30 == 0){
        std::clock_t next_t = clock();
        if(frameNum) {
            double elapsed_sec = double(next_t - t)/CLOCKS_PER_SEC;
            std::cout << "kinect fps " <<30./elapsed_sec << std::endl;
        }
        t = next_t;
    }*/
    //std::cout << "kinect frameNum " << frameNum << std::endl;
    
    pthread_mutex_lock(&depth_mutex);
    ++frameNum;
    uint16_t *p_depth1 = (uint16_t*)v_depth;
    uint16_t *p_depth2 = depthAnalyze;
    for (int i = 0; i < w * h; i++) {
        *(p_depth2++) = *(p_depth1++);
    }
    /*for (i=0; i< w * h; i++) {
        int pval = t_gamma[depth[i]];
        int lb = pval & 0xff;
        switch (pval>>8) {
            case 0:
            depth_mid[3*i+0] = 255;
            depth_mid[3*i+1] = 255-lb;
            depth_mid[3*i+2] = 255-lb;
            break;
            case 1:
            depth_mid[3*i+0] = 255;
            depth_mid[3*i+1] = lb;
            depth_mid[3*i+2] = 0;
            break;
            case 2:
            depth_mid[3*i+0] = 255-lb;
            depth_mid[3*i+1] = 255;
            depth_mid[3*i+2] = 0;
            break;
            case 3:
            depth_mid[3*i+0] = 0;
            depth_mid[3*i+1] = 255;
            depth_mid[3*i+2] = lb;
            break;
            case 4:
            depth_mid[3*i+0] = 0;
            depth_mid[3*i+1] = 255-lb;
            depth_mid[3*i+2] = 255;
            break;
            case 5:
            depth_mid[3*i+0] = 0;
            depth_mid[3*i+1] = 0;
            depth_mid[3*i+2] = 255-lb;
            break;
            default:
            depth_mid[3*i+0] = 0;
            depth_mid[3*i+1] = 0;
            depth_mid[3*i+2] = 0;
            break;
        }
    }*/
    got_depth++;
    newFrame = true;
    pthread_mutex_unlock(&depth_mutex);
}

void video_cb(freenect_device *dev, void *rgb, uint32_t timestamp)
{
    pthread_mutex_lock(&video_mutex);
    
    // swap buffers
    assert (rgb_back == rgb);
    rgb_back = rgb_mid;
    freenect_set_video_buffer(dev, rgb_back);
    rgb_mid = (uint8_t*)rgb;
    
    got_rgb++;
    pthread_mutex_unlock(&video_mutex);
}

void *freenect_threadfunc(void *arg)
{
    freenect_set_led(f_dev,LED_RED);
    freenect_set_depth_callback(f_dev, depth_cb);
    freenect_set_video_callback(f_dev, video_cb);
    freenect_set_video_mode(f_dev, freenect_find_video_mode(current_resolution, current_format));
    freenect_depth_format depth_format = FREENECT_DEPTH_REGISTERED; //FREENECT_DEPTH_11BIT
    switch(depth_format){
        case FREENECT_DEPTH_11BIT:
            MAX_KINECT_VALUE  = FREENECT_DEPTH_RAW_MAX_VALUE;
            MAX_NEIGHB_DIFF_COARSE  = 4;
            MAX_NEIGHB_DIFF_FINE  = 1;
            break;
        case FREENECT_DEPTH_REGISTERED:
            MAX_KINECT_VALUE  = FREENECT_DEPTH_MM_MAX_VALUE;
            MAX_NEIGHB_DIFF_COARSE  = 80;
            MAX_NEIGHB_DIFF_FINE  = 20;
            break;
            break;
            
    }
    
    freenect_set_depth_mode(f_dev, freenect_find_depth_mode(FREENECT_RESOLUTION_MEDIUM, depth_format));
    rgb_back = (uint8_t*)malloc(freenect_find_video_mode(current_resolution, current_format).bytes);
    rgb_mid = (uint8_t*)malloc(freenect_find_video_mode(current_resolution, current_format).bytes);
    rgb_front = (uint8_t*)malloc(freenect_find_video_mode(current_resolution, current_format).bytes);
    freenect_set_video_buffer(f_dev, rgb_back);
    
    freenect_start_depth(f_dev);
    freenect_start_video(f_dev);
    
    int status = 0;
    
    while (!die && status >= 0) {
        status = freenect_process_events(f_ctx);
        if (requested_format != current_format || requested_resolution != current_resolution) {
            freenect_stop_video(f_dev);
            freenect_set_video_mode(f_dev, freenect_find_video_mode(requested_resolution, requested_format));
            pthread_mutex_lock(&video_mutex);
            free(rgb_back);
            free(rgb_mid);
            free(rgb_front);
            rgb_back = (uint8_t*)malloc(freenect_find_video_mode(requested_resolution, requested_format).bytes);
            rgb_mid = (uint8_t*)malloc(freenect_find_video_mode(requested_resolution, requested_format).bytes);
            rgb_front = (uint8_t*)malloc(freenect_find_video_mode(requested_resolution, requested_format).bytes);
            current_format = requested_format;
            current_resolution = requested_resolution;
            pthread_mutex_unlock(&video_mutex);
            freenect_set_video_buffer(f_dev, rgb_back);
            freenect_start_video(f_dev);
        }
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
    
    printf("-- done!\n");
    return NULL;
}