//
//  analyze.c
//  KinectMusic
//
//  Created by Mikhail Svetlov on 17/09/16.
//  Copyright © 2016 mgsvetlov. All rights reserved.
//

#include <sys/mman.h>   /* shared memory and mmap() */
#include <unistd.h>     /* for getopt() */
#include <errno.h>      /* errno and perror */
#include <fcntl.h>      /* O_flags */
#include <iostream>
#include <ctime>

#include "types.h"
#include "extractframedata.h"
#include "processframedata.h"
#include "../log/logs.h"
#include "../config/config.h"

uint16_t * const ExtractFrameData::depthAnalyze = new uint16_t[w*h];

pthread_mutex_t ExtractFrameData::depth_mutex = PTHREAD_MUTEX_INITIALIZER;

volatile int ExtractFrameData::die_kinect = 0;
volatile int ExtractFrameData::die_gesture = 0;

volatile int ExtractFrameData::frameNum = 0;
volatile int ExtractFrameData::frameNum_analyze = 0;

volatile int ExtractFrameData::MAX_KINECT_VALUE;
volatile int ExtractFrameData::MAX_NEIGHB_DIFF_COARSE  = 4;

volatile int ExtractFrameData::w = 640, ExtractFrameData::h = 480; //why should be initialized here???


void *ExtractFrameData::analyze_threadfunc(void *arg) {
    while (!ExtractFrameData::die_gesture){
        pthread_mutex_lock(&depth_mutex);
        if(frameNum_analyze == frameNum) {
            pthread_mutex_unlock(&depth_mutex);
            usleep(100);
            continue;
        }
        
        static std::clock_t t = clock();
        if(frameNum != 0){
            std::clock_t next_t = clock();
            double elapsed_sec = double(next_t - t)/CLOCKS_PER_SEC;
            std::stringstream ss;
            ss <<"Frame num " << frameNum << " output fps " << (frameNum- frameNum_analyze)/elapsed_sec;
            if(frameNum - frameNum_analyze > 1)
                ss << ", missed " << frameNum - frameNum_analyze - 1 << " frames";
            t = next_t;
            Logs::writeLog("gestures", ss.str());
        }
        if(frameNum_analyze == 0){
            ProcessFrameData::Init(w);
        }
        int frameNum_ = frameNum_analyze = frameNum;
        cv::Mat mat16(h, w, CV_16U, depthAnalyze);
        pthread_mutex_unlock(&depth_mutex);
        
        ProcessFrameData processFrameData(mat16, frameNum_);
    }
    
    return NULL;
}



