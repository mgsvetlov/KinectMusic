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

#include "../kinect.h"

#include "types.h"
#include "extractframedata.h"
#include "processframedata.h"
#include "params.h"
#include "../log/logs.h"
#include "../config/config.h"

pthread_mutex_t ExtractFrameData::depth_mutex = PTHREAD_MUTEX_INITIALIZER;

volatile int ExtractFrameData::die_kinect = 0;
volatile int ExtractFrameData::die_gesture = 0;

volatile int ExtractFrameData::MAX_KINECT_VALUE;
volatile int ExtractFrameData::MAX_NEIGHB_DIFF_COARSE  = 4;

void *ExtractFrameData::analyze_threadfunc(void *arg) {
    static volatile int frameNumExtract = 0;
    while (!ExtractFrameData::die_gesture){
        pthread_mutex_lock(&depth_mutex);
        if(frameNumExtract == Sensor::frameNum) {
            pthread_mutex_unlock(&depth_mutex);
            usleep(100);
            continue;
        }
        
        static std::clock_t t = clock();
        if(Sensor::frameNum != 0){
            std::clock_t next_t = clock();
            double elapsed_sec = double(next_t - t)/CLOCKS_PER_SEC;
            std::stringstream ss;
            ss <<"Frame num " << Sensor::frameNum << " output fps " << (Sensor::frameNum - frameNumExtract)/elapsed_sec;
            if(Sensor::frameNum - frameNumExtract > 1)
                ss << ", missed " << Sensor::frameNum - frameNumExtract - 1 << " frames";
            t = next_t;
            Logs::writeLog("gestures", ss.str());
        }
        int frameNum_ = frameNumExtract = Sensor::frameNum;
        cv::Mat mat16(Params::getMatrixHeight(), Params::getMatrixWidth(), CV_16U, Sensor::pDepthMatrix);
        pthread_mutex_unlock(&depth_mutex);
        
        ProcessFrameData processFrameData(mat16, frameNum_);
    }
    
    return NULL;
}



