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
#include <queue>

#include "types.h"
#include "visualization/visualization.h"
#include "analyze.h"
#include "blobs/blob.h"
#include "blobs/blobsmask.h"
#include "handsHeadExtractor/handsheadextractor.h"

pthread_mutex_t depth_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t video_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t visualisation_mutex = PTHREAD_MUTEX_INITIALIZER;
volatile int die = 0;

volatile bool newFrame = false;

int w = 640, h = 480;
uint16_t * const depthAnalyze = new uint16_t[w*h];

int MAX_KINECT_VALUE  = 2047;
int BLOBS_RESIZE_POW  = 2;
int BLOB_MIN_SIZE = 25;
int BLOB_MIN_SIZE_LAST = 3000;
int MAX_NEIGHB_DIFF_COARSE  = 4;
int MAX_NEIGHB_DIFF_FINE  = 1;

volatile int frameNum = 0;
volatile int frameNum_analyze = 0;
volatile int analyzeThreadId = 0;

void *analyze_threadfunc(void *arg) {

    int frameNum_analyze_local (0);
    while (!die){
        pthread_mutex_lock(&depth_mutex);
        if(frameNum_analyze == frameNum) {
            pthread_mutex_unlock(&depth_mutex);
            usleep(100);
            continue;
        }
        frameNum_analyze_local = frameNum_analyze = frameNum;
        cv::Mat mat16(h, w, CV_16U, depthAnalyze);
        pthread_mutex_unlock(&depth_mutex);
        
        cv::Mat mat16_resized;
        cv::resize(mat16, mat16_resized, cv::Size(w>>BLOBS_RESIZE_POW, h>>BLOBS_RESIZE_POW));
        
        std::list<Blob> lBlobs;
        Blob::findBlobs(mat16_resized, lBlobs);
        
        cv::Mat matBlobs = Blob::blobs2mat(lBlobs, mat16_resized.size());
        
        int filt_size(10), filt_depth(18), iterCount(2);
        HandsHeadExtractor handsHeadExtractor(matBlobs, filt_size, filt_depth, iterCount);
        cv::Mat matDst = handsHeadExtractor.extractHandsHead();
        
        std::list<Blob> lBlobs1;
        int mode (1);
        Blob::findBlobs(matDst, lBlobs1, mode);
        
        std::list<Blob> lBlobsClust;
        int xyThresh(20), depthThresh(1000);
        Blob::blobsClustering(lBlobs1, lBlobsClust, xyThresh, depthThresh);
   
        Blob::extendBlobs(matBlobs, lBlobsClust);
        
        cv::Mat imgResized = Visualization::blobs2img_mark(lBlobsClust, matDst.size());
        //cv::Mat imgResized = Visualization::mat2img(mat);
        
        pthread_mutex_lock(&visualisation_mutex);
        Visualization::setMatImage(imgResized);
        Visualization::setIsNeedRedraw(true);
        pthread_mutex_unlock(&visualisation_mutex);
        
    }
    return NULL;
}
