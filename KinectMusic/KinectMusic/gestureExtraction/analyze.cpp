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
#include "handsHeadExtractor/handsheadextractor.h"
#include "handsHeadExtractor/handsfrompoints.h"
#include "hand/hand.h"
#include "tracking/tracking.h"
#include "gesture/gesture.h"

#ifdef USE_CSOUND
#include "../mapping/mapping.h"
#endif //USE_CSOUND

pthread_mutex_t depth_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t video_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t visualisation_mutex = PTHREAD_MUTEX_INITIALIZER;
volatile int die = 0;

volatile bool newFrame = false;

int w = 640, h = 480;
uint16_t * const depthAnalyze = new uint16_t[w*h];

int MAX_KINECT_VALUE;
int MAX_KINECT_DEPTH = 1800;
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
        
        //filter far area
        cv::Mat mat16_filt = mat16.clone();
        uint16_t* p_mat = (uint16_t*)(mat16_filt.data);
        for(size_t i = 0; i < mat16_filt.total(); i++, p_mat++)
        {
            if(*p_mat > MAX_KINECT_DEPTH)
                *p_mat = 0;
        }
        
        //resize
        cv::Mat mat16_resized;
        cv::resize(mat16_filt, mat16_resized, cv::Size(w>>BLOBS_RESIZE_POW, h>>BLOBS_RESIZE_POW));
        
        //extract person
        std::list<Blob> lBlobs;
        Blob::findBlobs(mat16_resized, lBlobs);
        
        cv::Mat matBlobs = Blob::blobs2mat(lBlobs, mat16_resized.size());
        
        //extract hands
        int filt_size(10), filt_depth(15), iterCount(2);
        HandsHeadExtractor handsHeadExtractor(matBlobs, filt_size, filt_depth, iterCount);
        cv::Mat matDst = handsHeadExtractor.extractHandsHead();
        
        std::list<Blob> lBlobs1;
        int mode (1);
        Blob::findBlobs(matDst, lBlobs1, mode);
        
        std::list<Blob> lBlobsClust;
        int xyThresh(20), depthThresh(1000);
        Blob::blobsClustering(lBlobs1, lBlobsClust, xyThresh, depthThresh);
        
        //extract hands frim points in full matrix
        int bbXY (60), bbZ(200);
        HandsFromPoints handsFromPoints(mat16, lBlobsClust, bbXY, bbZ);
        std::list<Hand> lHands = handsFromPoints.extractHandBlobs();
        
        //tracking hands
        Tracking::analyzeFrame(lHands);
        std::vector<Tracking> vTracks = Tracking::getTracksConst();
        
        //create and analyze hands tracked stream data
        Gesture::addDataToStream(vTracks);
        Gesture::analyzeGestures();
        
        cv::Mat img;
        Visualization::mat2img(mat16, img);
        Visualization::tracks2img( vTracks, img, false);
        Visualization::handsTrackedStreams2img(Gesture::getHandsTrackedStreams(), img, 60);
        
#ifdef USE_CSOUND
        Mapping::MapDirect(Gesture::getGesturesConst());
#endif //USE_CSOUND
        
        pthread_mutex_lock(&visualisation_mutex);
        Visualization::setMatImage(img);
        Visualization::setIsNeedRedraw(true);
        pthread_mutex_unlock(&visualisation_mutex);
        
    }
    return NULL;
}
