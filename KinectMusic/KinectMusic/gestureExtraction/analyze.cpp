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
#include <sstream>
#include <algorithm>

#include "types.h"
#include "visualization/visualization.h"
#include "analyze.h"
#include "blobs/blob.h"
#include "handsExtractor/handsextractor.h"
#include "hand/hand.h"
#include "tracking/tracking.h"
#include "gesture/gesturefabrique.h"
#include "share.h"
#include "../log/logs.h"
#include "../config/config.h"


pthread_mutex_t depth_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t video_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t visualisation_mutex = PTHREAD_MUTEX_INITIALIZER;

volatile int die_kinect = 0;
volatile int die_gesture = 0;

volatile bool newFrame = false;

int w = 640, h = 480;
uint16_t * const depthAnalyze = new uint16_t[w*h];

int MAX_KINECT_VALUE;
int MAX_KINECT_DEPTH = 1800;
int MIN_KINECT_DEPTH = 1000;
int BLOBS_RESIZE_POW  = 2;
int BLOB_MIN_SIZE = 25;
int BLOB_MIN_SIZE_LAST = 3000;
int MAX_NEIGHB_DIFF_COARSE  = 4;
int MAX_NEIGHB_DIFF_FINE  = 1;

volatile int frameNum = 0;
volatile int frameNum_analyze = 0;
volatile int analyzeThreadId = 0;

void *analyze_threadfunc(void *arg) {
    
    while (!die_gesture){
        pthread_mutex_lock(&depth_mutex);
        if(frameNum_analyze == frameNum) {
            pthread_mutex_unlock(&depth_mutex);
            usleep(100);
            continue;
        }
        frameNum_analyze = frameNum;
        cv::Mat mat16(h, w, CV_16U, depthAnalyze);
        pthread_mutex_unlock(&depth_mutex);
        
        //filter far area
        cv::Mat mat16_filt = mat16.clone();
        uint16_t* p_mat = (uint16_t*)(mat16_filt.data);
        for(size_t i = 0; i < mat16_filt.total(); i++, p_mat++)
        {
            if(*p_mat > MAX_KINECT_DEPTH || *p_mat < MIN_KINECT_DEPTH)
                *p_mat = 0;
        }
        
        //resize
        cv::Mat mat16_resized;
        cv::resize(mat16_filt, mat16_resized, cv::Size(w>>BLOBS_RESIZE_POW, h>>BLOBS_RESIZE_POW));
        
        //extract person
        std::list<Blob> lBlobs;
        int body_depth = Blob::findBlobs(mat16_resized, lBlobs);
        
        cv::Mat matBlobs = Blob::blobs2mat(lBlobs, mat16_resized.size());
        
        //extract hands
        static int filt_size(8), filt_depth(15), core_half_size(2);
        cv::Mat matDst = HandsExtractor::extractHands(matBlobs, filt_size, filt_depth, core_half_size);
       
        std::list<Blob> lBlobs1;
        int mode (1);
        Blob::findBlobs(matDst, lBlobs1, mode);
        
        std::list<Blob> lBlobsClust;
        int xyThresh(20), depthThresh(100);
        Blob::blobsClustering(lBlobs1, lBlobsClust, xyThresh, depthThresh);
        
        if(!lBlobsClust.empty()){
            int minDistToBody(150);
            Blob::filterNearBody(lBlobsClust,  body_depth, minDistToBody);
            Blob::sort(lBlobsClust);
        }
        Logs::writeLog("gestures", "frame");
        std::list<Hand> lHands;
        if(!lBlobsClust.empty()){
            for(auto& blob : lBlobsClust){
                blob.originalData(mat16_filt);
                blob.computeAngle();
                lHands.push_back(Hand(blob, mat16_filt.size()));
            }
        }
        
        //tracking hands
        Track::analyzeFrame(lHands);
        std::vector<Track> vTracks = Track::getTracksConst();
       
        //create and analyze hands tracked stream data
        FrameData frameData = GestureFabrique::extractGestures(vTracks);
        frameData.bodyDepth = body_depth;
        
        if(Config::instance()->getIsCsound()){
            if(!Share::share(frameData)){
                Logs::writeLog("gestures", "Share error!");
                break;
            }
        }
#ifdef WRITE_LOGS
        std::cout << frameData;
#endif //WRITE_LOGS
        if(Config::instance()->getIsVisualisation()){
            cv::Mat img;
            Visualization::mat2img(mat16, img);
            Visualization::blobs2img( lBlobsClust, img, false);
            Visualization::gestures2img(GestureFabrique::getGestures(), img);
            
            pthread_mutex_lock(&visualisation_mutex);
            Visualization::setMatImage(img);
            Visualization::setIsNeedRedraw(true);
            pthread_mutex_unlock(&visualisation_mutex);
        }
    }
    
    return NULL;
}

std::ostream& operator << (std::ostream& os, const FrameData& frameData){
    std::stringstream ss;
    ss << frameData.frameNum << " ";
    for(auto& gestureData: frameData.data){
        ss << (gestureData.phase == START_GESTURE_VALUE ? "START" : gestureData.phase ==INSIDE_GESTURE_VALUE ? "INSIDE" :
               gestureData.phase ==
               END_GESTURE_VALUE ? "END" :
               "NO_DATA")
        << " " << gestureData.point << " ";
    }
    Logs::writeLog("gestures", ss.str());
    return os;
}


