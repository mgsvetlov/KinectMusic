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
#include <ctime>

#include "types.h"
#include "visualization/visualization.h"
#include "extractframedata.h"
#include "blobs/blobsfabrique.hpp"
#include "blobs/cells/cell.h"
#include "convex3d/convex3d.h"
#include "hand/hand.h"
#include "tracking/tracking.h"
#include "gesture/gesturefabrique.h"
#include "share.h"
#include "../log/logs.h"
#include "../config/config.h"

uint16_t * const ExtractFrameData::depthAnalyze = new uint16_t[w*h];

pthread_mutex_t ExtractFrameData::depth_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t ExtractFrameData::video_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t ExtractFrameData::visualisation_mutex = PTHREAD_MUTEX_INITIALIZER;

volatile int ExtractFrameData::die_kinect = 0;
volatile int ExtractFrameData::die_gesture = 0;

volatile int ExtractFrameData::frameNum = 0;
volatile int ExtractFrameData::frameNum_analyze = 0;

volatile int ExtractFrameData::MAX_KINECT_VALUE;
volatile int ExtractFrameData::MAX_NEIGHB_DIFF_COARSE  = 4;

volatile int ExtractFrameData::w = 640, ExtractFrameData::h = 480; //why should be initialized here???
int MAX_KINECT_DEPTH = 2000;
int MIN_KINECT_DEPTH = 800;
int BLOBS_RESIZE_POW;
int BLOB_MIN_SIZE;
int BLOB_MIN_SIZE_LAST;
int BLOB_EXT_MAX_SIZE;

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
            BLOBS_RESIZE_POW = w == 640 ? 3 : 2;
            BLOB_MIN_SIZE = (w >> BLOBS_RESIZE_POW)  * 0.15625 * 0.5;
            BLOB_MIN_SIZE_LAST = (w >> BLOBS_RESIZE_POW)  * 18.75;
            BLOB_EXT_MAX_SIZE = w * 6.4;
        }
        int frameNum_ = frameNum_analyze = frameNum;
        cv::Mat mat16(h, w, CV_16U, depthAnalyze);
        pthread_mutex_unlock(&depth_mutex);
        
        //filter far area
        cv::Mat mat16_filt = mat16.clone();
        uint16_t* p_mat = (uint16_t*)(mat16_filt.data);
        for(size_t i = 0; i < mat16_filt.total(); i++, p_mat++)
        {
            if( *p_mat > MAX_KINECT_DEPTH || *p_mat < MIN_KINECT_DEPTH)
                *p_mat = MAX_KINECT_DEPTH;
        }
        
        //resize
        cv::Mat mat16_resized;
        cv::resize(mat16_filt, mat16_resized, cv::Size(w>>BLOBS_RESIZE_POW, h>>BLOBS_RESIZE_POW));
        
        //extract all the blobs up to person
        BlobsFabrique<BlobPrim> blobsFabrique(0, mat16_resized);
        auto& blobs = blobsFabrique.getBlobs();
        
        //extract 3d convexes
        cv::Mat matBlobs = BlobPrim::blobs2mat(blobs, mat16_resized.size());
        static int filt_size(mat16_resized.cols / 20), filt_depth(mat16_resized.cols / 10), core_half_size(2);
        cv::Mat matDst = Convex3d::extractConvexities(matBlobs, filt_size, filt_depth, core_half_size);
        BlobsFabrique<BlobPrim> blobsFabrique1(1, matDst);
        std::list<BlobFinal> blobsExt;
        blobsFabrique1.constructBlobsExt(mat16.clone(), blobsExt);

        //tracking hands
        /*Track::analyzeFrame(lHands);
        std::vector<Track> vTracks = Track::getTracksConst();
        std::vector<Blob*> vp_Blobs;
        
        //create and analyze hands tracked stream data
        FrameData frameData = GestureFabrique::extractGestures(vTracks);
        frameData.bodyDepth = blobsFabrique.getBodyDepth();// body_depth;
        
        if(Config::instance()->getIsCsound()){
            if(!Share::share(frameData)){
                Logs::writeLog("gestures", "Share error!");
                break;
            }
        }*/
#ifdef WRITE_LOGS
        std::cout << frameData;
#endif //WRITE_LOGS
        if(Config::instance()->getIsVisualisation()){
            cv::Mat img;
            Visualization::mat2img(mat16, img);
            for(auto& blob: blobsExt)
                blob.SetFrameNum(frameNum_);
            Visualization::blobs2img( blobsExt, img, true);
            cv::flip(img, img, 1);
            std::stringstream ss;
            ss << frameNum_;
            Visualization::drawText(img, ss.str(), 1.0, 1, cv::Scalar(0,0,255), cv::Point2f(0.5, 0.15));
            
            //Visualization::gestures2img(GestureFabrique::getGestures(), img);
            
            pthread_mutex_lock(&visualisation_mutex);
            Visualization::setMatImage(img);
            Visualization::setIsNeedRedraw(true);
            pthread_mutex_unlock(&visualisation_mutex);
        }
    }
    
    return NULL;
}

/*std::ostream& operator << (std::ostream& os, const FrameData& frameData){
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
}*/

