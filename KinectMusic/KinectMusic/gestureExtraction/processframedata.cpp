//
//  processframedata.cpp
//  KinectMusic
//
//  Created by Mikhail Svetlov on 22/07/17.
//  Copyright © 2017 mgsvetlov. All rights reserved.
//

#include "processframedata.h"
#include "blobs/blobsfabrique.hpp"
#include "convex3d/convex3d.h"
#include "share.h"
#include "../log/logs.h"
#include "../config/config.h"
#include "visualization/visualization.h"

int ProcessFrameData::MAX_KINECT_DEPTH = 2000;
int ProcessFrameData::MIN_KINECT_DEPTH = 800;
int ProcessFrameData::BLOBS_RESIZE_POW;
int ProcessFrameData::BLOB_MIN_SIZE;
int ProcessFrameData::BLOB_MIN_SIZE_LAST;
int ProcessFrameData::BLOB_EXT_MAX_SIZE;

pthread_mutex_t ProcessFrameData::visualisation_mutex = PTHREAD_MUTEX_INITIALIZER;

void ProcessFrameData::Init(int w){
    BLOBS_RESIZE_POW = w == 640 ? 3 : 2;
    BLOB_MIN_SIZE = (w >> BLOBS_RESIZE_POW)  * 0.15625 * 0.5;
    BLOB_MIN_SIZE_LAST = (w >> BLOBS_RESIZE_POW)  * 18.75;
    BLOB_EXT_MAX_SIZE = w * 6.4;
}

ProcessFrameData::ProcessFrameData(cv::Mat mat, int frameNum) :
mat(mat)
{
    filterFar();
    resize();

    //extract all the blobs up to person
    BlobsFabrique<BlobPrim> blobsFabrique(0, matResized);
    auto& blobs = blobsFabrique.getBlobs();
    
    //extract 3d convexes
    cv::Mat matBlobs = BlobPrim::blobs2mat(blobs, matResized.size());
    static int filt_size(matResized.cols / 20), filt_depth(matResized.cols / 10), core_half_size(2);
    cv::Mat matDst = Convex3d::extractConvexities(matBlobs, filt_size, filt_depth, core_half_size);
    BlobsFabrique<BlobPrim> blobsFabrique1(1, matDst);
    std::list<BlobFinal> blobsExt;
    blobsFabrique1.constructBlobsExt(matFilt, blobsExt);
    
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
    
    if(Config::instance()->getIsVisualisation()){
        cv::Mat img;
        Visualization::mat2img(mat, img);
        for(auto& blob: blobsExt)
            blob.SetFrameNum(frameNum);
        Visualization::blobs2img( blobsExt, img, true);
        cv::flip(img, img, 1);
        std::stringstream ss;
        ss << frameNum;
        Visualization::drawText(img, ss.str(), 1.0, 1, cv::Scalar(0,0,255), cv::Point2f(0.5, 0.15));
        
        //Visualization::gestures2img(GestureFabrique::getGestures(), img);
        
        pthread_mutex_lock(&visualisation_mutex);
        Visualization::setMatImage(img);
        Visualization::setIsNeedRedraw(true);
        pthread_mutex_unlock(&visualisation_mutex);
    }
    
}

void ProcessFrameData::filterFar(){
    //filter far area
    matFilt = mat.clone();
    uint16_t* p_mat = (uint16_t*)(matFilt.data);
    for(size_t i = 0; i < matFilt.total(); i++, p_mat++)
    {
        if( *p_mat > ProcessFrameData::MAX_KINECT_DEPTH || *p_mat < ProcessFrameData::MIN_KINECT_DEPTH)
            *p_mat = ProcessFrameData::MAX_KINECT_DEPTH;
    }
}

void ProcessFrameData::resize(){
    //resize
    cv::resize(matFilt, matResized, cv::Size(ExtractFrameData::w>>ProcessFrameData::BLOBS_RESIZE_POW, ExtractFrameData::h>>ProcessFrameData::BLOBS_RESIZE_POW));
}