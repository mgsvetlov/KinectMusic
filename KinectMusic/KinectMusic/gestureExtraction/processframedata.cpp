//
//  processframedata.cpp
//  KinectMusic
//
//  Created by Mikhail Svetlov on 22/07/17.
//  Copyright © 2017 mgsvetlov. All rights reserved.
//
#include <sys/stat.h>
#include "processframedata.h"
#include "params.h"
#include "blobs/blobsfabrique.hpp"
#include "convex3d/convex3d.h"
#include "share.h"
#include "../log/logs.h"
#include "../config/config.h"
#include "visualization/visualization.h"


pthread_mutex_t ProcessFrameData::visualisation_mutex = PTHREAD_MUTEX_INITIALIZER;

ProcessFrameData::ProcessFrameData(cv::Mat mat, int frameNum) :
mat(mat),
frameData(frameNum)
{
    if(!Params::getIsInit())
        Params::Init();
    filterFar();
    resize();
    createBlobsAndBorders();
    learnTest();
    tracking();
    shareFrameData();
    visualize();

}

void ProcessFrameData::filterFar(){
    //filter far area
    static const int maxKinectDepth(Params::getMaxKinectDepth());
    static const int minKinectDepth(Params::getMinKinectDepth());
    matFilt = mat.clone();
    uint16_t* p_mat = (uint16_t*)(matFilt.data);
    for(size_t i = 0; i < matFilt.total(); i++, p_mat++)
    {
        uint16_t val = *p_mat;
        if( val > maxKinectDepth || val < minKinectDepth)
            *p_mat = maxKinectDepth;
    }
}

void ProcessFrameData::resize(){
    //resize
    const static int blobsResizePow(Params::getBlobsResizePow());
    cv::resize(matFilt, matResized, cv::Size(Params::getMatrixWidth()>>blobsResizePow, Params::getMatrixHeight()>>blobsResizePow));
}

void ProcessFrameData::createBlobsAndBorders(){
    //extract all the blobs up to person
    BlobsFabrique<BlobPrim> blobsFabrique(0, matResized);
    auto& blobs = blobsFabrique.getBlobs();
    frameData.averagedBodyPoint = blobsFabrique.getAveragedBodyPoint();
    
    //extract 3d convexes
    matBlobsPrim = BlobPrim::blobs2mat(blobs, matResized.size());
    static int filt_size(matResized.cols / 20), filt_depth(matResized.cols / 10), core_half_size(2);
    cv::Mat matDst = Convex3d::extractConvexities(matBlobsPrim, filt_size, filt_depth, core_half_size);
    BlobsFabrique<BlobPrim> blobsFabrique1(1, matDst);
    
    //create blobs extended and borders
    blobsFabrique1.constructBlobsExt(matFilt, blobsExt);
}

void ProcessFrameData::learnTest(){
    auto it = blobsExt.begin();
    while(it != blobsExt.end()){
        it->computeFeatures(frameData.averagedBodyPoint);
        const auto& features = it->getFeatures();
        if(Config::instance()->getIsLearning()){
            std::stringstream ss;
            ss << "features: ";
            for(auto& feature : features)
                ss << feature << " ";
            ss << frameData.frameNum;
            Logs::writeLog("ann/train/", ss.str());
        }
        ++it;
    }
}

void ProcessFrameData::tracking(){
    //tracking hands
     Track::analyzeFrame(blobsExt);
    
}

void ProcessFrameData::shareFrameData(){
    if(Config::instance()->getIsCsound()){
        const auto& tracks = Track::getTracksConst();
        for(const auto& track : tracks){
            const std::list<HandData>& handHistory = track.getHandHistoryConst();
            if(!handHistory.empty())
                frameData.data.push_back(handHistory.back());
            else
                frameData.data.push_back(HandData());
        }
        if(!Share::share(frameData)){
            Logs::writeLog("gestures", "Share error!");
        }
    }
}

void ProcessFrameData::visualize(){
    if(Config::instance()->getIsVisualisation()){
        cv::Mat img;
        //cv::resize(matBlobsPrim, matBlobsPrim, cv::Size(Params::getMatrixWidth(), Params::getMatrixHeight()));
        //Visualization::mat2img(matBlobsPrim, img);
        Visualization::mat2img(mat, img);
        for(auto& blob: blobsExt)
            blob.SetFrameNum(frameData.frameNum);
        const auto& point = frameData.averagedBodyPoint;
        cv::circle(img, cv::Point(point.x << Params::getBlobsResizePow(), point.y << Params::getBlobsResizePow()), 5,  cv::Scalar(255, 0, 255), -1);
        Visualization::blobs2img( blobsExt, img, false);
        Visualization::tracks2img(Track::getTracksConst(), img);
        //Visualization::gestures2img(GestureFabrique::getGestures(), img);
        
        cv::flip(img, img, 1);
        std::stringstream ss;
        ss << frameData.frameNum;
        Visualization::drawText(img, ss.str(), 1.0, 1, cv::Scalar(0,0,255), cv::Point2f(0.5, 0.15));
        if(Config::instance()->getIsLearning()) {
            static std::string dirName;
            static int status = -1;
            if(dirName == ""){
                dirName = "ann/train/" + Logs::getCurrentTime();
                status = mkdir(dirName.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
            }
            if(status == 0)
                cv::imwrite( dirName + "/" + ss.str() + ".jpg", img );
        }
        pthread_mutex_lock(&visualisation_mutex);
        Visualization::setMatImage(img);
        Visualization::setIsNeedRedraw(true);
        pthread_mutex_unlock(&visualisation_mutex);
    }
}