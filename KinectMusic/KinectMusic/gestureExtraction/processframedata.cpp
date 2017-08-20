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
    markEdges();
    resize();
    createBlobsAndBorders();
    tracking();
    shareFrameData();
    visualize();

}

void ProcessFrameData::filterFar(){
    //filter far area
    static const int maxKinectDepth(Params::getMaxKinectDepth());
    static const int minKinectDepth(Params::getMinKinectDepth());
    matFilt = mat.clone();
    uint16_t* p_matFilt = (uint16_t*)(matFilt.data);
    for(size_t i = 0; i < matFilt.total(); i++, p_matFilt++)
    {
        uint16_t val = *p_matFilt;
        if( val > maxKinectDepth || val < minKinectDepth)
            *p_matFilt = maxKinectDepth;
    }
}

void ProcessFrameData::markEdges(){
    uint16_t* p_mat = (uint16_t*)(mat.data);
    uint16_t* p_matFilt = (uint16_t*)(matFilt.data);
    uint16_t* p_mat1 = (uint16_t*)(mat.data) + mat.total() - 1;
    uint16_t* p_matFilt1 = (uint16_t*)(matFilt.data) + matFilt.total() - 1;
    for(int j = 0; j < mat.rows; ++j){
        for(int i = 0; i < mat.cols; ++i){
            if(*(p_mat +i))
                break;
            *(p_matFilt + i) = 0;
        }
        p_mat += mat.cols;
        p_matFilt += mat.cols;
        for(int i = 0; i < mat.cols; ++i){
            if(*(p_mat1 - i))
                break;
            *(p_matFilt1 - i) = 0;
        }
        p_mat1 -= mat.cols;
        p_matFilt1 -= mat.cols;
    }
    for(int i = 0; i < mat.cols; ++i){
        uint16_t* p_mat = (uint16_t*)(mat.data) + i;
        uint16_t* p_matFilt = (uint16_t*)(matFilt.data) + i;
        for(int j = 0; j < mat.rows; ++j, p_mat += mat.cols, p_matFilt += mat.cols){
            if(*p_mat)
                break;
            *p_matFilt = 0;
        }
        p_mat = (uint16_t*)(mat.data) + mat.total() - 1 - i;
        p_matFilt = (uint16_t*)(matFilt.data) + mat.total() - 1 - i;
        for(int j = 0; j < mat.rows; ++j, p_mat -= mat.cols, p_matFilt -= mat.cols){
            if(*p_mat)
                break;
            *p_matFilt = 0;
        }
    }
}

void ProcessFrameData::resize(){
    //resize
    const static int blobsResizePow(Params::getBlobResizePow());
    cv::resize(matFilt, matResized, cv::Size(Params::getMatrixWidth()>>blobsResizePow, Params::getMatrixHeight()>>blobsResizePow));
}

void ProcessFrameData::createBlobsAndBorders(){
    //matConvex = matResized.clone();
    //extract all the blobs up to person
    BlobsFabrique<BlobPrim> blobsFabrique(0, matResized);
    auto& blobs = blobsFabrique.getBlobs();
    if(blobs.size() == 0)
        return;
    frameData.averagedBodyPoint = blobsFabrique.getAveragedBodyPoint();
    //extract 3d convexes
    cv::Mat matDst = Convex3d::extractConvexities(matResized, Params::getConvex3dFilterSize(), Params::getConvex3dFilterDepth(), Params::getConvex3dCoreHalfSize(), Params::getConvex3dCountFalsePercent());
    uint16_t* p_mat = (uint16_t*)(matDst.data);
    for(int i = 0; i < matDst.total(); ++i, ++p_mat){
        if(*p_mat > frameData.averagedBodyPoint.z)
            *p_mat = 0;
    }
    BlobsFabrique<BlobPrim> blobsFabrique1(1, matDst);
    //auto& blobsClust = blobsFabrique1.getBlobs();
    //matConvex = BlobPrim::blobs2mat(blobsClust, matResized.size());
    //create blobs extended and borders
    frameData.averagedBodyPoint.x <<= Params::getBlobResizePow();
    frameData.averagedBodyPoint.y <<= Params::getBlobResizePow();
    blobsFabrique1.constructBlobsExt(matFilt, blobsExt, frameData.averagedBodyPoint);
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
        //cv::resize(matConvex, matConvex, mat.size());
        Visualization::mat2img(matFilt, img);
        const auto& point = frameData.averagedBodyPoint;
        cv::circle(img, cv::Point(point.x , point.y), 5,  cv::Scalar(255, 0, 255), -1);
        Visualization::blobs2img( blobsExt, img, false);
        Visualization::tracks2img(Track::getTracksConst(), img);
        //Visualization::gestures2img(GestureFabrique::getGestures(), img);
        
        cv::flip(img, img, 1);
        std::stringstream ss;
        ss << frameData.frameNum;
        Visualization::drawText(img, ss.str(), 1.0, 1, cv::Scalar(0,0,255), cv::Point2f(0.5, 0.15));
        
        //write jpgs to disk
        if(Config::instance()->getIsImwrite()) {
            static std::string dirName;
            static int status = -1;
            if(dirName == ""){
                dirName = "ann/train/" + Logs::getCurrentTime();
                status = mkdir(dirName.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
            }
            if(status == 0){
                cv::imwrite( dirName + "/" + ss.str() + ".jpg", img );
                /*int blobCount(-1);
                for(auto& blob : blobsExt){
                    ++blobCount;
                    const auto& projPoints = blob.getBorderPtr()->getProjPointsConst();
                    float maxX(-FLT_MAX), maxY(-FLT_MAX);
                    if(projPoints.empty())
                        continue;
                    for(const auto& p : projPoints){
                        if(p.x > maxX)
                            maxX = p.x;
                        if(p.y > maxY)
                            maxY = p.y;
                    }
                    int w = maxX + 2;
                    int h = maxY + 2;
                    cv::Mat img = cv::Mat_<unsigned char>::zeros(cv::Size(w, h));
                    unsigned char* p_img = (unsigned char*)(img.data);
                    int count(0);
                    for(auto& p : projPoints){
                        *(p_img + static_cast<int>(p.y + 1)* w + static_cast<int>(p.x + 1)) = 255;
                        if(count)
                        cv::line(img, cv::Point(p.x + 1, p.y + 1), cv::Point(projPoints[count-1].x+1,projPoints[count-1].y+1), cv::Scalar (255), 1);
                        count++;
                    }
                    std::stringstream ss;
                    ss << frameData.frameNum << "_" << blobCount;
                    cv::imwrite( dirName + "/" + ss.str() + ".jpg", img );
                }*/
            }
        }
        
        pthread_mutex_lock(&visualisation_mutex);
        Visualization::setMatImage(img);
        Visualization::setIsNeedRedraw(true);
        pthread_mutex_unlock(&visualisation_mutex);
    }
}