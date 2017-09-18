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
#include "integral/integralimage.h"
#include "integral/integralgrid.h"
#include "integral/integralfeatures.h"


pthread_mutex_t ProcessFrameData::visualisation_mutex = PTHREAD_MUTEX_INITIALIZER;

ProcessFrameData::ProcessFrameData(cv::Mat mat_, int frameNum) :
mat(mat_),
frameData(frameNum)
{
    filterFar();
    markEdges();
    
    computeIntegral();
    
    /*resize();
    createBlobsAndBorders();
    tracking();
    shareFrameData();*/
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
    static const int maxKinectDepth(Params::getMaxKinectDepth());
    uint16_t* p_mat = (uint16_t*)(mat.data);
    uint16_t* p_matFilt = (uint16_t*)(matFilt.data);
    uint16_t* p_mat1 = (uint16_t*)(mat.data) + mat.total() - 1;
    uint16_t* p_matFilt1 = (uint16_t*)(matFilt.data) + matFilt.total() - 1;
    for(int j = 0; j < mat.rows; ++j){
        for(int i = 0; i < mat.cols; ++i){
            if(*(p_mat +i))
                break;
            *(p_matFilt + i) = maxKinectDepth;//0;
        }
        p_mat += mat.cols;
        p_matFilt += mat.cols;
        for(int i = 0; i < mat.cols; ++i){
            if(*(p_mat1 - i))
                break;
            *(p_matFilt1 - i) = maxKinectDepth;//0;
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
            *p_matFilt = maxKinectDepth;//0;
        }
        p_mat = (uint16_t*)(mat.data) + mat.total() - 1 - i;
        p_matFilt = (uint16_t*)(matFilt.data) + mat.total() - 1 - i;
        for(int j = 0; j < mat.rows; ++j, p_mat -= mat.cols, p_matFilt -= mat.cols){
            if(*p_mat)
                break;
            *p_matFilt = maxKinectDepth;//0;
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
    std::list<int> inds;
    cv::Mat matDst = Convex3d::extractConvexities(matResized, Params::getConvex3dFilterSize(), Params::getConvex3dFilterDepth(), Params::getConvex3dCoreHalfSize(), Params::getConvex3dCountFalsePercent(), false, inds);
    uint16_t* p_mat = (uint16_t*)(matDst.data);
    for(int i = 0; i < matDst.total(); ++i, ++p_mat){
        if(*p_mat > frameData.averagedBodyPoint.z)
            *p_mat = 0;
    }
    BlobsFabrique<BlobPrim> blobsFabrique1(1, matDst, Params::getBlobClustXYThresh(), Params::getBlobClustDepthThresh(), Params::getBlobClustMinSize());
    cv::resize(matFilt, matResized1, cv::Size(Params::getMatrixWidth() >> Params::GET_BLOB_EXT_RESIZE_POW(), Params::getMatrixHeight()>>Params::GET_BLOB_EXT_RESIZE_POW()));
    blobsFabrique1.constructBlobsExt(matResized1, blobsExt);
}

void ProcessFrameData::tracking(){
    //tracking hands
     Track::analyzeFrame(blobsExt);
}

void ProcessFrameData::shareFrameData(){
    frameData.averagedBodyPoint.x <<= Params::getBlobResizePow();
    frameData.averagedBodyPoint.y <<= Params::getBlobResizePow();
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
        //Visualization::mat2img(mat, img);
        Visualization::mat2img(matFilt, img);
        Visualization::integralFeatures2img(integralFeatures, img);
        //Visualization::vecs2img(matVec, img);
        const auto& point = frameData.averagedBodyPoint;
        cv::circle(img, cv::Point(point.x , point.y), 5,  cv::Scalar(255, 0, 255), -1);
        for(auto& blob : blobsExt)
            blob.Enlarge(img.cols);
        Visualization::blobs2img( blobsExt, img, false);
        //Visualization::tracks2img(Track::getTracksConst(), img);
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

void ProcessFrameData::computeIntegral(){
    size_t resizePow = 0;
    IntegralImage integralImage(matFilt, resizePow);
    cv::Mat integrMat = integralImage.getMatIntegral();
    
    static const int cellSize(32);
    static const int step (cellSize >> 2);
    static const cv::Vec2i edge(64, 64 * 1.75);
    IntegralGrid integralGrid(integralImage, cellSize, step, edge);
    
    const auto& s  = integralGrid.getSize();
    std::vector<int> thresh {-200 * cellSize * cellSize, -200 * cellSize * cellSize};
    std::vector<std::vector<cv::Vec2i>> geometries {{{-cellSize/step, 0}, {cellSize/step,0}}, {{0, -cellSize/step}, {0,cellSize/step}}};
    for(const auto& geometry : geometries){
        integralFeatures.push_back(std::vector<cv::Rect>());
        std::vector<std::vector<float>> vecResponse = integralGrid.getVecResponses(geometry);
        for(int i = 0; i < vecResponse.size(); ++i){
            const std::vector<float>& resp = vecResponse[i];
            bool isFeature(true);
            for(int i = 0; i < resp.size(); ++i){
                if(resp[i] == FLT_MAX || resp[i] > thresh[i]) {
                    isFeature = false;
                    break;
                }
            }
            if(isFeature){
                int x = i % s.width;
                int y = (i - x) / s.width;
                integralFeatures.back().emplace_back((x << resizePow) * step + (cellSize >>1), (y << resizePow) * step + (cellSize >>1), cellSize, cellSize);
            }
        }
    }

}

void ProcessFrameData::gradientMat(){
    std::vector<std::pair<int,int>> neighbours {
        {-1, 0}, {-1, 1}, {0, 1}, {1, 1},
        {1, 0}, {1, -1},  {0, -1},  {-1, -1}
    };
    cv::resize(matFilt, matResized, cv::Size(Params::getMatrixWidth()>> 3, Params::getMatrixHeight()>>3));
    matVec = cv::Mat_<cv::Vec2f>(matResized.size(), cv::Vec2f(0.f, 0.f));
    uint16_t* p_mat = (uint16_t*)(matResized.data);
    cv::Vec2f* p_matVec = (cv::Vec2f*)(matVec.data);
    int ind(0);
    for( ; p_mat < (uint16_t*)(matResized.data) + matResized.total(); ++p_mat, ++p_matVec, ++ind){
        //auto val = *p_mat;
        auto x = ind % matResized.cols;
        auto y = (ind - x) / matResized.cols;
        std::vector<cv::Vec2f> vecs(neighbours.size() >> 1, cv::Vec2f(0.f, 0.f));
        for(int i = 0; i < (neighbours.size() >> 1); ++i){
            std::vector<uint16_t> valNeighb(2, 0);
            for(int j = 0; j < 2; ++j){
                auto& neighb = neighbours[i + j * (neighbours.size() >> 1)];
                int xNeighb = x + neighb.first;
                int yNeighb = y + neighb.second;
                if(xNeighb < 0 || xNeighb >= matResized.cols || yNeighb < 0 || yNeighb >= matResized.rows)
                    continue;
                int indNeighb = yNeighb * matResized.cols + xNeighb;
                valNeighb[j] =  *((uint16_t*)(matResized.data) + indNeighb);
            }
            if(!valNeighb[0] || !valNeighb[1])
                continue;
            auto dval = valNeighb[1] - valNeighb[0];
            vecs[i] = i == 0 ? cv::Vec2f(dval, 0.f) :
            i == 1 ? cv::Vec2f(dval, dval) :
            i == 2 ? cv::Vec2f(0.f, dval) :
            cv::Vec2f(-dval, dval);
        }
        cv::Vec2f res(0.f, 0.f);
        for(auto& vec : vecs)
            res += vec;
        *p_matVec = res;
    }
}


