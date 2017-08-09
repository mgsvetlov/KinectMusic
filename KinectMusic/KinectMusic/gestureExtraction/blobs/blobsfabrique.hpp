//
//  blobsfabrique.hpp
//  KinectMusic
//
//  Created by Mikhail Svetlov on 16/07/17.
//  Copyright © 2017 mgsvetlov. All rights reserved.
//

#ifndef blobsfabrique_hpp
#define blobsfabrique_hpp

#include <limits>
#include "../params.h"
#include "blobext.hpp"
#include "blobsclust.hpp"
#include "../../log/logs.h"

template<typename T>
class BlobsFabrique {
public:
    BlobsFabrique(int mode, cv::Mat mat, const std::list<int>& inds = std::list<int>());
    std::list<T>& getBlobs();
    cv::Point3i getAveragedBodyPoint();
    void checkConnectivity(cv::Mat mat, const cv::Point3i& averagedBodyPoint);
    template<typename T1> std::list<T1>& constructBlobsExt(cv::Mat origMat, std::list<T1>& blobsExt, const cv::Point3i& averagedBodyPoint);
private:
    void blobsFabrique0();
    void blobsFabrique1();
    int minCellIndex();
    void computeAveragedBodyPoint();
private:
    cv::Mat mat;
    int mode = 0;
    std::list<T> blobs;
    cv::Point3i averagedBodyPoint = cv::Point3i(-1);
    
};

template<typename T>
BlobsFabrique<T>::BlobsFabrique(int mode, cv::Mat m, const std::list<int>& inds) :
mat(m.clone()),
mode(mode)
{
    switch(mode) {
        case 0:
            blobsFabrique0(); //extract evrything up to body
            break;
        case 1 :
            blobsFabrique1(); //hands and head matrix segmentation
            break;
        default:
            break;
    }
}

template<typename T>
void BlobsFabrique<T>::blobsFabrique0(){
    int largeBlobMaxVal(NO_DATA_VALUE);
    while(true){
        int ind = minCellIndex();
        if(ind == -1)
            break;
        
        int minVal = *((uint16_t*)(mat.data) + ind);
        if(minVal >= Params::getMaxKinectValue())
            break;
        
        T nearestBlob(mat, ind);
        
        if(nearestBlob.getCellsConst().Size() < Params::getBlobMinSize())
            continue;
        
        if(largeBlobMaxVal != NO_DATA_VALUE && nearestBlob.getCellsConst().MinValCell() && nearestBlob.getCellsConst().MinValCell()->val > largeBlobMaxVal){
            break;
        }
        
        if(nearestBlob.getCellsConst().Size() > Params::getBlobMinSizeLast() && nearestBlob.getCellsConst().MaxValCell()){
            if(nearestBlob.getCellsConst().MaxValCell()->val < Params::getMaxKinectDepth()){
                largeBlobMaxVal = nearestBlob.getCellsConst().MaxValCell()->val;
                blobs.push_front(std::move(nearestBlob));
            }
            else {
                break;
            }
        }
        else {
            blobs.push_back(std::move(nearestBlob));
        }
        
    }
    if(!blobs.empty())
        computeAveragedBodyPoint();
}

template<typename T>
void BlobsFabrique<T>::blobsFabrique1(){
    while(true){
        int ind = minCellIndex();
        if(ind == -1)
            break;
        
        int minVal = *((uint16_t*)(mat.data) + ind);
        if(minVal == Params::getMaxKinectValue())
            return;
        blobs.emplace_back(mat, ind);
        continue;
    }
    BlobsClust<T> blobsClust(blobs, Params::getBlobClustXYThresh(), Params::getBlobClustDepthThresh());
    blobs = std::move(blobsClust.getBlobsClust());
}

template<typename T>
std::list<T>& BlobsFabrique<T>::getBlobs(){
    return blobs;
}

template<typename T>
cv::Point3i BlobsFabrique<T>::getAveragedBodyPoint(){
    return averagedBodyPoint;
}

template<typename T>
void BlobsFabrique<T>::checkConnectivity(cv::Mat mat, const cv::Point3i& averagedBodyPoint){
    int averagedBodyPointInd = averagedBodyPoint.y * mat.cols + averagedBodyPoint.x;
    T blobBody (mat.clone(), averagedBodyPointInd);
    cv::Mat matBodyBlob = blobBody.blob2mat();
    blobBody.computeBorderCells(matBodyBlob);
    const auto& bodyBorderCont = blobBody.getBorderCellsConst().AllConst();
    auto it = blobs.begin();
    while(it != blobs.end()){
        if(it->getCellsConst().Size() == 0){
            it = blobs.erase(it);
            continue;
        }
        int ind = it->getCellsConst().MinValCell()->ind;
        T blob (mat.clone(), ind);
        cv::Mat matBlob = blob.blob2mat();
        blob.computeBorderCells(matBlob);
        const auto& blobBorderCont = blob.getBorderCellsConst().AllConst();
        int xyDist = INT_MAX;
        for(const auto& cell : blobBorderCont){
            for(const auto& cellBody : bodyBorderCont){
                int dx = cell.x - cellBody.x;
                int dy = cell.y - cellBody.y;
                int dist = dx*dx + dy*dy;
                if(dist < xyDist)
                    xyDist = dist;
            }
        }
         int depthDist = blob.getCellsConst().MinValCell()->val - blobBody.getCellsConst().MinValCell()->val;
        
        if((xyDist > Params::getBlobConnectivityXYThresh1() && depthDist > Params::getBlobConnectivityDepthThresh())
           || xyDist > Params::getBlobConnectivityXYThresh2()
           ){
            it = blobs.erase(it);
            continue;
        }
            
       
        ++it;
    }
}

template<typename T>
template<typename T1> std::list<T1>& BlobsFabrique<T>::constructBlobsExt(cv::Mat origMat, std::list<T1>& blobsExt, const cv::Point3i& averagedBodyPoint){
    std::vector<int> inds;
    for(auto& blob  : blobs) {
        int ind = blob.indOriginNearest(origMat);
        if(ind != NO_DATA_VALUE)
            inds.push_back(ind);
    }
    const uint16_t* const p_mat = (uint16_t*)(origMat.data);
    std::sort(inds.begin(), inds.end(),
              [p_mat](int ind1, int ind2)
              {return *(p_mat + ind1) < *(p_mat + ind2);});
    int blobInd(0);
    cv::Mat origMatClone = origMat.clone();
    for(auto& ind : inds) {
        blobsExt.emplace_back(origMat, origMatClone, ind, blobInd++);
        auto& blobExt = blobsExt.back();
        if( blobExt.cells.Size() == 0
           ||blobExt.cells.MaxValCell()->val < Params::getBlobExtMaxDepthThresh()
           || blobExt.CreateBorder() == 0
           || (blobExt.borderPtr->bodyAdjacentCount != 0
               && cv::norm(blobExt.AveragePoint()-blobExt.borderPtr->adjacentAveragePoint) < Params::getBlobExtDistToAdjacentBorderThresh() )
           ) {
            blobsExt.pop_back();
            continue;
        }
    }
    return blobsExt;
}


template<typename T>
int BlobsFabrique<T>::minCellIndex(){
    double minVal = Params::getMaxKinectValue();
    int ind (-1);
    uint16_t* p = (uint16_t*)(mat.data);
    for(int i = 0; i < mat.total(); ++i, ++p){
        uint16_t val = *p;
        if(val && val < minVal){
            ind = i;
            minVal = val;
        }
    }
    return ind;
}

template<typename T>
void BlobsFabrique<T>::computeAveragedBodyPoint(){
    auto it = blobs.begin();
    T* largestBlob = &(*it++);
    for(; it != blobs.end(); ++it){
        if(it->getCellsConst().Size() > largestBlob->getCellsConst().Size())
            largestBlob = &(*it);
    }
    
    averagedBodyPoint = largestBlob->getCells().AveragedMinPoint(largestBlob->getCellsConst().Size());
}

#endif /* blobsfabrique_hpp */
