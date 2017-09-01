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
    BlobsFabrique(int mode, cv::Mat mat, int xyThresh = 0, int depthThresh = 0, int blobMinSize = -1, size_t maxNeighbDiff = Params::getMaxNeighbDiffCoarse());
    std::list<T>& getBlobs();
    cv::Point3i getAveragedBodyPoint();
    template<typename T1> std::list<T1>& constructBlobsExt(cv::Mat origMat, std::list<T1>& blobsExt);
private:
    void blobsFabrique0();
    void blobsFabrique1(int xyThresh, int depthThresh, int blobMinSize, size_t maxNeighbDiff);
    int minCellIndex();
    void computeAveragedBodyPoint();
private:
    cv::Mat mat;
    int mode = 0;
    std::list<T> blobs;
    cv::Point3i averagedBodyPoint = cv::Point3i(-1);
    
};

template<typename T>
BlobsFabrique<T>::BlobsFabrique(int mode, cv::Mat m, int xyThresh, int depthThresh, int blobMinSize, size_t maxNeighbDiff) :
mat(m.clone()),
mode(mode)
{
    switch(mode) {
        case 0:
            blobsFabrique0(); //extract evrything up to body
            break;
        case 1 :
            blobsFabrique1(xyThresh, depthThresh, blobMinSize, maxNeighbDiff); //hands and head matrix segmentation
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
void BlobsFabrique<T>::blobsFabrique1(int xyThresh, int depthThresh, int blobMinSize, size_t maxNeighbDiff){
    while(true){
        int ind = minCellIndex();
        if(ind == -1)
            break;
        
        int minVal = *((uint16_t*)(mat.data) + ind);
        if(minVal == Params::getMaxKinectValue())
            return;
        blobs.emplace_back(mat, ind, maxNeighbDiff);
        continue;
    }
    if(xyThresh > 0 && depthThresh > 0) {
        BlobsClust<T> blobsClust(blobs, xyThresh, depthThresh, blobMinSize);
        blobs = std::move(blobsClust.getBlobsClust());
    }
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
template<typename T1> std::list<T1>& BlobsFabrique<T>::constructBlobsExt(cv::Mat origMat, std::list<T1>& blobsExt){
    std::vector<int> inds;
    for(auto& blob  : blobs) {
        int ind = blob.indOriginNearest(origMat);
        if(ind != NO_DATA_VALUE)
            inds.push_back(ind);
    }
    for(auto& ind : inds) {
        uint16_t firstVal = *((uint16_t*)(origMat.data) + ind);
        double coeff = static_cast<double>(Params::getBlobExtDepthCoeff()) / firstVal;
        int maxCount = coeff * coeff * Params::getBlobExtMaxSize();
        blobsExt.emplace_back(origMat, ind, Params::getMaxNeighbDiffCoarse(), maxCount, Params::getBlobExtMaxDepthRange());
        auto& blobExt = blobsExt.back();
        if( blobExt.cells.Size() == 0
           ||blobExt.cells.MaxValCell()->val < Params::getBlobExtMaxDepthThresh()
           ) {
            blobsExt.pop_back();
            continue;
        }
        std::list<int> cellInds;
        for(const auto& cell : blobExt.cells.AllConst()) {
            cellInds.push_back(cell.ind);
        }
        cv::Mat matDst = Convex3d::extractConvexities(origMat,
            Params::GET_BLOB_EXT_CONVEX3D_FILTER_SIZE_COARSE(),
            Params::GET_BLOB_EXT_CONVEX3D_FILTER_DEPTH_COARSE(),
            Params::GET_BLOB_EXT_CONVEX3D_CORE_HALF_SIZE_COARSE(),
            Params::GET_BLOB_EXT_CONVEX3D_COUNT_FALSE_PERCENT_COARSE(), true, cellInds);
        if(cellInds.size() < Params::GET_BLOB_EXT_FEATURE_INDS_COARSE_MIN_SIZE()){
            blobsExt.pop_back();
            continue;
        }
        blobExt.CreateBlobsFingers();
        blobExt.ComputeAngle();
        uint16_t* p_origMat = (uint16_t*)(origMat.data);
        for(const auto& cell : blobExt.cells.AllConst()) {
            *(p_origMat + cell.ind) = 0;
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
