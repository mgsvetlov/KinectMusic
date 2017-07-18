//
//  blobsfabrique.hpp
//  KinectMusic
//
//  Created by Mikhail Svetlov on 16/07/17.
//  Copyright © 2017 mgsvetlov. All rights reserved.
//

#ifndef blobsfabrique_hpp
#define blobsfabrique_hpp

#include "blob.hpp"
#include "blobsclust.hpp"
#include "../../log/logs.h"

template<typename T> class BlobsFabrique {
public:
    BlobsFabrique(int mode, cv::Mat mat, const std::list<int>& inds = std::list<int>());
    std::list<T>& getBlobs();
    template<typename T1> std::list<T1>& constructBlobsExt(cv::Mat origMat, std::list<T1>& blobsExt);
    int getBodyDepth();
private:
    void blobsFabrique0();
    void blobsFabrique1();
    int minCellIndex();
    void computeBodyDepth();
private:
    cv::Mat mat;
    int mode = 0;
    std::list<T> blobs;
    int bodyDepth = -1;
};

template<typename T> BlobsFabrique<T>::BlobsFabrique(int mode, cv::Mat m, const std::list<int>& inds) :
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

template<typename T> void BlobsFabrique<T>::blobsFabrique0(){
    int largeBlobMaxVal(-1);
    while(true){
        int ind = minCellIndex();
        if(ind == -1)
            break;
        
        int minVal = *((uint16_t*)(mat.data) + ind);
        if(minVal >= MAX_KINECT_VALUE)
            break;
        
        T nearestBlob(mat, ind);
        
        if(nearestBlob.getCellsConst().Size() < BLOB_MIN_SIZE)
            continue;
        
        if(largeBlobMaxVal != -1 && nearestBlob.getCellsConst().MinValCell() && nearestBlob.getCellsConst().MinValCell()->val > largeBlobMaxVal){
            break;
        }
        
        if(nearestBlob.getCellsConst().Size() > BLOB_MIN_SIZE_LAST && nearestBlob.getCellsConst().MaxValCell()){
            largeBlobMaxVal = nearestBlob.getCellsConst().MaxValCell()->val;
            blobs.push_front(std::move(nearestBlob));
        }
        else {
            blobs.push_back(std::move(nearestBlob));
        }
        
    }
    if(!blobs.empty())
        computeBodyDepth();
}

template<typename T> void BlobsFabrique<T>::blobsFabrique1(){
    while(true){
        int ind = minCellIndex();
        if(ind == -1)
            break;
        
        int minVal = *((uint16_t*)(mat.data) + ind);
        if(minVal == MAX_KINECT_VALUE)
            return;
        blobs.emplace_back(mat, ind);
        continue;
    }
    int xyThresh(mat.cols / 8), depthThresh(100);
    BlobsClust<T> blobsClust(blobs, xyThresh, depthThresh);
    blobs = std::move(blobsClust.getBlobsClust());
}

template<typename T> std::list<T>& BlobsFabrique<T>::getBlobs(){
    return blobs;
}

template<typename T> template<typename T1> std::list<T1>& BlobsFabrique<T>::constructBlobsExt(cv::Mat origMat, std::list<T1>& blobsExt){
    static const float distThresh(400.0f);
    static const int sizeThresh(4000);
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
    for(auto& ind : inds) {
        blobsExt.emplace_back(origMat, ind, blobInd++, true, distThresh, sizeThresh);
        if(blobsExt.back().getCellsConst().Size() <= 1
           || blobsExt.back().getBorder1Const().Size() == 0
           || blobsExt.back().getBorder1Const().Size() < blobsExt.back().getBorder2Const().Size())
            blobsExt.pop_back();
    }
    return blobsExt;
}


template<typename T> int BlobsFabrique<T>::getBodyDepth(){
    return bodyDepth;
}

template<typename T> int BlobsFabrique<T>::minCellIndex(){
    double minVal = MAX_KINECT_VALUE;
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

template<typename T> void BlobsFabrique<T>::computeBodyDepth(){
    
    auto it = blobs.begin();
    T* largestBlob = &(*it++);
    for(; it != blobs.end(); ++it){
        if(it->getCellsConst().Size() > largestBlob->getCellsConst().Size())
            largestBlob = &(*it);
    }
    
    bodyDepth = largestBlob->getCellsConst().AverageValue();
}

#endif /* blobsfabrique_hpp */
