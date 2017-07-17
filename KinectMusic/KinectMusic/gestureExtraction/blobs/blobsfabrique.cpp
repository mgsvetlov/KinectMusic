//
//  blobsfabrique.cpp
//  KinectMusic
//
//  Created by Mikhail Svetlov on 16/07/17.
//  Copyright © 2017 mgsvetlov. All rights reserved.
//

#include "blobsfabrique.hpp"
#include "blobsclust.hpp"

BlobsFabrique::BlobsFabrique(int mode, cv::Mat m, const std::list<int>& inds) :
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

void BlobsFabrique::blobsFabrique0(){
    int largeBlobMaxVal(-1);
    while(true){
        int ind = minCellIndex();
        if(ind == -1)
            break;
        
        int minVal = *((uint16_t*)(mat.data) + ind);
        if(minVal >= MAX_KINECT_VALUE)
            break;
        
        Blob nearestBlob(mat, ind);
        
        if(nearestBlob.cells.Size() < BLOB_MIN_SIZE)
            continue;
        
        if(largeBlobMaxVal != -1 && nearestBlob.cells.MinValCell() && nearestBlob.cells.MinValCell()->val > largeBlobMaxVal){
            break;
        }
        
        if(nearestBlob.cells.Size() > BLOB_MIN_SIZE_LAST && nearestBlob.cells.MaxValCell()){
            largeBlobMaxVal = nearestBlob.cells.MaxValCell()->val;
            blobs.push_front(std::move(nearestBlob));
        }
        else {
            blobs.push_back(std::move(nearestBlob));
        }
        
    }
    if(!blobs.empty())
        computeBodyDepth();
}

void BlobsFabrique::blobsFabrique1(){
    while(true){
        int ind = minCellIndex();
        if(ind == -1)
            break;
        
        int minVal = *((uint16_t*)(mat.data) + ind);
        if(minVal == MAX_KINECT_VALUE)
            return;
        blobs.emplace_back(Blob (mat, ind));
        continue;
    }
    int xyThresh(mat.cols / 8), depthThresh(100);
    BlobsClust blobsClust(blobs, xyThresh, depthThresh);
    blobs = std::move(blobsClust.getBlobsClust());
}

void BlobsFabrique::constructBlobsExt(cv::Mat origMat){
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
        blobsExt.emplace_back(Blob(origMat, ind, blobInd++, true, distThresh, sizeThresh));
        if(blobsExt.back().cells.Size() <= 1
           || blobsExt.back().border1.Size() == 0
           || blobsExt.back().border1.Size() < blobsExt.back().border2.Size())
            blobsExt.pop_back();
    }
}

std::list<Blob>& BlobsFabrique::getBlobs(){
    return blobs;
}

std::list<Blob>& BlobsFabrique::getBlobsExt(){
    return blobsExt;
}


int BlobsFabrique::getBodyDepth(){
    return bodyDepth;
}

int BlobsFabrique::minCellIndex(){
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

void BlobsFabrique::computeBodyDepth(){
    
    auto it = blobs.begin();
    Blob* largestBlob = &(*it++);
    for(; it != blobs.end(); ++it){
        if(it->getCellsConst().Size() > largestBlob->getCellsConst().Size())
            largestBlob = &(*it);
    }

    bodyDepth = largestBlob->getCellsConst().AverageValue();
}