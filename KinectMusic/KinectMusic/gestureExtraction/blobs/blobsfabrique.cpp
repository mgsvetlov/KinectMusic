//
//  blobsfabrique.cpp
//  KinectMusic
//
//  Created by Mikhail Svetlov on 16/07/17.
//  Copyright © 2017 mgsvetlov. All rights reserved.
//

#include "blobsfabrique.hpp"

BlobsFabrique::BlobsFabrique(cv::Mat m, int mode) :
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
}

std::list<Blob>& BlobsFabrique::getBlobs(){
    return blobs;
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