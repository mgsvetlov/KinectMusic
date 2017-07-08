//
//  nearestblob.cpp
//  KinectMusic
//
//  Created by Mikhail Svetlov on 01/10/16.
//  Copyright © 2016 mgsvetlov. All rights reserved.
//
#include <iostream>
#include <queue>
#include <algorithm>    // std::sort
#include <vector>       // std::vector
#include <vector>
#include <sstream>
#include <iterator>
#include "blob.h"
#include "../../log/logs.h"
#include "../analyze.h"
#include  "../pcl/pcl.hpp"


Blob::Blob() :
lCells(std::list<Cell>()),
p_maxValCell(nullptr),
p_minValCell(nullptr),
centralCell(NO_DATA_VALUE, NO_DATA_VALUE)
{}

Blob::Blob(cv::Mat mat16, int x, int y) :
lCells(std::list<Cell>()),
p_maxValCell(nullptr),
p_minValCell(nullptr),
centralCell(NO_DATA_VALUE, NO_DATA_VALUE),
matSize(mat16.size())
{
    int w = mat16.cols;
    int h = mat16.rows;
    uint16_t* p_mat16 = (uint16_t*)(mat16.data);
    std::queue<int> q;
    int indStart = y*w + x;
    addCell(indStart, *(p_mat16 + indStart));
    p_minValCell = &lCells.front();
    q.push(indStart);
    while(!q.empty()){
        const int ind = q.front();
        uint16_t val =  *(p_mat16 + ind);
        *(p_mat16 + ind) = MAX_KINECT_VALUE;
        q.pop();
        int x_ = ind % w;
        int y_ = (ind - x_)/ w;
        
        for(int yNeighb = y_ - 1; yNeighb <= y_ + 1; yNeighb++){
            if(yNeighb < 0 || yNeighb >= h)
                continue;
            for(int xNeighb = x_-1; xNeighb <= x_ + 1; xNeighb++){
                if(xNeighb < 0 || xNeighb >= w)
                    continue;
                int indNeighb = yNeighb * w + xNeighb;
                uint16_t valNeighb =  *(p_mat16 + indNeighb);
                if(valNeighb >= MAX_KINECT_VALUE || valNeighb == 0)
                    continue;
                if(abs(valNeighb-val) < MAX_NEIGHB_DIFF_COARSE){
                    addCell(indNeighb, *(p_mat16 + indNeighb));
                    q.push(indNeighb);
                }
            }
        }
    }
}

void Blob::addCell(int ind, int val){
    lCells.push_back(Cell(ind, val));
    if(!p_maxValCell || p_maxValCell->val < val)
        p_maxValCell = &lCells.back();
}

int Blob::findBlobs(cv::Mat mat16, std::list<Blob>& lBlobs, int mode ){
    
    int body_depth = -1;
    
    cv::Mat mat16_clone = mat16.clone();
    int largeBlobMaxVal(-1);
    while(true){
        double minVal = MAX_KINECT_VALUE;
        int minIdx[mat16.dims];
        uint16_t* p_mat16_clone = (uint16_t*)(mat16_clone.data);
        for(int y = 0; y < mat16_clone.rows; y++)
        for(int x = 0 ; x < mat16_clone.cols; x++){
            uint16_t val = *p_mat16_clone++;
            if(val && val < minVal){
                minVal = val;
                minIdx[1] = x;
                minIdx[0] = y;
            }
        }
        
        if(mode == 1){ //hands head segmentation
            if(minVal == MAX_KINECT_VALUE)
                break;
            Blob nearestBlob(mat16_clone, minIdx[1], minIdx[0]);
            if(nearestBlob.getLCells().size() > 10)
                lBlobs.push_back(nearestBlob);
            continue;
        }
        
        if(minVal >= MAX_KINECT_VALUE)
            break;

        Blob nearestBlob(mat16_clone, minIdx[1], minIdx[0]);
        
        if(nearestBlob.getLCells().size() < BLOB_MIN_SIZE)
            continue;
        
        if(largeBlobMaxVal != -1 && nearestBlob.p_minValCell && nearestBlob.p_minValCell->val > largeBlobMaxVal){
            break;
        }
        
        if(nearestBlob.getLCells().size() > BLOB_MIN_SIZE_LAST && nearestBlob.p_maxValCell){
            largeBlobMaxVal = nearestBlob.p_maxValCell->val;
            lBlobs.push_front(nearestBlob);
        }
        else {
            lBlobs.push_back(nearestBlob);
        }
        
    }
    
    //compute bode_depth
    if(mode == 0 && !lBlobs.empty()){
        auto it = lBlobs.begin();
        Blob& largestBlob = *it++;
        for(; it != lBlobs.end(); ++it){
            if(it->lCells.size() > largestBlob.lCells.size())
                largestBlob = *it;
        }
        body_depth = largestBlob.computeAverageValue();
    }
    
    return body_depth;
}

bool Blob::computeCentralNearCell(double med){
    if(lCells.empty())
        return false;
    
    std::vector<Cell> vCells(lCells.begin(), lCells.end());
    std::sort(vCells.begin(), vCells.end(),
         [](const Cell& c1, const Cell& c2) -> bool
         {
             return c1.val < c2.val;
         }
         );
    
    int indLim = vCells.size() * med;
    
    double  ySum(0), xSum(0), valSum(0);
    int count(0);
    for( auto& cell: vCells){
        int ind = cell.ind;
        int x = ind % this->matSize.width;
        int y = (ind-x) /this->matSize.width;
        xSum += x;
        ySum += y;
        valSum += cell.val;
        count ++;
        if(count == indLim)
            break;
    }
    int x = xSum / indLim;
    int y = ySum /indLim;
    int val = valSum / indLim;
    int ind = this->matSize.width * y + x;
    this->centralCell.ind = ind;
    this->centralCell.val = val;
    return true;
}

bool Blob::computeCentralCell(){
    if(lCells.empty())
        return false;
    int ind(0), val(MAX_KINECT_DEPTH);
    for( auto& cell: lCells){
        if(cell.val < val){
            ind = cell.ind;
            val = cell.val;
        }
    }
    this->centralCell.ind = ind;
    this->centralCell.val = val;
    /*double  ySum(0), xSum(0), valSum(0);

    for( auto& cell: lCells){
        int ind = cell.ind;
        int x = ind % this->matSize.width;
        int y = (ind-x) /this->matSize.width;
        xSum += x;
        ySum += y;
        valSum += cell.val;
        
    }
    int x = xSum / lCells.size();
    int y = ySum /lCells.size();
    int val = valSum / lCells.size();
    int ind = this->matSize.width * y + x;
    this->centralCell.ind = ind;
    this->centralCell.val = val;*/
    return true;
}

int Blob::computeAverageValue(){
    int sum (0);
    for( auto& cell: lCells){
        sum += cell.val;
    }
    return sum / lCells.size();
}

bool Blob::isBlobNear(const Blob& blob, const int xyThresh, const int depthThresh)
{
    if(this->centralCell.ind == -1 || blob.centralCell.ind == -1)
        return false;
    if(abs(this->centralCell.val - blob.centralCell.val) > depthThresh)
        return false;
    int ind = this->centralCell.ind;
    int x = ind % this->matSize.width;
    int y = (ind-x) /this->matSize.width;
    int indBlob = blob.centralCell.ind;
    int xBlob = indBlob % blob.matSize.width;
    int yBlob = (indBlob-xBlob) /blob.matSize.width;
    int dx = x - xBlob;
    int dy = y - yBlob;
    if(dx * dx + dy * dy > xyThresh * xyThresh)
        return false;
    return true;
}

void Blob::mergeBlob(const Blob& blob){
    this->lCells.insert(this->lCells.end(), blob.lCells.begin(), blob.lCells.end());
    
    this->computeCentralCell();
}

bool Blob::blobsClustering(std::list<Blob>& lBlobs, std::list<Blob>& lBlobsClustered, int xyThresh, int depthThresh) {
    if(lBlobs.empty())
        return false;
    
    for(auto& blob : lBlobs) {
        blob.computeCentralCell();
        bool isMerged(false);
        for(auto& blobClust : lBlobsClustered){
            if(blobClust.isBlobNear(blob, xyThresh, depthThresh)){
                blobClust.mergeBlob(blob);
                isMerged = true;
                break;
            }
        }
        if(!isMerged)
            lBlobsClustered.push_back(blob);
    }
    
    bool merge (true);
    while(merge) {
        merge = false;
        auto it = lBlobsClustered.begin();
        while(it != lBlobsClustered.end()){
            auto it1 = it;
            it1++;
            while(it1 != lBlobsClustered.end()){
                if(it->isBlobNear(*it1, xyThresh, depthThresh )) {
                    it->mergeBlob(*it1);
                    it1 = lBlobsClustered.erase(it1);
                    merge = true;
                    continue;
                }
                it1++;
            }
            it++;
        }
    }
    
    auto it = lBlobsClustered.begin();
    while(it != lBlobsClustered.end()){
        if(it->lCells.size() < 5){
            it = lBlobsClustered.erase(it);
            continue;
        }
        //it->computeCentralNearCell(0.25);
        it->computeCentralCell();
        it++;
    }
    
    return true;
}

void Blob::sort(std::list<Blob>& lBlobs) {
    if(lBlobs.size() < 2){
        //lBlobs.clear();
        return;
    }
    std::vector<Blob> vBlobs (lBlobs.begin(), lBlobs.end());
    std::sort(vBlobs.begin(), vBlobs.end(), [](const Blob& b1, const Blob& b2){ return b1.centralCell.val < b2.centralCell.val; });
    lBlobs = std::list<Blob>(vBlobs.begin(), vBlobs.begin() + 2);
}

void Blob::filterNearBody(std::list<Blob>& lBlobs, int bodyDepth, int minDistToBody){
    std::list<Blob> lBlobsResult;
    auto it = lBlobs.begin();
    while(it != lBlobs.end()){
        if(it->centralCell.val < bodyDepth - minDistToBody){
            lBlobsResult.push_back(std::move(*it));
        }
        ++it;
    }
    lBlobs = std::move(lBlobsResult);
}

void Blob::originalData(cv::Mat originalMat){
    int ind = centralCell.ind;
    int x = ind % this->matSize.width;
    int y = (ind-x) /this->matSize.width;
    x <<= BLOBS_RESIZE_POW;
    y <<= BLOBS_RESIZE_POW;
    static constexpr int halfSize(60);
    static constexpr int depthThresh(150);
    static constexpr int depthThresh2(depthThresh * depthThresh * 0.8);
    int minX(x - halfSize), minY(y - halfSize), maxX(x + halfSize), maxY(y + halfSize);
    if(minX < 0)
        minX = 0;
    if(minY < 0)
        minY = 0;
    if(maxX >= originalMat.cols)
        maxX = originalMat.cols - 1;
    if(maxY >= originalMat.rows)
        maxY = originalMat.rows - 1;
    
    uint16_t* p_nearest = (uint16_t*)(originalMat.data) + originalMat.cols * minY + minX;
    for(int y = minY; y <= maxY; ++y){
        uint16_t* p = (uint16_t*)(originalMat.data) + originalMat.cols * y + minX;
        for(int x = minX; x <= maxX; ++x, ++p){
            if((*p>0) &&   ((*p_nearest) == 0 || *p < *p_nearest))
                p_nearest = p;
        }
    }
    
    centralCell = Cell(static_cast<int>(p_nearest - (uint16_t*)(originalMat.data)), *p_nearest);
    matSize = originalMat.size();
    p_maxValCell = p_minValCell = nullptr;
    
    lCells.clear();
    
    int closest_z = centralCell.val;
    int closest_x = centralCell.ind % this->matSize.width;
    int closest_y = (centralCell.ind-closest_x) /this->matSize.width;
    for(int y = minY; y <= maxY; ++y){
        uint16_t* p = (uint16_t*)(originalMat.data) + originalMat.cols * y + minX;
        for(int x = minX; x <= maxX; ++x, ++p){
            if((*p>0) &&   *p - closest_z < depthThresh){
                int dx = x - closest_x;
                int dy = y - closest_y;
                int dz = *p - closest_z;
                if(dx*dx + dy*dy + dz*dz< depthThresh2)
                lCells.emplace_back(Cell(static_cast<int>(p - (uint16_t*)(originalMat.data)), *p));
            }
        }
    }
    
}

void Blob::computeAngle(){
    if( lCells.size() < 3){
        this->angle = 0.0f;
        return;
    }
    float x(0.0), y(0.0), z(0.0), w(0.0);
    Pcl::fitPlane(*this, x, y, z, w);
    float angle = std::abs(z);
    float norm = sqrt(y*y + z*z);
    if( norm != 0.0f)
        angle /= norm;
    if(z * y < 0)
        angle *= -1;
    this->angle = angle * 100.f;
}

cv::Mat Blob::blobs2mat(const std::list<Blob>& lBlobs, const cv::Size& size) {
    cv::Mat mat = cv::Mat_<uint16_t>::zeros(size);
    
    for(auto& blob : lBlobs) {
        auto& lCells = blob.getLCellsConst();
        for(auto& cell : lCells) {
            int ind = cell.ind;
            uint16_t* p_mat = (uint16_t*)(mat.data) + ind;
            *p_mat = cell.val;
        }
    }
    
    return mat;
}


