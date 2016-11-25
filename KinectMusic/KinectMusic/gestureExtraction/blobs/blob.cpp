//
//  nearestblob.cpp
//  KinectMusic
//
//  Created by Mikhail Svetlov on 01/10/16.
//  Copyright © 2016 mgsvetlov. All rights reserved.
//
#include <iostream>
#include "blob.h"
#include <queue>
#include <algorithm>    // std::sort
#include <vector>       // std::vector

Blob::Blob() :
lCells(std::list<Cell>()),
p_maxValCell(nullptr),
p_minValCell(nullptr),
centralCell(-1, -1),
isHandOpened(false),
nearestBorderCell(-1, MAX_KINECT_VALUE)
{}

Blob::Blob(cv::Mat mat16, int x, int y) :
lCells(std::list<Cell>()),
p_maxValCell(nullptr),
p_minValCell(nullptr),
centralCell(-1, -1),
matSize(mat16.size()),
isHandOpened(false),
nearestBorderCell(-1, MAX_KINECT_VALUE)
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

void Blob::findBlobs(cv::Mat mat16, std::list<Blob>& lBlobs, int mode ){

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
        
        if(largeBlobMaxVal != -1 && nearestBlob.getP_minValCell()->val > largeBlobMaxVal){
            break;
        }
        
        if(nearestBlob.getLCells().size() > BLOB_MIN_SIZE_LAST){
            largeBlobMaxVal = nearestBlob.getP_maxValCell()->val;
            lBlobs.push_front(nearestBlob);
            //std::cout << "largest size  " << nearestBlob.getLCells().size() << std::endl;
        }
        else {
            lBlobs.push_back(nearestBlob);
        }
        
    }
    
}

bool Blob::computeCentralCell(){
    if(lCells.empty())
        return false;
    double  ySum(0), xSum(0), valSum(0);

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
    this->centralCell.val = val;
    return true;
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

double Blob::dist2blob(const Blob& blob){
    int ind = this->centralCell.ind;
    int x = ind % this->matSize.width;
    int y = (ind-x) /this->matSize.width;
    int indBlob = blob.centralCell.ind;
    int xBlob = indBlob % blob.matSize.width;
    int yBlob = (indBlob-xBlob) /blob.matSize.width;
    int dx = x - xBlob;
    int dy = y - yBlob;
    return sqrt(static_cast<double>(dx * dx + dy * dy));
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
        it->computeCentralCell();
        it->nearestCell = Cell(-1, MAX_KINECT_VALUE);
        for(auto& cell: it->lCells){
            if(cell.val < it->nearestCell.val)
                it->nearestCell = cell;
        }
        it++;
    }
    
    return true;
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

void Blob::filterFar(){
    std::vector<Cell> vCells(lCells.begin(), lCells.end());
    std::sort(vCells.begin(), vCells.end(), [] (const Cell& c1, const Cell& c2) -> bool {
        return c1.val < c2.val;
    }
              );
    size_t size = vCells.size() * 0.5;
    if(size > 100)
        size = 100;
    lCells = std::list<Cell>(vCells.begin(), vCells.begin() + size);
    
}

void Blob::detectHandOpened(){
    int xMin(matSize.width), xMax(0), yMin(matSize.height), yMax(0);
    double valSum (0);
    for(auto& cell : lCells){
        valSum += cell.val;
        int ind = cell.ind;
        int x = ind % this->matSize.width;
        int y = (ind-x) /this->matSize.width;
        if(x < xMin)
            xMin = x;
        if(x > xMax)
            xMax = x;
        if(y < yMin)
            yMin = y;
        if(y > yMax)
            yMax = y;
    }
    double valAvg = valSum /lCells.size();
    double devSum(0.);
    for(auto& cell : lCells){
        devSum += std::abs(cell.val - valAvg);
    }
    double devAvg = devSum / lCells.size();

    double dx = xMax - xMin;
    double dy = yMax - yMin;
    if(devAvg * dx * dy*  pow(static_cast<double>(yMax)/matSize.height, 0.3) > 250)
        isHandOpened = true;

}

std::list<Blob> Blob::segmentation() const{
    std::list<Blob> lBlobs;
    int w = matSize.width;
    int h = matSize.height;
    cv::Mat matBlob = cv::Mat_<uint16_t>::zeros(matSize);
    cv::Mat matBlobOrig = cv::Mat_<uint16_t>::zeros(matSize);
    cv::Mat matBlobBorder = cv::Mat_<unsigned char>::zeros(matSize);
    uint16_t* const p_matBlob = (uint16_t*)(matBlob.data);
    uint16_t* const p_matBlobOrig = (uint16_t*)(matBlobOrig.data);
    unsigned char* const p_matBlobBorder = (unsigned char*)(matBlobBorder.data);
    for(auto& cell : lCells) {
        *(p_matBlobOrig + cell.ind) = *(p_matBlob + cell.ind) = cell.val;
        *(p_matBlobBorder + cell.ind) = cell.border;
    }
    
    uint16_t* p_matBlob1 = (uint16_t*)(matBlob.data);
    for(int i = 0; i< matBlob.total(); i++, p_matBlob1++) {
        if(*p_matBlob1 == 0)
            continue;
        Blob blob;
        blob.addCell(i, *p_matBlob1);
        *p_matBlob1 = 0;
        std::queue<int> q;
        q.push(i);
        while(!q.empty()){
            const int ind = q.front();
            q.pop();
            int x = ind % w;
            int y = (ind - x)/ w;
            for(int yNeighb = y - 1; yNeighb <= y + 1; yNeighb++){
                if(yNeighb < 0 || yNeighb >= h)
                    continue;
                for(int xNeighb = x - 1; xNeighb <= x + 1; xNeighb++){
                    if(xNeighb < 0 || xNeighb >= w)
                        continue;
                    int indNeighb = yNeighb * w + xNeighb;
                    uint16_t valNeighb =  *(p_matBlob + indNeighb);
                    if(valNeighb == 0)
                        continue;
                    uint16_t val = *(p_matBlobOrig + ind);
                    if(std::abs(val - valNeighb) > 10)
                        continue;
                    blob.addCell(indNeighb, valNeighb);
                    blob.lCells.back().border = *(p_matBlobBorder + indNeighb);
                    q.push(indNeighb);
                    *(p_matBlob + indNeighb) = 0;
                }
            }
            
        }
        if(!blob.lCells.empty()){
            blob.setMatSize(matSize);
            lBlobs.push_back(blob);
        }
    }
    
    return lBlobs;
    
}

void Blob::fiterBorder(){
    Blob borderBlob;
    borderBlob.setMatSize(matSize);
    for(auto& cell : lCells){
        if(cell.border){
            borderBlob.lCells.push_back(cell);
        }
    }
    std::list<Blob> lBlobsSegm = borderBlob.segmentation();
    if(lBlobsSegm.size() < 2)
        return;
    Blob& largestBlob = lBlobsSegm.front();
    for(auto& blob : lBlobsSegm){
        if(blob.lCells.size() > largestBlob.lCells.size()) {
            largestBlob = blob;
        }
    }
    for(auto& cell : lCells){
        if(cell.border){
            cell.border = 0;
        }
    }
    for(auto& cell1 : largestBlob.lCells){
        for(auto& cell : lCells){
            if(cell.ind == cell1.ind){
                cell.border = cell1.border;
                break;
            }
        }
    }
}

bool Blob::findNearestBorderCell(){
    for(auto& cell : lCells){
        if(cell.border != 0 && cell.val < nearestBorderCell.val){
            nearestBorderCell = cell;
        }
    }
    return nearestBorderCell.ind != -1;
}
