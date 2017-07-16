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
#include  "../pcl/pclplane.hpp"
#include  "../pcl/pcldownsample.hpp"
#ifdef USE_CELL_NORMAL
#include "../pcl/pclnormals.hpp"
#endif //USE_CELL_NORMAL
#include "../pcl/pclsegmentation.hpp"


Blob::Blob()
{}

Blob::Blob(cv::Mat mat16, int ind) :
matSize(mat16.size())
{
    int w = mat16.cols;
    int h = mat16.rows;
    uint16_t* p_mat16 = (uint16_t*)(mat16.data);
    int x = ind % w;
    int y = (ind -x) / w;
    cells.AddCell(x, y, ind, *(p_mat16 + ind));
    *(p_mat16 + ind) = MAX_KINECT_VALUE;
    auto it = cells.All().begin();
    while(it != cells.All().end()){
        uint16_t val =  it->val;
        int x_ = it->x;
        int y_ = it->y;
        
        for(int yNeighb = y_ - 1; yNeighb <= y_ + 1; yNeighb++){
            if(yNeighb < 0 || yNeighb >= h)
                continue;
            for(int xNeighb = x_-1; xNeighb <= x_ + 1; xNeighb++){
                if((yNeighb == y_ && xNeighb == x_) || xNeighb < 0 || xNeighb >= w)
                    continue;
                int indNeighb = yNeighb * w + xNeighb;
                uint16_t valNeighb =  *(p_mat16 + indNeighb);
                if(valNeighb >= MAX_KINECT_VALUE || valNeighb == 0)
                    continue;
                if(abs(valNeighb-val) < MAX_NEIGHB_DIFF_COARSE){
                    cells.AddCell(xNeighb, yNeighb, indNeighb, valNeighb);
                    *(p_mat16 + indNeighb) = MAX_KINECT_VALUE;
                }
            }
        }
        ++it;
    }
}

bool Blob::filterLargeBlobs(cv::Mat originalMat){
    int ind = cells.MinValCell()->ind;
    int x = ind % this->matSize.width;
    int y = (ind-x) /this->matSize.width;
    x <<= BLOBS_RESIZE_POW;
    y <<= BLOBS_RESIZE_POW;
    
    static const int halfSize(originalMat.cols * 0.09375);
    
    int minX(x - halfSize), minY(y - halfSize), maxX(x + halfSize), maxY(y + halfSize);
    if(minX < 0)
        minX = 0;
    if(minY < 0)
        minY = 0;
    if(maxX >= originalMat.cols)
        maxX = originalMat.cols - 1;
    if(maxY >= originalMat.rows)
        maxY = originalMat.rows - 1;
    
    matSize = originalMat.size();
    int closest_z = cells.MinValCell()->val;
    cells.Clear();
    
    static constexpr int depthThresh(150);
    
    for(int y = minY; y <= maxY; ++y){
        uint16_t* p = (uint16_t*)(originalMat.data) + originalMat.cols * y + minX;
        for(int x = minX; x <= maxX; ++x, ++p){
            if((*p>0) &&   *p - closest_z < depthThresh){
                cells.AddCell(Cell(x, y, static_cast<int>(p - (uint16_t*)(originalMat.data)), *p));
            }
        }
    }
    
    if(cells.Size() > originalMat.cols * 6.25 || cells.Size() == 0)
        return false;

    return true;
}

void Blob::createCellsTree(cv::Mat mat, int ind, int val, bool connectivity, float distThresh){
    cv::Mat matMask = cv::Mat_<unsigned char>::zeros(mat.size());
    int w =  mat.cols;
    int h = mat.rows;
    uint16_t* p_mat = (uint16_t*)(mat.data);
     unsigned char* p_matMask  = (unsigned char*)(matMask.data);
    int x = ind % w;
    int y = (ind - x)/ w;
    cells.AddCell(Cell(x, y, ind, val));
    *(p_matMask + ind) = 255;
    auto it = cells.All().begin();
    while(it != cells.All().end()){
        const int dist = it->dist;
        if(dist <= distThresh ){
            const uint16_t val =  it->val;
            int x_ = it->x;
            int y_ = it->y;
            for(int yNeighb = y_ - 1; yNeighb <= y_ + 1; yNeighb++){
                if(yNeighb < 0 || yNeighb >= h)
                    continue;
                for(int xNeighb = x_-1; xNeighb <= x_ + 1; xNeighb++){
                    if((yNeighb == y_ && xNeighb == x_) || xNeighb < 0 || xNeighb >= w)
                        continue;
                    int indNeighb = yNeighb * w + xNeighb;
                    if(*(p_matMask + indNeighb) != 0) {
                        continue;
                    }
                    uint16_t valNeighb =  *(p_mat + indNeighb);
                    if(valNeighb >= MAX_KINECT_DEPTH  || valNeighb == 0)
                        continue;
                    if(abs(valNeighb-val) < MAX_NEIGHB_DIFF_COARSE){
                        cells.AddCell(xNeighb, yNeighb,indNeighb, valNeighb, *it);
                        *(p_matMask + indNeighb) = 255;
                        if(connectivity){
                            it->child = &cells.All().back();
                            cells.All().back().parent = &(*it);
                        }
                    }
                }
            }
        }
        ++it;
    }
}

void Blob::createSubBlobs(){
    static const int subBlobSize = matSize.width * 9 / 16;
    int count(0);
    for(auto& cell : cells.All()){
        if(count == 0){
            subBlobs.push_back(SubBlob());
        }
        cell.subBlob = &subBlobs.back();
        subBlobs.back().vpCells.push_back(&cell);
        ++count;
        if(count == subBlobSize)
            count = 0;
    }
}

void Blob::createBorders(){
    auto it = subBlobs.begin();
    for(int i = 0; i < subBlobs.size() - 1; ++i, ++it){
        borders.emplace_back(Border());
        auto& border = borders.back();
        border.level = i;
        auto& borderCells = border.borderCells;
        SubBlob* cuurentSubBlob = &(*it);
        for(const auto& cell : it->vpCells){
            if(cell->child && cell->child->subBlob != cuurentSubBlob)
                borderCells.push_back(cell);
        }
    }
}

bool Blob::analyzeHand(cv::Mat originalMat){
    //create tree from front
    static const float distThresh(500.0f);
    createCellsTree(originalMat, cells.MinValCell()->ind, cells.MinValCell()->val, true, distThresh);

    createSubBlobs();
    
    createBorders();
    
    //PclNormals::estimateNormals(*this);
    //computeAngle();
    return true;
}

void Blob::computeAngle(){
    if( cells.Size() < 3){
        this->angle = 0.0f;
        return;
    }
    float x(0.0), y(0.0), z(0.0), w(0.0);
    PclPlane::fitPlane(*this, x, y, z, w);
    float angle = std::abs(z);
    float norm = sqrt(y*y + z*z);
    if( norm != 0.0f)
        angle /= norm;
    if(z * y < 0)
        angle *= -1;
    this->angle = angle * 100.f;
}

cv::Mat Blob::blob2mat(){
    cv::Mat mat = cv::Mat_<uint16_t>::zeros(this->matSize);
    for(auto& cell : cells.All()) {
        int ind = cell.ind;
        uint16_t* p_mat = (uint16_t*)(mat.data) + ind;
        *p_mat = cell.val;
    }
    return mat;
}

cv::Mat Blob::blobs2mat(const std::list<Blob>& lBlobs, const cv::Size& size) {
    cv::Mat mat = cv::Mat_<uint16_t>::zeros(size);
    
    for(auto& blob : lBlobs) {
        for(auto& cell : blob.getCellsConst().AllConst()) {
            int ind = cell.ind;
            uint16_t* p_mat = (uint16_t*)(mat.data) + ind;
            *p_mat = cell.val;
        }
    }
    
    return mat;
}
