//
//  nearestblob.hpp
//  KinectMusic
//
//  Created by Mikhail Svetlov on 01/10/16.
//  Copyright © 2016 mgsvetlov. All rights reserved.
//

#ifndef blob_hpp
#define blob_hpp

#include <limits>
#include "cells.hpp"
#include <iostream>
#include <queue>
#include <algorithm>    // std::sort
#include <vector>       // std::vector
#include <sstream>
#include <iterator>
#include "../../log/logs.h"
#include "../analyze.h"
#include  "../pcl/pclplane.hpp"
#include  "../pcl/pcldownsample.hpp"
#ifdef USE_CELL_NORMAL
#include "../pcl/pclnormals.hpp"
#endif //USE_CELL_NORMAL
#include "../pcl/pclsegmentation.hpp"

template<typename T> class Blob {
public:
    Blob();
    
//private:
    Blob(cv::Mat mat, int ind);
    Blob(cv::Mat mat, int ind, int blobInd, bool connectivity, float distThresh, int sizeThresh = NO_DATA_VALUE);
    
public:
    Cells<T>& getCells() {return cells;}
    const Cells<T>& getCellsConst() const {return cells;}
    const Cells<T>& getBorder1Const() const {return border1;}
    const Cells<T>& getBorder2Const() const {return border2;}
    
    static cv::Mat blobs2mat(const std::list<Blob>& lBlobs, const cv::Size& size);
    
    const cv::Size& getMatSize() const {return this->matSize;}
    void setMatSize(cv::Size size) {this->matSize = size;}
    
    int indOriginNearest(cv::Mat originalMat) const;
    bool analyzeHand(cv::Mat originalMat);
    
private:
    cv::Mat blob2mat();
    void computeAngle();
    
private:
    Cells<T> cells;
    Cells<T> border1;
    Cells<T> border2;

    int angle;
    cv::Size matSize;
    
    friend class Hand;
    friend class Visualization;
};
/////////////////////////////////////////////////////////////////////////////////

template<typename T> Blob<T>::Blob()
{}

template<typename T> Blob<T>::Blob(cv::Mat mat, int ind) :
matSize(mat.size())
{
    cells.All().reserve(mat.total());
    int w = mat.cols;
    int h = mat.rows;
    uint16_t* p_mat = (uint16_t*)(mat.data);
    int x = ind % w;
    int y = (ind -x) / w;
    cells.AddCell(x, y, ind, *(p_mat + ind));
    *(p_mat + ind) = MAX_KINECT_VALUE;
    auto it = cells.All().begin();
    for( ;it != cells.All().end(); ++it){
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
                uint16_t valNeighb =  *(p_mat + indNeighb);
                if(valNeighb >= MAX_KINECT_VALUE || valNeighb == 0)
                    continue;
                if(abs(valNeighb-val) < MAX_NEIGHB_DIFF_COARSE){
                    cells.AddCell(xNeighb, yNeighb, indNeighb, valNeighb);
                    *(p_mat + indNeighb) = MAX_KINECT_VALUE;
                }
            }
        }
    }
}

//blob extended
template<typename T> Blob<T>::Blob(cv::Mat mat, int ind, int blobInd, bool connectivity, float distThresh, int sizeThresh){
    cells.All().reserve(sizeThresh);
    const uint16_t maskValue = 65535 - blobInd;
    int w =  mat.cols;
    int h = mat.rows;
    uint16_t* p_mat = (uint16_t*)(mat.data);
    int x = ind % w;
    int y = (ind - x)/ w;
    cells.AddCell(x, y, ind, *(p_mat + ind));
    *(p_mat + ind) = maskValue;
    bool isEndModeGlobal (false);
    auto it = cells.All().begin();
    for(;it != cells.All().end();++it){
        bool isEndMode (false);
        if(isEndModeGlobal || (sizeThresh != NO_DATA_VALUE && cells.Size() >= sizeThresh)
           || (connectivity && it->dist > distThresh )) {
            isEndMode = true;
        }
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
                uint16_t valNeighb =  *(p_mat + indNeighb);
                if(valNeighb >= MAX_KINECT_DEPTH  || valNeighb == 0 || abs(valNeighb-val) >= MAX_NEIGHB_DIFF_COARSE){
                    if(valNeighb > maskValue){
                        cells.Clear();
                        border1.Clear();
                        border2.Clear();
                        return;
                    }
                    if(valNeighb != maskValue){
                        *(p_mat + indNeighb) = maskValue;
                        border1.AddCell(xNeighb, yNeighb,indNeighb, valNeighb, *it);
                        border1.All().back().parent = &(*it);
                    }
                    continue;
                }
                *(p_mat + indNeighb) = maskValue;
                if(isEndMode){
                    border2.AddCell(xNeighb, yNeighb,indNeighb, valNeighb, *it);
                    border2.All().back().parent = &(*it);
                    continue;
                }
                if(!isEndMode && !isEndModeGlobal){
                    if(connectivity){
                        cells.AddCell(xNeighb, yNeighb,indNeighb, valNeighb, *it);
                        it->child = &cells.All().back();
                        cells.All().back().parent = &(*it);
                    }
                    else {
                        cells.AddCell(xNeighb, yNeighb, indNeighb, valNeighb);
                    }
                    if(cells.Size() == sizeThresh)
                        isEndModeGlobal = true;
                }
            }
        }
    }
}

template<typename T> int Blob<T>::indOriginNearest(cv::Mat originalMat) const{
    int ind = cells.MinValCell()->ind;
    int x = ind % this->matSize.width;
    int y = (ind-x) /this->matSize.width;
    x <<= BLOBS_RESIZE_POW;
    y <<= BLOBS_RESIZE_POW;
    
    static const int halfSize(1 << BLOBS_RESIZE_POW);
    
    int minX(x - halfSize), minY(y - halfSize), maxX(x + halfSize), maxY(y + halfSize);
    if(minX < 0)
        minX = 0;
    if(minY < 0)
        minY = 0;
    if(maxX >= originalMat.cols)
        maxX = originalMat.cols - 1;
    if(maxY >= originalMat.rows)
        maxY = originalMat.rows - 1;
    
    int indNearest (NO_DATA_VALUE);
    int valNearest (MAX_KINECT_VALUE);
    
    for(int y = minY; y <= maxY; ++y){
        uint16_t* p = (uint16_t*)(originalMat.data) + originalMat.cols * y + minX;
        for(int x = minX; x <= maxX; ++x, ++p){
            int val = *p;
            if((val > 0) &&   val < valNearest){
                indNearest = static_cast<int>(p - (uint16_t*)(originalMat.data));
                valNearest = val;
            }
        }
    }
    
    return indNearest;
    
}

template<typename T> bool Blob<T>::analyzeHand(cv::Mat originalMat){
    
    //PclNormals::estimateNormals(*this);
    //computeAngle();
    return true;
}

template<typename T> void Blob<T>::computeAngle(){
    /*if( cells.Size() < 3){
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
    this->angle = angle * 100.f;*/
}

template<typename T> cv::Mat Blob<T>::blob2mat(){
    cv::Mat mat = cv::Mat_<uint16_t>::zeros(this->matSize);
    for(auto& cell : cells.All()) {
        int ind = cell.ind;
        uint16_t* p_mat = (uint16_t*)(mat.data) + ind;
        *p_mat = cell.val;
    }
    return mat;
}

template<typename T> cv::Mat Blob<T>::blobs2mat(const std::list<Blob>& lBlobs, const cv::Size& size) {
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

using BlobPrim = Blob<Cell>;
#endif /* blob_hpp */
