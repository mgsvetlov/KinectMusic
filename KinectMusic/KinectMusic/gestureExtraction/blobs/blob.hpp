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
#include <algorithm>

#include "../params.h"
#include "cells/cells.hpp"

#include "../../log/logs.h"
#include  "../pcl/pcldownsample.hpp"
#ifdef USE_CELL_NORMAL
#include "../pcl/pclnormals.hpp"
#endif //USE_CELL_NORMAL
#include "../pcl/pclsegmentation.hpp"

template<template<typename> class TContainer, typename T>
class Blob {
public:
    Blob();
    Blob(cv::Mat mat, int ind, size_t maxNeighbDiff = Params::getMaxNeighbDiffCoarse());
    
    Cells<TContainer,T>& getCells() {return cells;}
    const Cells<TContainer,T>& getCellsConst() const {return cells;}
    const Cells<TContainer,T>& getBorderCellsConst() const {return borderCells;}
    const cv::Size& getMatSize() const {return this->matSize;}
    void setMatSize(cv::Size size) {this->matSize = size;}
    int indOriginNearest(cv::Mat originalMat) const;
    
    static cv::Mat blobs2mat(const std::list<Blob>& lBlobs, const cv::Size& size);
    cv::Mat blob2mat();
    void computeBorderCells(cv::Mat mat);
    
protected:
    Cells<TContainer,T> cells;
    Cells<TContainer,T> borderCells;
    cv::Size matSize;

private:
    static std::vector<std::pair<int,int>> neighbours;
    
    friend class Visualization;
};

/////////////////////////////////////////////////////////////////////////////////
template<template<typename> class  TContainer, typename T>
std::vector<std::pair<int,int>> Blob<TContainer, T>::neighbours {
    {-1,0}, {0, -1}, {1,0}, {0,1}
};

template<template<typename> class TContainer, typename T>
Blob<TContainer, T>::Blob()
{}

template<template<typename> class TContainer, typename T>
Blob<TContainer,T>::Blob(cv::Mat mat, int ind, size_t maxNeighbDiff) :
matSize(mat.size())
{
    cells.All().reserve(mat.total());
    int w = mat.cols;
    int h = mat.rows;
    uint16_t* p_mat = (uint16_t*)(mat.data);
    int x = ind % w;
    int y = (ind -x) / w;
    cells.AddCell(x, y, ind, *(p_mat + ind));
    *(p_mat + ind) = Params::getMaxKinectValue();
    auto it = cells.All().begin();
    for( ;it != cells.All().end(); ++it){
        const uint16_t val =  it->val;
        int x_ = it->x;
        int y_ = it->y;
        for(auto& neighb : neighbours){
            int xNeighb = x_ + neighb.first;
            int yNeighb = y_ + neighb.second;
            if(xNeighb < 0 || xNeighb >= w || yNeighb < 0 || yNeighb >= h)
                continue;
            int indNeighb = yNeighb * w + xNeighb;
            uint16_t valNeighb =  *(p_mat + indNeighb);
            if(valNeighb >= Params::getMaxKinectValue() || valNeighb == 0 || abs(valNeighb-val) >= maxNeighbDiff)
                continue;
            *(p_mat + indNeighb) = Params::getMaxKinectValue();
            cells.AddCell(xNeighb, yNeighb, indNeighb, valNeighb);
        }
    }
}

template<template<typename> class TContainer, typename T>
int Blob<TContainer,T>::indOriginNearest(cv::Mat originalMat) const{
    int ind = cells.MinValCell()->ind;
    int x = ind % this->matSize.width;
    int y = (ind-x) /this->matSize.width;
    static const int resizePow = log2(originalMat.cols/this->matSize.width);
    x <<= resizePow;
    y <<= resizePow;
    
    static const int halfSize(1 << resizePow);
    
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
    int valNearest (Params::getMaxKinectValue());
    
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

template<template<typename> class TContainer, typename T>
cv::Mat Blob<TContainer,T>::blob2mat(){
    cv::Mat mat = cv::Mat_<uint16_t>::zeros(this->matSize);
    for(auto& cell : cells.All()) {
        int ind = cell.ind;
        uint16_t* p_mat = (uint16_t*)(mat.data) + ind;
        *p_mat = cell.val;
    }
    return mat;
}

template<template<typename> class TContainer, typename T>
cv::Mat Blob<TContainer,T>::blobs2mat(const std::list<Blob<TContainer,T>>& lBlobs, const cv::Size& size) {
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

template<template<typename> class TContainer, typename T>
void Blob<TContainer,T>::computeBorderCells(cv::Mat mat){
    int w = mat.cols;
    int h = mat.rows;
    uint16_t* p_mat = (uint16_t*)(mat.data);
    auto it = cells.All().begin();
    for( ;it != cells.All().end(); ++it){
        int x_ = it->x;
        int y_ = it->y;
        for(auto& neighb : neighbours){
            int xNeighb = x_ + neighb.first;
            int yNeighb = y_ + neighb.second;
            if(xNeighb < 0 || xNeighb >= w || yNeighb < 0 || yNeighb >= h)
                continue;
            int indNeighb = yNeighb * w + xNeighb;
            uint16_t valNeighb =  *(p_mat + indNeighb);
            if(valNeighb == 0){
                borderCells.AddCell(*it);
                break;
            }
        }
    }
}

using BlobPrim = Blob<Vector, Cell>;


#endif /* blob_hpp */
