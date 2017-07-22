
//
//  blobext.h
//  KinectMusic
//
//  Created by Mikhail Svetlov on 18/07/17.
//  Copyright © 2017 mgsvetlov. All rights reserved.
//

#ifndef blobext_hpp
#define blobext_hpp

#include <memory>
#include "blob.hpp"
#include "borders/border.hpp"


template<template<typename> class  TContainer, typename T> class BlobExt : public Blob<TContainer,  T> {
    using Blob<TContainer,T>::cells;
public:
    BlobExt() = delete;
    BlobExt(cv::Mat mat, int ind, int blobInd, int sizeThresh);
    
    void CreateBorders();
    
    void SetFrameNum(int frameNum);
private:
    void computeAngle();

private:
    const cv::Mat mat;
    std::unique_ptr<Border<TContainer, T>> borderPtr;
    int angle;
    int frameNum;
    friend class Visualization;
    static std::vector<std::pair<int,int>> neighbours;
};

template<template<typename> class  TContainer, typename T>
std::vector<std::pair<int,int>> BlobExt<TContainer, T>::neighbours {
    {-1,0}, {0, -1}, {1,0}, {0,1}
};

//blob extended
template<template<typename> class  TContainer, typename T>
BlobExt<TContainer, T>::BlobExt(cv::Mat mat, int ind, int blobInd, int sizeThresh) :
mat(mat)
{
    cells.All().reserve(sizeThresh);
    const uint16_t maskValue = 65535 - blobInd;
    int w = mat.cols;
    int h = mat.rows;
    uint16_t* p_mat = (uint16_t*)(mat.data);
    int x = ind % w;
    int y = (ind - x)/ w;
    cells.AddCell(x, y, ind, *(p_mat + ind));
    *(p_mat + ind) = maskValue;
    auto it = cells.All().begin();
    for(;it != cells.All().end();++it){
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
            if(valNeighb >= MAX_KINECT_DEPTH  || valNeighb == 0 || abs(valNeighb-val) >= MAX_NEIGHB_DIFF_COARSE){
                if(valNeighb > maskValue){
                    cells.Clear();
                    return;
                }
                continue;
            }
            *(p_mat + indNeighb) = maskValue;
            cells.AddCell(xNeighb, yNeighb, indNeighb, valNeighb);
            if(cells.Size() == sizeThresh)
                return;
        }
    }
}


template<template<typename> class  TContainer,  typename T>
void BlobExt<TContainer, T>::CreateBorders() {
    borderPtr = std::unique_ptr<Border<TContainer, T>>(new Border<TContainer, T>(mat, cells));
}

template<template<typename> class  TContainer, typename T>
void BlobExt<TContainer, T>::computeAngle(){
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

template<template<typename> class  TContainer, typename T>
void BlobExt<TContainer, T>::SetFrameNum(int frameNum_){
    frameNum = frameNum_;
}

using BlobFinal = BlobExt<Vector, Cell>;

#endif /* blobext_hpp */
