
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
    BlobExt(const cv::Mat mat, cv::Mat matClone, int ind, int blobInd);
    
    size_t CreateBorder();
    
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
    {-1,0}, {0, -1}, {1,0}, {0,1},
    {-1, -1}, {-1, 1}, {1, -1}, {1, 1}
};

//blob extended
template<template<typename> class  TContainer, typename T>
BlobExt<TContainer, T>::BlobExt(const cv::Mat mat, cv::Mat matClone, int ind, int blobInd) :
mat(mat)
{
    cells.All().reserve(BLOB_EXT_MAX_SIZE);
    const uint16_t maskValue = 65535 - blobInd;
    int w = matClone.cols;
    int h = matClone.rows;
    uint16_t* p_mat = (uint16_t*)(matClone.data);
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
            if(valNeighb >= MAX_KINECT_DEPTH  || valNeighb == 0 || abs(valNeighb-val) >= ExtractFrameData::MAX_NEIGHB_DIFF_COARSE){
                if(valNeighb > maskValue){
                    cells.Clear();
                    return;
                }
                continue;
            }
            *(p_mat + indNeighb) = maskValue;
            cells.AddCell(xNeighb, yNeighb, indNeighb, valNeighb);
            if(cells.Size() == BLOB_EXT_MAX_SIZE)
                return;
        }
    }
}


template<template<typename> class  TContainer,  typename T>
size_t BlobExt<TContainer, T>::CreateBorder() {
    borderPtr = std::unique_ptr<Border<TContainer, T>>(new Border<TContainer, T>(mat, cells));
    return borderPtr->getContour().size();
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
