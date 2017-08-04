
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
#include "../params.h"


template<template<typename> class  TContainer, typename T> class BlobExt : public Blob<TContainer,  T> {
    using Blob<TContainer,T>::cells;
public:
    BlobExt() = delete;
    BlobExt(const cv::Mat mat, cv::Mat matClone, int ind, int blobInd);
    
    size_t CreateBorder();
    const std::vector<double>& getFeatures() const {return features;}
    void computeFeatures(const cv::Point3i& averagedBodyPoint);
    const cv::Point3i& AveragePoint();
private:
    void computeAngle();

private:
    const cv::Mat mat;
    std::unique_ptr<Border<TContainer, T>> borderPtr;
    cv::Point3i averagePoint = cv::Point3i(-1);
    std::vector<double> features;
    
    static std::vector<std::pair<int,int>> neighbours;
   
    friend class Visualization;
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
    
    const uint16_t maskValue = 65535 - blobInd;
    int w = matClone.cols;
    int h = matClone.rows;
    uint16_t* p_mat = (uint16_t*)(matClone.data);
    int x = ind % w;
    int y = (ind - x)/ w;
    uint16_t firstVal = *(p_mat + ind);
    double coeff = 1200. / firstVal;
    int maxCount = coeff * coeff * Params::getBlobExtMaxSize();
    cells.All().reserve(maxCount);
    cells.AddCell(x, y, ind, firstVal);
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
            if(valNeighb >= Params::getMaxKinectDepth()  || valNeighb == 0 || abs(valNeighb-val) >= Params::getMaxHeighbDiffCoarse()){
                if(valNeighb > maskValue ){
                    cells.Clear();
                    return;
                }
                continue;
            }
            *(p_mat + indNeighb) = maskValue;
            cells.AddCell(xNeighb, yNeighb, indNeighb, valNeighb);
            if(cells.Size() == maxCount)
                return;
        }
    }
}


template<template<typename> class  TContainer,  typename T>
size_t BlobExt<TContainer, T>::CreateBorder() {
    borderPtr = std::unique_ptr<Border<TContainer, T>>(new Border<TContainer, T>(mat, cells));
    return borderPtr->getContour().size();
}

template<template<typename> class TContainer, typename T>
void BlobExt<TContainer,T>::computeFeatures(const cv::Point3i& averagedBodyPoint) {
    averagePoint = AveragePoint();
    features.push_back(static_cast<double>(averagePoint.z)/averagedBodyPoint.z);
    int diff = averagePoint.y - averagedBodyPoint.y;
    features.push_back((Params::getMatrixWidth()  - 2.0 * static_cast<double>(std::abs(diff))) / Params::getMatrixWidth());
    features.push_back(static_cast<double>(borderPtr->bodyAdjacentCount)/borderPtr->nonBodyAdjacentCount);
    /*features.push_back(averagePoint.x);
    features.push_back(averagePoint.y);
    for(auto& cell : borderPtr->contourCompressed){
        features.push_back(cell.x - averagedBodyPoint.x);
        features.push_back(cell.y - averagedBodyPoint.y);
        features.push_back(cell.val - averagedBodyPoint.z);
        bool isAdj = cell.flags & FLAGS::ADJACENT_BODY;
        features.push_back(isAdj ? -1 : 1);
    }*/
}

template<template<typename> class TContainer, typename T>
const cv::Point3i& BlobExt<TContainer,T>::AveragePoint() {
    if(averagePoint != cv::Point3i(-1))
        return averagePoint;
    averagePoint = cells.AveragedMinPoint(Params::getBlobFrontCellsCount());
    return averagePoint;
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


using BlobFinal = BlobExt<Vector, Cell>;

#endif /* blobext_hpp */
