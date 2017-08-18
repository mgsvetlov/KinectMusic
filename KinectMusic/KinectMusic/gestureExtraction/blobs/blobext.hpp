
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
#include "../convex3d/convex3d.h"


template<template<typename> class  TContainer, typename T> class BlobExt : public Blob<TContainer,  T> {
    using Blob<TContainer,T>::cells;
public:
    BlobExt() = delete;
    BlobExt(const cv::Mat mat, cv::Mat matClone, int ind, int blobInd);
    
    size_t CreateBorder();
    std::unique_ptr<Border<TContainer, T>>& getBorderPtr() { return borderPtr; }
    const cv::Point3i& AveragePoint();
private:
    void computeAngles();

private:
    const cv::Mat mat;
    std::unique_ptr<Border<TContainer, T>> borderPtr;
    cv::Point3i averagePoint = cv::Point3i(-1);
    std::list<int> convexInds;
    
    static std::vector<std::pair<int,int>> neighbours;
   
    template<typename U>
    friend class BlobsFabrique;
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
    double coeff = static_cast<double>(Params::getBlobExtDepthCoeff()) / firstVal;
    int maxCount = coeff * coeff * Params::getBlobExtMaxSize();
    std::list<Cell> lCells, lCells_1;
    int minVal = firstVal;
    lCells.emplace_back(x, y, ind, firstVal);
    lCells_1.emplace_back(x, y, ind, firstVal);
    *(p_mat + ind) = maskValue;
    while(!lCells.empty()){
        auto cell = lCells.front();
        const uint16_t val =  cell.val;
        int x_ = cell.x;
        int y_ = cell.y;
        lCells.pop_front();
        bool isEnd(false);
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
            lCells_1.emplace_back(xNeighb, yNeighb, indNeighb, valNeighb);
            if(valNeighb < minVal)
                minVal = valNeighb;
            if(lCells_1.size() == maxCount){
                isEnd = true;
                break;
            }
            else if(valNeighb < val)
                lCells.emplace_front(xNeighb, yNeighb, indNeighb, valNeighb);
            else
                lCells.emplace_back(xNeighb, yNeighb, indNeighb, valNeighb);
        }
        if(isEnd)
            break;
    }
    cells.All().reserve(lCells_1.size());
    static const int maxDepthRange = Params::getBlobExtMaxDepthRange();
    for(auto& cell : lCells_1){
        if(cell.val - minVal < maxDepthRange)
            cells.AddCell(cell);
    }
}


template<template<typename> class  TContainer,  typename T>
size_t BlobExt<TContainer, T>::CreateBorder() {
    borderPtr = std::unique_ptr<Border<TContainer, T>>(new Border<TContainer, T>(mat, cells));
    return borderPtr->getContour().size();
}


template<template<typename> class TContainer, typename T>
const cv::Point3i& BlobExt<TContainer,T>::AveragePoint() {
    if(averagePoint != cv::Point3i(-1))
        return averagePoint;
    averagePoint = cells.AveragedMinPoint(Params::getBlobExtFrontCellsCount());
    return averagePoint;
}

template<template<typename> class  TContainer, typename T>
void BlobExt<TContainer, T>::computeAngles(){
    if(borderPtr){
        borderPtr->computeAngles();
    }
    cv::Mat matBlob = cv::Mat_<uint16_t>::zeros(this->mat.size());
    static int val = Params::getMaxKinectDepth();
    matBlob  = cv::Scalar::all(val);
    for(auto& cell : cells.All()) {
        int ind = cell.ind;
        uint16_t* p_mat = (uint16_t*)(matBlob.data) + ind;
        *p_mat = cell.val;
    }
    int filterSize(mat.cols * 0.02);
    int filterDepth(mat.cols * 0.1);
    int coreHalfSize(1);
    cv::Mat matDst = Convex3d::extractConvexities(matBlob, filterSize, filterDepth, coreHalfSize);
    
    uint16_t* p_mat = (uint16_t*)(matDst.data);
    for(int i = 0; i < matDst.total(); ++i, ++p_mat){
        if(*p_mat)
            convexInds.push_back(i);
    }
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
