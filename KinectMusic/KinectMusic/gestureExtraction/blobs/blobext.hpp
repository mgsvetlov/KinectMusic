
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
#include "../angles3d/angles3d.h"


template<template<typename> class  TContainer, typename T> class BlobExt : public Blob<TContainer,  T> {
    using Blob<TContainer,T>::cells;
public:
    BlobExt() = delete;
    BlobExt(const cv::Mat mat, int ind, int blobInd);
    std::list<int> FindFeatureInds(const std::list<int>& inds, int filterSize, int filterDepth, int coreHalfSize, int countFalsePercent);
    void ComputeAngle();
    //size_t CreateBorder();
    //std::unique_ptr<Border<TContainer, T>>& getBorderPtr() { return borderPtr; }
    const cv::Point3i& AveragePoint();
    std::list<int>& getFeatureIndsCoarse(){return featureIndsCoarse;}
    std::list<int>& getFeatureIndsFine(){return featureIndsFine;}
private:

private:
    const cv::Mat mat;
    //std::unique_ptr<Border<TContainer, T>> borderPtr;
    cv::Point3i averagePoint = cv::Point3i(-1);
    std::list<int> featureIndsCoarse;
    std::list<int> featureIndsFine;
    std::unique_ptr<Angles3d> angles3dPtr;
    
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
BlobExt<TContainer, T>::BlobExt(const cv::Mat mat, int ind, int blobInd) :
mat(mat)
{
    cv::Mat matClone = mat.clone();
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


/*
template<template<typename> class  TContainer,  typename T>
size_t BlobExt<TContainer, T>::CreateBorder() {
    borderPtr = std::unique_ptr<Border<TContainer, T>>(new Border<TContainer, T>(mat, cells));
    return borderPtr->getContour().size();
}*/


template<template<typename> class TContainer, typename T>
const cv::Point3i& BlobExt<TContainer,T>::AveragePoint() {
    if(averagePoint != cv::Point3i(-1))
        return averagePoint;
    averagePoint = cells.AveragedMinPoint(Params::getBlobExtFrontCellsCount());
    return averagePoint;
}

template<template<typename> class  TContainer, typename T>
std::list<int> BlobExt<TContainer, T>::FindFeatureInds(const std::list<int>& inds, int filterSize, int filterDepth, int coreHalfSize, int countFalsePercent){
    std::list<int> featureInds;
    cv::Mat matDst = Convex3d::extractConvexities(mat, filterSize, filterDepth, coreHalfSize, countFalsePercent, true, inds);
    uint16_t* p_mat = (uint16_t*)(matDst.data);
    for(int i = 0; i < matDst.total(); ++i, ++p_mat){
        if(*p_mat)
            featureInds.push_back(i);
    }
    return featureInds;
}

template<template<typename> class  TContainer, typename T>
void BlobExt<TContainer, T>::ComputeAngle(){
    std::list<cv::Point3i> points;
    for(auto ind : featureIndsFine){
        int x = ind % mat.cols;
        int y = (ind - x ) / mat.cols;
        int z = *((uint16_t*)(mat.data) + ind);
        points.emplace_back(x, y, z);
    }
    angles3dPtr = std::unique_ptr<Angles3d>(new Angles3d(points));
 
}


using BlobFinal = BlobExt<Vector, Cell>;

#endif /* blobext_hpp */
