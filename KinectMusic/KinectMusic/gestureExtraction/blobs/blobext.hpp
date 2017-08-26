
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
#include "blobsfabrique.hpp"

template<template<typename> class  TContainer, typename T> class BlobExt : public Blob<TContainer,  T> {
    using Blob<TContainer,T>::cells;
public:
    BlobExt() = delete;
    BlobExt(const cv::Mat mat, int ind, size_t maxNeighbDiff = Params::getMaxNeighbDiffCoarse(), int maxCount = -1, int maxDepthRange = -1);
    void CreateBlobsFingers();
    void ComputeAngle();
    //size_t CreateBorder();
    //std::unique_ptr<Border<TContainer, T>>& getBorderPtr() { return borderPtr; }
    const cv::Point3i& AveragePoint();
    void Enlarge(int width);
private:

private:
    const cv::Mat mat;
    cv::Mat matBlob;
    //std::unique_ptr<Border<TContainer, T>> borderPtr;
    cv::Point3i averagePoint = cv::Point3i(-1);
    std::list<BlobExt<TContainer,T>> blobsFingers;
    std::unique_ptr<Angles3d> angles3dPtr;
    
    bool testFeature;
    
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
BlobExt<TContainer, T>::BlobExt(const cv::Mat mat, int ind, size_t maxNeighbDiff, int maxCount, int maxDepthRange) :
mat(mat)
{
    matBlob = cv::Mat_<uint16_t>::zeros(cv::Size(mat.cols, mat.rows));
    int w = mat.cols;
    int h = mat.rows;
    uint16_t* p_mat = (uint16_t*)(mat.data);
    uint16_t* p_matBlob = (uint16_t*)(matBlob.data);
    int x = ind % w;
    int y = (ind - x)/ w;
    uint16_t firstVal = *(p_mat + ind);
    std::list<Cell> lCells, lCells_1;
    int minVal = firstVal;
    lCells.emplace_back(x, y, ind, firstVal);
    lCells_1.emplace_back(x, y, ind, firstVal);
    *(p_matBlob + ind) = firstVal;
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
            if(*(p_matBlob + indNeighb))
                continue;
            uint16_t valNeighb =  *(p_mat + indNeighb);
            if(valNeighb >= Params::getMaxKinectDepth()  || valNeighb == 0 || abs(valNeighb-val) >= maxNeighbDiff){
                continue;
            }
            *(p_matBlob + indNeighb) = valNeighb;
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
    for(auto& cell : lCells_1){
        if(maxDepthRange <= 0 || cell.val - minVal < maxDepthRange)
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
void BlobExt<TContainer, T>::CreateBlobsFingers(){
    int filterSize = Params::GET_BLOB_EXT_CONVEX3D_FILTER_SIZE_FINE();
    int filterDepth = Params::GET_BLOB_EXT_CONVEX3D_FILTER_DEPTH_FINE();
    int coreHalfSize = Params::GET_BLOB_EXT_CONVEX3D_CORE_HALF_SIZE_FINE();
    int countFalsePercent = Params::GET_BLOB_EXT_CONVEX3D_COUNT_FALSE_PERCENT_FINE();
    std::list<int> inds;
    for(auto& cell : cells.All())
        inds.push_back(cell.ind);
    cv::Mat matDst = Convex3d::extractConvexities(mat, filterSize, filterDepth, coreHalfSize, countFalsePercent, true, inds);
    uint16_t* p_mat = (uint16_t*)(matDst.data);
    for(int i = 0; i < matDst.total(); ++i, ++p_mat){
        if(!*p_mat)
            continue;
        blobsFingers.emplace_back(mat, i, 40, 10);
        auto& blobFingers = blobsFingers.back();
        auto& cellsFing = blobFingers.getCellsConst();
        std::list<int> inds1;
        for(const auto& cell : cellsFing.AllConst()){
            *((uint16_t*)(matDst.data) + cell.ind) = 0;
            inds1.push_back(cell.ind);
        }
        
        Convex3d::extractConvexities1(mat, 1, filterDepth, 2, inds1);
        if(inds1.size() < (cellsFing.Size() >> 2)){
            blobsFingers.pop_back();
            continue;
        }
        
        for(auto& blob : blobsFingers){
            bool isNear(false);
            if (&blob == &blobFingers)
                break;
            auto& cellsFing1 = blob.getCells();
            for(const auto& cell : cellsFing.AllConst()){
                for(auto& cell1 : cellsFing1.AllConst()){
                    if(std::abs(cell.x - cell1.x) <= 1 && std::abs(cell.y - cell1.y) <= 1){
                        isNear = true;
                        break;
                    }
                }
                if(isNear)
                    break;
            }
            if(isNear){
                for(const auto& cell : cellsFing.AllConst())
                    cellsFing1.AddCell(cell);
                blobsFingers.pop_back();
                break;
            }
        }
    }
    blobsFingers.sort([](const BlobExt<TContainer,T>& bl1, const BlobExt<TContainer,T>& bl2){return bl1.getCellsConst().MinValCell()->x < bl2.getCellsConst().MinValCell()->x;});
}

template<template<typename> class  TContainer, typename T>
void BlobExt<TContainer, T>::ComputeAngle(){
    std::list<cv::Point3i> points;
    for(const auto& blob : blobsFingers){
        for(const auto& cell : blob.cells.AllConst()){
            points.emplace_back(cell.x, cell.y, cell.val);
        }
    }
    angles3dPtr = std::unique_ptr<Angles3d>(new Angles3d(points));
    const auto& anglesData = angles3dPtr->getDataConst();
    if(anglesData.empty())
        return;
    const auto& plane = std::get<0>(anglesData.front());
    const auto& point = std::get<1>(anglesData.front());
    int count1(0), count2(0);
    float x(plane.x), y(plane.y), z(plane.z), w(plane.w);
    for(const auto& cell : cells.AllConst()){
        if(abs(cell.x - point.x) > 50 || abs(cell.y - point.y) > 50|| abs(cell.val - point.z) > 30)
            continue;
        if(cell.x * x + cell.y * y + cell.val * z + w < 0)
            ++count1;
        else
            ++count2;
        /*int x = cell.x;
        int y = cell.y;
        int z = cell.val;
        if(
        if(abs(x - point.x) > 50 || abs(y - point.y) > 50|| abs(z - point.z) > 30)
            continue;
        float crossZ = -(plane.w + plane.x*x + plane.y*y) / static_cast<float>(plane.z);
        if(crossZ < z)
            ++count1;
        else
            ++count2;*/
    }
    testFeature = (count1  < count2) ? true : false;
}

template<template<typename> class  TContainer, typename T>
void BlobExt<TContainer, T>::Enlarge(int width){
    static const int resizePow = log2(width/this->mat.cols);
    if(resizePow == 0)
        return;
    for(auto& cell : cells.All()){
        cell.x <<= resizePow;
        cell.y <<= resizePow;
        cell.ind = cell.y * width + cell.x;
    }
    for(auto& blob : blobsFingers){
        for(auto& cell : blob.getCells().All()){
            cell.x <<= resizePow;
            cell.y <<= resizePow;
            cell.ind = cell.y * width + cell.x;
        }
    }
}

using BlobFinal = BlobExt<Vector, Cell>;

#endif /* blobext_hpp */
