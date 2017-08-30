
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
#include "../graphs/shortestpath.hpp"

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
    bool IsAdjacent(const BlobExt& blob, int depthThresh = INT_MAX) const;
    void Enlarge(int width);
private:
    void CheckBlobFingers();
    void CreateGraph();
    
private:
    const cv::Mat mat;
    cv::Mat matBlob;
    std::map<int, int> cellsMap;
    Graph graph;
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
    {-1, 0}, {-1, 1}, {0, 1}, {1, 1},
    {1, 0}, {1, -1},  {0, -1},  {-1, -1}
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
    if(lCells_1.size() < maxCount){
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
                lCells.emplace_back(xNeighb, yNeighb, indNeighb, valNeighb);
            }
            if(isEnd)
                break;
        }
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
    Convex3d::extractConvexities(mat, filterSize, filterDepth, coreHalfSize, countFalsePercent, true, inds);
    for(auto i : inds){
        blobsFingers.emplace_back(matBlob, i, 40, 1);
    }
    if(blobsFingers.empty())
        return;
    BlobsClust<BlobExt<TContainer,T>> blobsClust(blobsFingers, 15, 40, 1, true);
    blobsFingers = std::move(blobsClust.getBlobsClust());
    CheckBlobFingers();
}

template<template<typename> class  TContainer, typename T>
void BlobExt<TContainer, T>::CheckBlobFingers(){
    //CreateGraph();
    //ShortestPath shortestPath(graph, 0);
    /*auto it = blobsFingers.begin();
    while(it != blobsFingers.end()){
        auto it1 = it;
        ++it1;
        ShortestPath shortestPath(graph, 0);
        while(it1 != blobsFingers.end()){
            ++it1;
        }
        ++it;
    }*/
    blobsFingers.sort([](const BlobExt<TContainer,T>& bl1, const BlobExt<TContainer,T>& bl2){return bl1.getCellsConst().MinValCell()->x < bl2.getCellsConst().MinValCell()->x;});
}

template<template<typename> class  TContainer, typename T>
bool BlobExt<TContainer, T>::IsAdjacent(const BlobExt& blob, int depthThresh) const{
    float x1 = this->cells.MinValCell()->x;
    float y1 = this->cells.MinValCell()->y;
    float x2 = blob.cells.MinValCell()->x;
    float y2 = blob.cells.MinValCell()->y;
    if(x1 == x2 && y1 == y2)
        return true;
    float dx = x2 - x1;
    float dy = y2 - y1;
    float stepX = std::abs(dx) > std::abs(dy) ? dx / std::abs(dx) : dx / std::abs(dy);
    float stepY = std::abs(dx) < std::abs(dy) ? dy / std::abs(dy) : dy / std::abs(dx);
    int src = std::abs(dx) >  std::abs(dy) ? x1 : y1;
    int dst = std::abs(dx) >  std::abs(dy) ? x2 : y2;
    int step = src < dst ? 1 : -1;
    int val = *((uint16_t*)(mat.data) + this->cells.MinValCell()->ind);
    for(int i = src; (i-src)*(i-dst) <= 0; i+= step, x1 += stepX, y1 += stepY){
        int ind = static_cast<int>(y1) * mat.cols + static_cast<int>(x1);
        if(ind >= mat.total())
            continue;
        int newVal = *((uint16_t*)(mat.data) + ind);
        if(newVal == 0 || std::abs(newVal - val) > depthThresh)
            return false;
        val = newVal;
    }
    return true;
}

template<template<typename> class  TContainer, typename T>
void BlobExt<TContainer, T>::CreateGraph(){
    int i = 0;
    for(auto& cell : cells.AllConst()){
        cellsMap[cell.ind] = i++;
    }
    int w = matBlob.cols;
    int h = matBlob.rows;
    uint16_t* p_matBlob = (uint16_t*)(matBlob.data);
    graph.verts_count = cells.Size();
    i = 0;
    for(auto& cell : cells.AllConst()){
        int x_ = cell.x;
        int y_ = cell.y;
        for(int i = 0; i < (neighbours.size() >> 1); ++i){
            int xNeighb = x_ + neighbours[i].first;
            int yNeighb = y_ + neighbours[i].second;
            if(xNeighb < 0 || xNeighb >= w || yNeighb < 0 || yNeighb >= h)
                continue;
            int indNeighb = yNeighb * w + xNeighb;
            int valNeighb = *(p_matBlob + indNeighb);
            if(valNeighb == 0)
                continue;
            int iNeighb  = cellsMap[indNeighb];
            graph.edges.emplace_back(i, iNeighb);
            graph.weights.push_back(static_cast<int>(Cell::Distance(cell, xNeighb, yNeighb, indNeighb )));
        }
        ++i;
    }
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
