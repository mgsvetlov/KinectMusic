
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
    BlobExt(const cv::Mat mat, int ind, size_t maxNeighbDiff = Params::getMaxNeighbDiffCoarse(), int maxCount = -1);
    void CreateBlobsFingers();
    void ComputeAngle();
    //size_t CreateBorder();
    //std::unique_ptr<Border<TContainer, T>>& getBorderPtr() { return borderPtr; }
    const cv::Point3i& AveragePoint();
    bool IsAdjacent(const BlobExt& blob, int depthThresh = INT_MAX) const;
    void Enlarge(int width);
    void SetMat (cv::Mat m) {mat = m;}
private:
    inline bool GoRoundNeighbours(std::list<Cell>& lCells, Cell& cell, size_t maxNeighbDiff, int maxCount);
    inline bool IsAdjacentCells(int x1, int y1, uint16_t val1, int x2, int y2, int depthThresh = INT_MAX) const;
    void Extend(size_t maxNeighbDiff, int adjacThresh, float adjacPercent);
    void CheckBlobFingers();
    void CreateGraph();
    
private:
    cv::Mat mat;
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
BlobExt<TContainer, T>::BlobExt(const cv::Mat mat, int ind, size_t maxNeighbDiff, int maxCount) :
mat(mat)
{
    matBlob = cv::Mat_<uint16_t>::zeros(cv::Size(mat.cols, mat.rows));
    
    //create first cell
    int w = mat.cols;
    int x = ind % w;
    int y = (ind - x)/ w;
    uint16_t val = *((uint16_t*)(mat.data) + ind);
    std::list<T> lCells (1, Cell(x, y, ind, val));
    *((uint16_t*)(matBlob.data) + ind) = val;
    cells.AddCell(lCells.back());
    //go round matrix
    if(maxCount == -1 || lCells.size() < maxCount){
        auto it = lCells.begin();
        while(it != lCells.end()){
            auto& cell = *it;
            if(!GoRoundNeighbours(lCells, cell, maxNeighbDiff, maxCount))
                break;
            ++it;
        }
    }
    auto it = lCells.begin();
    ++it;
    for(;it != lCells.end();++it){
        cells.AddCell(*it);
    }
}

template<template<typename> class  TContainer, typename T>
void BlobExt<TContainer, T>::Extend(size_t maxNeighbDiff, int adjacThresh, float adjacPercent) {
    std::list<T> lCellsBase;
    for(const auto& cell : this->cells.AllConst()){
        lCellsBase.push_back(cell);
    }

    for(const auto& cellBase : lCellsBase){
        std::list<T> lCells(1, cellBase);
        auto it = lCells.begin();
        while(it != lCells.end()){
            auto& cell = *it;
            if(!GoRoundNeighbours(lCells, cell, maxNeighbDiff, -1))
                break;
            auto it1 = it;
            ++it1;
            while(it1 != lCells.end()){
                auto& cell1 = *it1;
                size_t testCount (lCellsBase.size());
                size_t thresh = testCount * adjacPercent;
                size_t count(0);
                size_t i (0);
                for(const auto& cellBase : lCellsBase){
                    count += IsAdjacentCells(cell1.x, cell1.y, cell1.val, cellBase.x, cellBase.y, adjacThresh);
                    if(count > thresh)
                        break;
                    ++i;
                    if(i - count > testCount - thresh)
                        break;
                }
                if(count <= thresh){
                    it1 = lCells.erase(it1);
                    continue;
                }
                ++it1;
            }
            ++it;
        }
        for(const auto& cell : lCells)
            cells.AddCell(cell);
    }
}

template<template<typename> class  TContainer,  typename T>
inline bool BlobExt<TContainer, T>::GoRoundNeighbours(std::list<Cell>& lCells, Cell& cell, size_t maxNeighbDiff, int maxCount){
    int x = cell.x;
    int y = cell.y;
    int val = cell.val;
    for(auto& neighb : neighbours){
        int xNeighb = x + neighb.first;
        int yNeighb = y + neighb.second;
        if(xNeighb < 0 || xNeighb >= mat.cols || yNeighb < 0 || yNeighb >= mat.rows)
            continue;
        int indNeighb = yNeighb * mat.cols + xNeighb;
        if(*((uint16_t*)(matBlob.data) + indNeighb))
            continue;
        uint16_t valNeighb =  *((uint16_t*)(mat.data) + indNeighb);
        if(valNeighb >= Params::getMaxKinectDepth()  || valNeighb == 0 || abs(valNeighb-val) >= maxNeighbDiff){
            continue;
        }
        *((uint16_t*)(matBlob.data) + indNeighb) = valNeighb;

        if(valNeighb - lCells.front().val > 150)
            continue;
        lCells.emplace_back(xNeighb, yNeighb, indNeighb, valNeighb);
        
        
        if(maxCount != -1 && lCells.size() == maxCount){
            return false;
        }
    }
    return true;
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
    Convex3d::extractConvexities1(mat, filterSize, filterDepth, coreHalfSize, countFalsePercent, true, inds, true);
    
    int minVal = this->cells.MinValCell()->val;
    cv::Mat matBlobClone = matBlob.clone();
    for(auto i : inds){
        //filter far
        int val =*((uint16_t*)(mat.data) + i);
        if(val > minVal + 150)
            continue;
        if(!*((uint16_t*)(matBlobClone.data) + i))
            continue;
        blobsFingers.emplace_back(matBlobClone,
                                  i,
                                  Params::getMaxNeighbDiffCoarse() * 0.25,
                                  1//Params::GET_BLOB_FINGER_MAX_SIZE()
                                  /*,Params::getMaxNeighbDiffCoarse()*/);
        for(const auto& cell : blobsFingers.back().cells.AllConst()){
            *((uint16_t*)(matBlobClone.data) + cell.ind) = 0;
        }
        blobsFingers.back().SetMat(matBlob);
    }
    if(blobsFingers.empty())
        return;
    
    BlobsClust<BlobExt> blobsFingersClust(blobsFingers, 2, 20, 2, true);
    blobsFingers = std::move(blobsFingersClust.getBlobsClust());
    BlobsClust<BlobExt> blobsFingersClust1(blobsFingers, 2, -1, 6, true);
    blobsFingers = std::move(blobsFingersClust1.getBlobsClust());
    
    /*for(auto& blob : blobsFingers)
        blob.Extend(Params::getMaxNeighbDiffCoarse() * 0.25, Params::getMaxNeighbDiffCoarse() * 0.05, 0.5);*/
        
    blobsFingers.sort([](const BlobExt<TContainer,T>& bl1, const BlobExt<TContainer,T>& bl2){return bl1.getCellsConst().MinValCell()->x < bl2.getCellsConst().MinValCell()->x;});
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
    
}

template<template<typename> class  TContainer, typename T>
bool BlobExt<TContainer, T>::IsAdjacent(const BlobExt& blob, int depthThresh) const{
    if(depthThresh < 0)
        return true;
    int testCount (this->cells.Size() * blob.cells.Size());
    int thresh = testCount * 0.75;
    auto& lCells = this->cells.AllConst();
    auto& lCells1 = blob.cells.AllConst();
    int count(0);
    int i (0);
    for(const auto& cell : lCells){
        for(const auto& cell1 : lCells1) {
            count += IsAdjacentCells(cell.x, cell.y, cell.val, cell1.x, cell1.y, depthThresh);
            if(count > thresh)
                return true;
            ++i;
            if(i - count > testCount - thresh)
                return false;
        }
    }
    return count > thresh;
}

template<template<typename> class  TContainer, typename T>
inline bool BlobExt<TContainer, T>::IsAdjacentCells(int x1, int y1, uint16_t val1, int x2, int y2, int depthThresh) const {
    if(x1 == x2 && y1 == y2)
        return true;
    float dx = x2 - x1;
    float dy = y2 - y1;
    float stepX = std::abs(dx) > std::abs(dy) ? dx / std::abs(dx) : dx / std::abs(dy);
    float stepY = std::abs(dx) < std::abs(dy) ? dy / std::abs(dy) : dy / std::abs(dx);
    int src = std::abs(dx) >  std::abs(dy) ? x1 : y1;
    int dst = std::abs(dx) >  std::abs(dy) ? x2 : y2;
    int step = src < dst ? 1 : -1;
    int val = val1;
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
    if(points.size() < 5)
        return;
    angles3dPtr = std::unique_ptr<Angles3d>(new Angles3d(points));
    /*const auto& anglesData = angles3dPtr->getDataConst();
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
    }
    testFeature = (count1  < count2) ? true : false;*/
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
    if(angles3dPtr){
    auto& anglesData = angles3dPtr->getData();
        for(auto & d : anglesData){
            auto& p = std::get<1>(d);
            p.x <<= resizePow;
            p.y <<= resizePow;
        }
    }
}

using BlobFinal = BlobExt<Vector, Cell>;

#endif /* blobext_hpp */
