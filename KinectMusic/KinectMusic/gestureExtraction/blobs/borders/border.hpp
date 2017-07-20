//
//  border.hpp
//  KinectMusic
//
//  Created by Mikhail Svetlov on 19/07/17.
//  Copyright © 2017 mgsvetlov. All rights reserved.
//

#ifndef border_h
#define border_h

#include "../cells/cellsborder.hpp"
#include "../blobext.hpp"

template<typename T>
using List = std::list<T>;

struct BorderClust {
    BorderClust(uint16_t y, uint16_t x, uint16_t val, uint16_t parentVal) : y(y), first(x), last(x), valAvg(val), parentValAvg(parentVal) {}
    void Merge(BorderClust clust){
        int size = last - first + 1;
        int size1 = clust.last - clust.first;
        valAvg = (valAvg * size + clust.valAvg * size1) / (size + size1);
        parentValAvg = (parentValAvg * size + clust.parentValAvg * size1) / (size + size1);
        last = clust.last;
    }
    bool IsAdjacent(const BorderClust& cluster) const {
        if(parent != nullptr)
            return false;
        int min = first < cluster.first ? first : cluster.first;
        int max = last > cluster.last ? last : cluster.last;
        return max - min <= last - first + cluster.last - cluster.first + 1;
    }
    void AddChild(BorderClust* child){
        childs.push_back(child);
        child->parent = this;
    }
    uint16_t y;
    uint16_t first;
    uint16_t last;
    uint16_t valAvg;
    uint16_t parentValAvg;
    BorderClust* parent = nullptr;
    std::list<BorderClust*> childs;
};

struct  BorderClustRow {
    void MergeClusts() {
        clusts.sort([](const BorderClust& cl1, const BorderClust& cl2){return cl1.first < cl2.first;});
        auto it = clusts.begin();
        while(it != clusts.end()) {
            auto it1 = it;
            ++it1;
            if(it1 == clusts.end())
               break;
            if(it->last == it1->first - 1){
                it->Merge(*it1);
                clusts.erase(it1);
                continue;
            }
            ++it;
        }
    }
    std::list<BorderClust*> FindAdjacentChilds(const BorderClust& cluster)  {
        std::list<BorderClust*> clustsAdj;
        for(auto& clust : clusts){
            if(clust.IsAdjacent(cluster))
                clustsAdj.push_back(&clust);
        }
        return clustsAdj;
    }
    
    std::list<BorderClust> clusts;
};

template<template<typename> class TContainer, typename T> class Border {
public:
    Border(const cv::Mat& mat, const Cells<TContainer,T>& cells, const CellsBorder<TContainer, CellBorder>& borderCells1, const CellsBorder<TContainer, CellBorder>& borderCells2);
private:
    void AddCellToRow(const CellBorder& cell);
    void CreateBorder(BorderClust* clust);
    
private:
    const cv::Mat mat;
    const Cells<TContainer,T>& cells;
    const CellsBorder<TContainer, CellBorder>& borderCells1;
    const CellsBorder<TContainer, CellBorder>& borderCells2;
    std::vector<BorderClustRow> borderClusts;
    std::list<std::list<BorderClust*>> borders;
    friend class Visualization;
};

template<template<typename> class TContainer, typename T>
Border<TContainer,T>::Border(const cv::Mat& mat, const Cells<TContainer,T>& cells, const CellsBorder<TContainer, CellBorder>& borderCells1, const CellsBorder<TContainer, CellBorder>& borderCells2) :
mat(mat),
cells(cells),
borderCells1(borderCells1),
borderCells2(borderCells2),
borderClusts(mat.rows, BorderClustRow()),
borders()
{
    for(const auto& cell :  borderCells1.AllConst())
        AddCellToRow(cell);
    for(const auto& cell :  borderCells2.AllConst())
        AddCellToRow(cell);
    for(auto& row : borderClusts)
        if(!row.clusts.empty())
            row.MergeClusts();
    
    for(auto& row : borderClusts){
        for(auto& clust : row.clusts){
            if(clust.parent == nullptr)
                CreateBorder(&clust);
        }
    }
   
    
}

template<template<typename> class TContainer, typename T>
void Border<TContainer,T>::AddCellToRow(const CellBorder& cell) {
    BorderClustRow& row = borderClusts[cell.y];
    uint16_t x = cell.x;
    uint16_t y = cell.y;
    uint16_t val = cell.val;
    uint16_t parentVal = cells.AllConst()[cell.parentInd].val;
    row.clusts.emplace_back(y, x, val, parentVal);
}

template<template<typename> class TContainer, typename T>
void Border<TContainer,T>::CreateBorder(BorderClust* clustHead){
    borders.push_back(std::list<BorderClust*>());
    auto& border = borders.back();
    border.push_back(clustHead);
    clustHead->parent = clustHead;
    auto it = border.begin();
    for( ;it != border.end(); ++it){
        auto& clust = **it;
        uint16_t y = clust.y;
        if(y > 0){
            auto& rowPrev = borderClusts[y - 1];
            auto clustAdjPrev =  rowPrev.FindAdjacentChilds(clust);
            for(auto& child : clustAdjPrev){
                clust.AddChild(child);
                border.push_back(child);
            }
        }
        if(y < mat.rows - 1){
            auto& rowNext = borderClusts[y + 1];
            auto clustAdjNext =  rowNext.FindAdjacentChilds(clust);
            for(auto& child : clustAdjNext){
                clust.AddChild(child);
                border.push_back(child);
            }
        }
    }
    
}


#endif /* border_h */
