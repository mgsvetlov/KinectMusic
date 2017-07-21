//
//  border.hpp
//  KinectMusic
//
//  Created by Mikhail Svetlov on 19/07/17.
//  Copyright © 2017 mgsvetlov. All rights reserved.
//

#ifndef border_hpp
#define border_hpp

#include "../cells/cellsborder.hpp"
#include "../blobext.hpp"
#include "borderclustrow.h"

template<typename T>
using List = std::list<T>;

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
    //create clusters
    for(const auto& cell :  borderCells1.AllConst())
        AddCellToRow(cell);
    for(const auto& cell :  borderCells2.AllConst())
        AddCellToRow(cell);
    for(auto& row : borderClusts)
        if(!row.clusts.empty())
            row.MergeClusts();
    //create border tree from clusters
    for(auto& row : borderClusts){
        auto it = row.clusts.begin();
        for( ;it != row.clusts.end(); ++it){
            if(!it->isInBorder)
                CreateBorder(&(*it));
        }
    }
    
    if(borders.front().back()->child == borders.front().front())
        return;
    std::stringstream ss;
    ss << "\nBlob:\n";
    int i(0);
    for(auto& border : borders){
        auto& head = border.front();
        ss << "size " << border.size() << " " ;
        for(auto& clust : border){
            ss << "<y " << clust->y << " " << clust->first << "-" << clust->last
            << (clust->child == head? " parent = head " : "")
            << ">";
        }
        ++i;
        ss << "\n";
    }
    
    Logs::writeLog("gestures", ss.str());
    //borders.sort([](const std::list<BorderClust*>& cl1, const std::list<BorderClust*>& cl2){return cl1.size() > cl2.size();});
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
    clustHead->isInBorder = true;
    borders.emplace_back(1, clustHead);
    auto& border = borders.back();
    auto it = border.begin();
    for( ;it != border.end(); ++it){
        auto& clust = **it;
        uint16_t y = clust.y;
        std::list<BorderClustRow*> adjacentRows;
        if(y > 0)
            adjacentRows.push_back(&borderClusts[y - 1]);
        if(y < mat.rows - 1)
            adjacentRows.push_back(&borderClusts[y + 1]);
        std::list<BorderClust*> clustAdj;
        for(auto& adjacentRow : adjacentRows){
            auto clusts =  adjacentRow->FindAdjacentChildren(clust);
            clustAdj.insert(clustAdj.end(), clusts.begin(), clusts.end()); //TODO move!!
        }
        auto ccwAdjClust = clust.AdjClustCCW(clustAdj);
        if(ccwAdjClust)
            clustAdj = std::list<BorderClust*>(1, ccwAdjClust);
        
        for(auto& child : clustAdj){
            clust.child = child;
            child->parent = &clust;
            if(!child->isInBorder){
                border.push_back(child);
                child->isInBorder = true;
            }
        }
    }
}

#endif /* border_hpp */
