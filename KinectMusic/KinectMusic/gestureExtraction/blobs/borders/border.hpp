//
//  border.hpp
//  KinectMusic
//
//  Created by Mikhail Svetlov on 19/07/17.
//  Copyright © 2017 mgsvetlov. All rights reserved.
//

#ifndef border_hpp
#define border_hpp

#include <map>
#include "../blobext.hpp"
#include "../../params.h"

template<typename T>
using List = std::list<T>;

template<template<typename> class TContainer, typename T> class Border {
public:
    Border(const cv::Mat& mat, Cells<TContainer,T>& cells);
    bool isNotHandContour();
    const std::list<CellContour>& getContour() const { return contour;}
private:
    bool createContour();
    T* nextCell(const cv::Mat& matCells, CellContour& cell, int indDiff);
    int eraseLoops();
    bool checkContour();
    void countBodyAdjacentParts();
private:
    const cv::Mat mat;
    Cells<TContainer,T>& cells;
    std::list<CellContour> contour;
    int bodyAdjacentCount = 0;
    int nonBodyAdjacentCount = 0;
    int bodyAdjacentPartsCount = 0;
    static std::vector<std::pair<int, int>> int2pair;
    static std::map<std::pair<int, int>, int> pair2int;
    
    template<template<typename> class UContainer, typename U>
    friend class BlobExt;
    friend class Visualization;
};

template<template<typename> class TContainer, typename T>
std::vector<std::pair<int, int>> Border<TContainer,T>::int2pair {
    {std::pair<int, int>{1,-1}},
    {std::pair<int, int>{0,-1}},
    {std::pair<int, int>{-1,-1}},
    {std::pair<int, int>{-1,0}},
    {std::pair<int, int>{-1,1}},
    {std::pair<int, int>{0,1}},
    {std::pair<int, int>{1,1}},
    {std::pair<int, int>{1,0}},
};

template<template<typename> class TContainer, typename T>
std::map<std::pair<int, int>, int> Border<TContainer,T>::pair2int {
    {std::pair<int, int>{1,-1}, 0},
    {std::pair<int, int>{0,-1}, 1},
    {std::pair<int, int>{-1,-1}, 2},
    {std::pair<int, int>{-1,0}, 3},
    {std::pair<int, int>{-1,1}, 4},
    {std::pair<int, int>{0,1}, 5},
    {std::pair<int, int>{1,1}, 6},
    {std::pair<int, int>{1,0}, 7},
};

template<template<typename> class TContainer, typename T>
Border<TContainer,T>::Border(const cv::Mat& mat, Cells<TContainer,T>& cells) :
mat(mat),
cells(cells)
{
    if(cells.Size() == 0)
        return;
    if(!createContour()){
        contour.clear();
        return;
    }
    eraseLoops();
    checkContour();
    countBodyAdjacentParts();
}

template<template<typename> class TContainer, typename T>
bool Border<TContainer,T>::createContour(){
    cv::Mat matCells  = cv::Mat_<T*>::zeros(mat.size());
    T** p_matCells = (T**)(matCells.data);
    for(auto& cell : cells.All()){
        *(p_matCells + cell.ind) = &cell;
    }
    //find first cell
    T** p = p_matCells;
    for(int i = 0; i < matCells.total(); ++i){
        if(*(p+i) != nullptr){
            contour.emplace_back(**(p_matCells + i));
            break;
        }
    }
    if(contour.empty()) {
        return false;
    }
    //find second cell
    T* next = nextCell(matCells, contour.back(), 7);
    if(!next){
        return false;
    }
    contour.emplace_back(*next);
    //find all cells
    while(true){
        auto itPrevLast = contour.rbegin();
        ++itPrevLast;
        auto& cell = contour.back();
        std::pair<int, int> diff {(*itPrevLast).x - cell.x, (*itPrevLast).y - cell.y};
        int indDiff = pair2int[diff];
        T* next = nextCell(matCells, cell, indDiff);
        if(next->ind == contour.front().ind) {
            break;
        }
        contour.emplace_back(*next);
    }
    return nonBodyAdjacentCount > bodyAdjacentCount;
}

template<template<typename> class TContainer, typename T>
T* Border<TContainer,T>::nextCell(const cv::Mat& matCells, CellContour& cell, int indDiff){
    int w = matCells.cols;
    int h = matCells.rows;
    uint16_t x = cell.x;
    uint16_t y = cell.y;
    T* next = nullptr;
    uint16_t valOutMin(Params::getMaxKinectDepth());
    for(int i = 0; i < 8; ++i){
        (++indDiff) %= 8;
        std::pair<int, int> diff = int2pair[indDiff];
        int xNeighb = x + diff.first;
        int yNeighb = y + diff.second;
        if(xNeighb < 0 || xNeighb >= w || yNeighb < 0 || yNeighb >= h)
            continue;
        T* pcell = matCells.at<T*>(yNeighb, xNeighb);
        if(pcell){
            next = pcell;
            cell.valOut = valOutMin;
            int valDiff = abs(cell.val - cell.valOut);
            if( valDiff <= Params::getMaxHeighbDiffCoarse()) {
                cell.flags |= FLAGS::ADJACENT_BODY;
                ++bodyAdjacentCount;
            }
            else {
                ++nonBodyAdjacentCount;
            }
            break;
        }
        else {
            auto valOut = mat.at<uint16_t>(yNeighb, xNeighb);
            if(valOut < valOutMin)
                valOutMin = valOut;
        }
    }
    return next;
}

template<template<typename> class TContainer, typename T>
int Border<TContainer,T>::eraseLoops(){
    //std::stringstream ss;
    //ss << "eraseLoops ";
    int eraseCount(0);
    cv::Mat matMask  = cv::Mat_<unsigned char>::zeros(mat.size());
    unsigned char* p_matMask = (unsigned char*)(matMask.data);
    auto it = contour.begin();
    while( it != contour.end()) {
        int ind = it->ind;
        int maskVal = *(p_matMask + ind);
        if(maskVal == 0){
            *(p_matMask + ind) = 255;
        }
        else {
            auto it1 = contour.begin();
            for(; it1 != it; ++it1){
                if(it1->ind == ind)
                    break;
            }
            if(it1 != it) {
                int dist = static_cast<int>(std::distance(it1, it));
                if( dist < (contour.size() >> 1)){
                    it = contour.erase(it1, it);
                    eraseCount += dist;
                    //ss << dist << " erased ";
                }
                else {
                    contour.erase(it, contour.end());
                    contour.erase(contour.begin(), it1);
                    eraseCount += contour.size() - dist;
                    //ss << contour.size() - dist << " erased at end ";
                    break;
                }
            }
            else {
                std::stringstream ss;
                ss << "\nsize " << contour.size() << " dist " << std::distance(contour.begin(), it)
                << " error!\n";
                Logs::writeLog("gestures", ss.str());
            }
        }
        ++it;
    }
    //Logs::writeLog("gestures", ss.str());
    return eraseCount;
}

template<template<typename> class TContainer, typename T>
bool Border<TContainer,T>::checkContour(){
    if(contour.empty())
        return true;
    auto it1 = contour.begin();
    auto it2 = it1;
    ++it2;
    auto itPreEnd = contour.end();
    --itPreEnd;
    bool isEnd(false);
    for(; it2 != contour.end(); ++it1, ++it2){
        if(it2 == itPreEnd){
            it1 = contour.begin();
            isEnd = true;
        }
        if(!CellContour::IsNeighbours(*it1, *it2)){
            std::stringstream ss;
            for(auto it3 = contour.begin(); it3 != it1; ++it3)
                ss << *it3;
            ss << (!isEnd ? "disconnection!" : "disconnection end!");
            for(auto it3 = it2; it3 != contour.end(); ++it3)
                ss << *it3;
            ss  << std::flush;
            Logs::writeLog("gestures", ss.str());
            std::exit(EXIT_FAILURE);
        }
    }
    return true;
}

template<template<typename> class TContainer, typename T>
void Border<TContainer,T>::countBodyAdjacentParts(){
    bool isCurrAdjacentPart(false), isFirstAdjacentPart(false);
    bool isBegin(true), isAdjacent(false);
    for(auto& cell : contour){
        isAdjacent = cell.flags & FLAGS::ADJACENT_BODY;
        if(isBegin){
            if(isAdjacent)
                isFirstAdjacentPart = true;
            isBegin = false;
        }
        if(!isCurrAdjacentPart && isAdjacent){
            isCurrAdjacentPart = true;
            ++bodyAdjacentPartsCount;
        }
        else if(isCurrAdjacentPart && !isAdjacent){
            isCurrAdjacentPart = false;
        }
    }
    if(isAdjacent && isFirstAdjacentPart)
        --bodyAdjacentPartsCount;
}
#endif /* border_hpp */
