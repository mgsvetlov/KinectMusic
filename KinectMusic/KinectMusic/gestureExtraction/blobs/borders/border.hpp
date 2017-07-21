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

template<typename T>
using List = std::list<T>;

static std::map<int, std::pair<int, int>> int2pair {
    {0, std::pair<int, int>{1,-1}},
    {1, std::pair<int, int>{0,-1}},
    {2, std::pair<int, int>{-1,-1}},
    {3, std::pair<int, int>{-1,0}},
    {4, std::pair<int, int>{-1,1}},
    {5, std::pair<int, int>{0,1}},
    {6, std::pair<int, int>{1,1}},
    {7, std::pair<int, int>{1,0}},
};

static std::map<std::pair<int, int>, int> pair2int {
    {std::pair<int, int>{1,-1}, 0},
    {std::pair<int, int>{0,-1}, 1},
    {std::pair<int, int>{-1,-1}, 2},
    {std::pair<int, int>{-1,0}, 3},
    {std::pair<int, int>{-1,1}, 4},
    {std::pair<int, int>{0,1}, 5},
    {std::pair<int, int>{1,1}, 6},
    {std::pair<int, int>{1,0}, 7},
};

template<template<typename> class TContainer, typename T> class Border {
public:
    Border(const cv::Mat& mat, Cells<TContainer,T>& cells);
    std::list<T*>& getContour() { return contour;}
private:
    T* nextCell(const cv::Mat& matCells, T* cell, T* cellPrev);
private:
    const cv::Mat mat;
    Cells<TContainer,T>& cells;
    std::list<T*> contour;
    friend class Visualization;
};

template<template<typename> class TContainer, typename T>
Border<TContainer,T>::Border(const cv::Mat& mat, Cells<TContainer,T>& cells) :
mat(mat),
cells(cells)
{
    cv::Mat matCells  = cv::Mat_<T*>::zeros(mat.size());
    T** p_matCells = (T**)(matCells.data);
    for(auto& cell : cells.All()){
        *(p_matCells + cell.ind) = &cell;
    }
    //find first cell
    T** p = p_matCells;
    for(int i = 0; i < matCells.total(); ++i){
        if(*(p+i) != nullptr){
            contour.push_back(*(p_matCells + i));
            break;
        }
    }
    if(contour.empty()) {
        return;
    }
    //find second cell
    auto x = contour.back()->x;
    auto y = contour.back()->y;
    for(int ind = 0; ind <= 7; ++ind){
        std::pair<int, int> diff1 = int2pair[ind];
        T* pcell = matCells.at<T*>(y + diff1.second, x + diff1.first);
        if(pcell){
            contour.push_back(pcell);
            break;
        }
    }
    
    if(contour.size() == 1){
        return;
    }
    //find second cells
    while(true){
        auto itPrevLast = contour.rbegin();
        T* next = nextCell(matCells, *contour.rbegin(), *(++itPrevLast));
        if(next == contour.front()) {
            break;
        }
        contour.push_back(next);
    }
}

template<template<typename> class TContainer, typename T>
T* Border<TContainer,T>::nextCell(const cv::Mat& matCells, T* cell, T* cellPrev){
    T* next = nullptr;
    auto x = cell->x, y = cell->y;
    std::pair<int, int> diff {cellPrev->x - x, cellPrev->y - y};
    int ind = pair2int[diff];
    for(int i = 0; i < 8; ++i){
        (++ind) %= 8;
        std::pair<int, int> diff1 = int2pair[ind];
        T* pcell = matCells.at<T*>(y + diff1.second, x + diff1.first);
        if(pcell){
            next = pcell;
            break;
        }
    }
    return next;
}



#endif /* border_hpp */
