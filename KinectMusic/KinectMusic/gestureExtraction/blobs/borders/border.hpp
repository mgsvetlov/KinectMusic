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

template<template<typename> class TContainer, typename T> class Border {
public:
    Border(const cv::Mat& mat, Cells<TContainer,T>& cells);
    std::list<T*>& getContour() { return contour;}
private:
    bool createContour();
    T* nextCell(const cv::Mat& matCells, uint16_t x, uint16_t y, int indDiff);
private:
    const cv::Mat mat;
    Cells<TContainer,T>& cells;
    std::list<T*> contour;
    static std::vector<std::pair<int, int>> int2pair;
    static std::map<std::pair<int, int>, int> pair2int;
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
    if(!createContour()){
        contour.clear();
        return;
    }
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
            contour.push_back(*(p_matCells + i));
            break;
        }
    }
    if(contour.empty()) {
        return false;
    }
    //find second cell
    auto x = contour.back()->x;
    auto y = contour.back()->y;
    T* next = nextCell(matCells, x, y, 3);
    if(!next){
        return false;
    }
    contour.push_back(next);
    //find all cells
    while(true){
        auto itPrevLast = contour.rbegin();
        ++itPrevLast;
        auto x = contour.back()->x, y = contour.back()->y;
        std::pair<int, int> diff {(*itPrevLast)->x - x, (*itPrevLast)->y - y};
        int indDiff = pair2int[diff];
        T* next = nextCell(matCells, x, y, indDiff);
        if(next == contour.front()) {
            break;
        }
        contour.push_back(next);
    }
    return true;
}

template<template<typename> class TContainer, typename T>
T* Border<TContainer,T>::nextCell(const cv::Mat& matCells, uint16_t x, uint16_t y, int indDiff){
    int w = matCells.cols;
    int h = matCells.rows;
    T* next = nullptr;
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
            break;
        }
    }
    return next;
}



#endif /* border_hpp */
