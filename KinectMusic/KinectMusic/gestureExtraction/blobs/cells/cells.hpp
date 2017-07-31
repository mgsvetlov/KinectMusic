//
//  cells.hpp
//  KinectMusic
//
//  Created by Mikhail Svetlov on 15/07/17.
//  Copyright © 2017 mgsvetlov. All rights reserved.
//

#ifndef cells_hpp
#define cells_hpp

#include <type_traits>
#include "cell.h"
#include "../../../log/logs.h"

template<typename T>
using Vector = std::vector<T>;

template<template<typename> class TContainer, typename T> class Cells {
public:
    Cells();
    Cells(const Cells&) = delete;
    Cells(Cells&&);
    void AddCell(const T& cell);
    void AddCell(uint16_t x, uint16_t y, int ind, int val);
    void AddCell(uint16_t x, uint16_t y, int ind, int val, const T& cell);
    void Clear();
    TContainer<T>& All();
    const TContainer<T>& AllConst() const;
    const T* const MinValCell() const;
    const T* const MaxValCell() const;
    int AverageValue() const;
    cv::Point3i AveragedMinPoint(size_t pointsCount);
    int Size() const;
    void Merge(const Cells<TContainer,T>& cells1);
protected:
    void CheckBackMinMax();
    int Value(const T&, std::false_type) const;
    int Value(const T&, std::true_type) const;
    cv::Point3i Point3i(const T&, std::false_type) const;
    cv::Point3i Point3i(const T&, std::true_type) const;
    void AddCell(uint16_t x, uint16_t y, int ind, int val, std::false_type);
    void AddCell(uint16_t x, uint16_t y, int ind, int val, std::true_type);
    void AddCell(uint16_t x, uint16_t y, int ind, int val, const T& cell, std::false_type);
    void AddCell(uint16_t x, uint16_t y, int ind, int val, const T& cell, std::true_type);
protected:
    TContainer<T> cells;
    int minValInd = -1;
    int maxValInd = -1;
};

template<template<typename> class TContainer, typename T>
Cells<TContainer,T>::Cells() :
minValInd(NO_DATA_VALUE),
maxValInd(NO_DATA_VALUE)
{
}

template<template<typename> class  TContainer, typename T>
Cells<TContainer,T>::Cells(Cells&& other){
    std::move(other.cells.begin(), other.cells.end(), std::back_inserter(cells));
    minValInd = other.minValInd;
    maxValInd = other.maxValInd;
    
    other.Clear();
}

template<template<typename> class TContainer, typename T>
void Cells<TContainer,T>::AddCell(const T& cell){
    cells.push_back(cell);
    CheckBackMinMax();
}

template<template<typename> class  TContainer, typename T>
void Cells<TContainer,T>::AddCell(uint16_t x, uint16_t y, int ind, int val){
    AddCell(x, y, ind, val,  std::is_pointer<T>());
}

template<template<typename> class  TContainer, typename T>
void Cells<TContainer,T>::AddCell(uint16_t x, uint16_t y, int ind, int val,  std::false_type){
    cells.emplace_back(x, y, ind, val);
    CheckBackMinMax();
}

template<template<typename> class  TContainer, typename T>
void Cells<TContainer,T>::AddCell(uint16_t x, uint16_t y, int ind, int val, std::true_type){
    
}

template<template<typename> class  TContainer, typename T>
void Cells<TContainer,T>::AddCell(uint16_t x, uint16_t y, int ind, int val, const T& cell){
    AddCell(x, y, ind, val, cell,  std::is_pointer<T>());
}

template<template<typename> class  TContainer, typename T>
void Cells<TContainer,T>::AddCell(uint16_t x, uint16_t y, int ind, int val, const T& cell, std::false_type){
    cells.emplace_back(x, y, ind, val, cell);
    CheckBackMinMax();
}

template<template<typename> class  TContainer, typename T>
void Cells<TContainer,T>::AddCell(uint16_t x, uint16_t y, int ind, int val, const T& cell, std::true_type){
    
}

template<template<typename> class  TContainer, typename T>
void Cells<TContainer,T>::Clear(){
    cells.clear();
    minValInd = maxValInd = NO_DATA_VALUE;
}

template<template<typename> class  TContainer, typename T>
TContainer<T>& Cells<TContainer,T>::All() {
    return cells;
}

template<template<typename> class  TContainer, typename T>
const TContainer<T>& Cells<TContainer,T>::AllConst() const {
    return cells;
}

template<template<typename> class  TContainer, typename T>
const T* const Cells<TContainer,T>::MinValCell() const{
    if(minValInd == NO_DATA_VALUE)
       return nullptr;
    return &cells.at(minValInd);
}

template<template<typename> class  TContainer, typename T>
const T* const Cells<TContainer,T>::MaxValCell() const{
    if(maxValInd == NO_DATA_VALUE)
        return nullptr;
    return &cells.at(maxValInd);
}

template<template<typename> class  TContainer, typename T>
int Cells<TContainer,T>::AverageValue() const{
    int sum (0);
    for( auto& cell: cells){
        sum += Value(cell, std::is_pointer<T>());
    }
    return sum / cells.size();
}

template<template<typename> class  TContainer, typename T>
cv::Point3i Cells<TContainer,T>::AveragedMinPoint(size_t pointsCount) {    
    if(pointsCount < cells.size()){
        std::nth_element(cells.begin(), cells.begin() + pointsCount, cells.end(), [this](const T& cell1, const T& cell2){ return Value(cell1, std::is_pointer<T>()) < Value(cell2, std::is_pointer<T>());} );
    }
    size_t count = pointsCount < cells.size() ? pointsCount : cells.size();
    cv::Point3i  avMinPoint(0);
    for(size_t i = 0; i < count; ++i){
        avMinPoint += Point3i(cells[i], std::is_pointer<T>());
    }
    avMinPoint *= 1. / count;
    return avMinPoint;
}

template<template<typename> class  TContainer, typename T>
int Cells<TContainer,T>::Size() const {
    return static_cast<int>(cells.size());
}

template<template<typename> class  TContainer, typename T>
void Cells<TContainer,T>::Merge(const Cells<TContainer,T>& cells1){
    for(auto& cell : cells1.AllConst()){
        cells.push_back(cell);
        CheckBackMinMax();
    }
}

template<template<typename> class  TContainer, typename T>
void Cells<TContainer,T>::CheckBackMinMax(){
    auto val = Value(cells.back(), std::is_pointer<T>());
    if(minValInd == NO_DATA_VALUE || val < Value(cells[minValInd], std::is_pointer<T>()))
        minValInd = static_cast<int>(cells.size() - 1);
    if(maxValInd == NO_DATA_VALUE || val < Value(cells[maxValInd], std::is_pointer<T>()))
        maxValInd = static_cast<int>(cells.size() - 1);
}

template<template<typename> class  TContainer, typename T>
int Cells<TContainer,T>::Value(const T& cell, std::false_type) const{
    return cell.val;
}

template<template<typename> class  TContainer, typename T>
int Cells<TContainer,T>::Value(const T& cell, std::true_type) const{
    return cell->val;
}

template<template<typename> class  TContainer, typename T>
cv::Point3i Cells<TContainer,T>::Point3i(const T& cell, std::false_type) const {
    return cv::Point3i(cell.x, cell.y, cell.val);
}

template<template<typename> class  TContainer, typename T>
cv::Point3i Cells<TContainer,T>::Point3i(const T& cell, std::true_type) const {
    return cv::Point3i(cell->x, cell->y, cell->val);
}

#endif /* cells_hpp */
