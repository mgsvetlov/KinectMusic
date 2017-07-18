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
#include "cell.hpp"
#include "../../log/logs.h"

template<typename T>
using CellsContainer = std::vector<T>;

template<typename T> class Cells {
public:
    Cells();
    Cells(const Cells&) = delete;
    Cells(Cells&&);
    void AddCell(const T& cell);
    void AddCell(uint16_t x, uint16_t y, int ind, int val);
    void AddCell(uint16_t x, uint16_t y, int ind, int val, const T& cell);
    void Clear();
    CellsContainer<T>& All();
    const CellsContainer<T>& AllConst() const;
    const T* const MinValCell() const;
    const T* const MaxValCell() const;
    int AverageValue() const;
    int Size() const;
    void Merge(const Cells<T>& cells1);
private:
    void CheckBackMinMax();
    int Value(const T&, std::false_type) const;
    int Value(const T&, std::true_type) const;
    void AddCell(uint16_t x, uint16_t y, int ind, int val, std::false_type);
    void AddCell(uint16_t x, uint16_t y, int ind, int val, std::true_type);
    void AddCell(uint16_t x, uint16_t y, int ind, int val, const T& cell, std::false_type);
    void AddCell(uint16_t x, uint16_t y, int ind, int val, const T& cell, std::true_type);
private:
    CellsContainer<T> cells;
    int minValInd = -1;
    int maxValInd = -1;
};

template<typename T> Cells<T>::Cells() :
minValInd(NO_DATA_VALUE),
maxValInd(NO_DATA_VALUE)
{
}

template<typename T> Cells<T>::Cells(Cells&& other){
    std::move(other.cells.begin(), other.cells.end(), std::back_inserter(cells));
    minValInd = other.minValInd;
    maxValInd = other.maxValInd;
    
    other.Clear();
}

template<typename T> void Cells<T>::AddCell(const T& cell){
    cells.push_back(cell);
    CheckBackMinMax();
}

template<typename T> void Cells<T>::AddCell(uint16_t x, uint16_t y, int ind, int val){
    AddCell(x, y, ind, val,  std::is_pointer<T>());
}

template<typename T> void Cells<T>::AddCell(uint16_t x, uint16_t y, int ind, int val,  std::false_type){
    cells.emplace_back(x, y, ind, val);
    CheckBackMinMax();
}

template<typename T> void Cells<T>::AddCell(uint16_t x, uint16_t y, int ind, int val, std::true_type){
    
}

template<typename T> void Cells<T>::AddCell(uint16_t x, uint16_t y, int ind, int val, const T& cell){
    AddCell(x, y, ind, val, cell,  std::is_pointer<T>());
}

template<typename T> void Cells<T>::AddCell(uint16_t x, uint16_t y, int ind, int val, const T& cell, std::false_type){
    cells.emplace_back(x, y, ind, val, cell);
    CheckBackMinMax();
}

template<typename T> void Cells<T>::AddCell(uint16_t x, uint16_t y, int ind, int val, const T& cell, std::true_type){
    
}

template<typename T> void Cells<T>::Clear(){
    cells.clear();
    minValInd = maxValInd = NO_DATA_VALUE;
}

template<typename T> CellsContainer<T>& Cells<T>::All() {
    return cells;
}

template<typename T> const CellsContainer<T>& Cells<T>::AllConst() const {
    return cells;
}

template<typename T> const T* const Cells<T>::MinValCell() const{
    if(minValInd == NO_DATA_VALUE)
       return nullptr;
    return &cells.at(minValInd);
}

template<typename T> const T* const Cells<T>::MaxValCell() const{
    if(maxValInd == NO_DATA_VALUE)
        return nullptr;
    return &cells.at(maxValInd);
}

template<typename T> int Cells<T>::AverageValue() const{
    int sum (0);
    for( auto& cell: cells){
        sum += Value(cell, std::is_pointer<T>());
    }
    return sum / cells.size();
}

template<typename T> int Cells<T>::Size() const {
    return static_cast<int>(cells.size());
}

template<typename T> void Cells<T>::Merge(const Cells<T>& cells1){
    for(auto& cell : cells1.AllConst()){
        cells.push_back(cell);
        CheckBackMinMax();
    }
}

template<typename T> void Cells<T>::CheckBackMinMax(){
    auto val = Value(cells.back(), std::is_pointer<T>());
    if(minValInd == NO_DATA_VALUE || val < Value(cells[minValInd], std::is_pointer<T>()))
        minValInd = static_cast<int>(cells.size() - 1);
    if(maxValInd == NO_DATA_VALUE || val < Value(cells[maxValInd], std::is_pointer<T>()))
        maxValInd = static_cast<int>(cells.size() - 1);
}

template<typename T> int Cells<T>::Value(const T& cell, std::false_type) const{
    return cell.val;
}

template<typename T> int Cells<T>::Value(const T& cell, std::true_type) const{
    return cell->val;
}

#endif /* cells_hpp */
