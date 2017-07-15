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
#include "../types.h"

class SubBlob;

struct Cell {
    Cell(){}
    Cell(int ind, int val): ind(ind), val(val){}
    Cell(int ind, int val, float dist): ind(ind), val(val), dist(dist){}
    int ind = NO_DATA_VALUE;
    int val = NO_DATA_VALUE;
    float dist = 0;
    Cell* parent = nullptr;
    Cell* child = nullptr;
    SubBlob* subBlob = nullptr;
    cv::Vec4f normal = cv::Vec4f(0.f, 0.f, 0.f);
};

template<typename T> class Cells {
public:
    Cells(){}
    Cells(const Cells&) = delete;
    Cells(Cells&&);
    void AddCell(const T& cell);
    void AddCell(int ind, int val, int dist = 0);
    void Clear();
    std::list<T>& All();
    const std::list<T>& AllConst() const;
    const T* const MaxValCell() const;
    const T* const MinValCell() const;
    int AverageValue() const;
    int Size() const;
    void Merge(const Cells<T>& cells1);
private:
    void CheckBackMinMax();
    int Value(const T&, std::false_type) const;
    int Value(const T&, std::true_type) const;
    void AddCell(int ind, int val, int dist, std::false_type);
    void AddCell(int ind, int val, int dist, std::true_type);
private:
    std::list<T> cells;
    T* p_maxValCell = nullptr;
    T* p_minValCell = nullptr;
};

template<typename T> Cells<T>::Cells(Cells&& other){
    if(!other.p_maxValCell && !other.p_minValCell)
        return;
    int minNum (-1), maxNum (-1);
    int count (0);
    for(const auto& cell : other.cells){
        if(other.p_minValCell == &cell)
            minNum = count;
        if(other.p_maxValCell == &cell)
            maxNum = count;
        ++count;
    }
    
    std::move(other.cells.begin(), other.cells.end(), std::back_inserter(cells));
    
    p_minValCell = &(*next(cells.begin(), minNum));
    p_maxValCell = &(*next(cells.begin(), maxNum));
    
    other.Clear();
}

template<typename T> void Cells<T>::AddCell(const T& cell){
    cells.push_back(cell);
    CheckBackMinMax();
}

template<typename T> void Cells<T>::AddCell(int ind, int val, int dist){
    AddCell(ind, val, dist, std::is_pointer<T>());
}

template<typename T> void Cells<T>::AddCell(int ind, int val, int dist, std::false_type){
    cells.emplace_back(Cell(ind, val, dist));
    CheckBackMinMax();
}

template<typename T> void Cells<T>::AddCell(int ind, int val, int dist, std::true_type){
    
}

template<typename T> void Cells<T>::Clear(){
    p_maxValCell = p_minValCell= nullptr;
    cells.clear();
}

template<typename T> std::list<T>& Cells<T>::All() {
    return cells;
}

template<typename T> const std::list<T>& Cells<T>::AllConst() const {
    return cells;
}

template<typename T> const T* const Cells<T>::MaxValCell() const{
    return p_maxValCell;
}

template<typename T> const T* const Cells<T>::MinValCell() const{
    return p_minValCell;
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
    if(!p_minValCell || val < Value(*p_minValCell, std::is_pointer<T>()))
        p_minValCell = &cells.back();
    if(!p_maxValCell || val > Value(*p_maxValCell, std::is_pointer<T>()))
        p_maxValCell = &cells.back();
}

template<typename T> int Cells<T>::Value(const T& cell, std::false_type) const{
    return cell.val;
}

template<typename T> int Cells<T>::Value(const T& cell, std::true_type) const{
    return cell->val;
}

#endif /* cells_hpp */
