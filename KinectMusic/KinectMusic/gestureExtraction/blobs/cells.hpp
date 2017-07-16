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

template<typename T> class Cells {
public:
    Cells(){}
    Cells(const Cells&) = delete;
    Cells(Cells&&);
    void AddCell(const T& cell);
    void AddCell(uint16_t x, uint16_t y, int ind, int val);
    void AddCell(uint16_t x, uint16_t y, int ind, int val, const T& cell);
    void Clear();
    std::list<T>& All();
    const std::list<T>& AllConst() const;
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
    std::list<T> cells;
    typename std::list<T>::const_iterator it_minValCell = cells.cend();
    typename std::list<T>::const_iterator it_maxValCell = cells.cend();
};

template<typename T> Cells<T>::Cells(Cells&& other){
    size_t minNum = distance(other.cells.cbegin(), other.it_minValCell);
    size_t maxNum = distance(other.cells.cbegin(), other.it_maxValCell);
    
    std::move(other.cells.begin(), other.cells.end(), std::back_inserter(cells));
    
    it_minValCell = next(cells.cbegin(), minNum);
    it_maxValCell = next(cells.cbegin(), maxNum);
    
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
    cells.emplace_back(Cell(x, y, ind, val));
    CheckBackMinMax();
}

template<typename T> void Cells<T>::AddCell(uint16_t x, uint16_t y, int ind, int val, std::true_type){
    
}

template<typename T> void Cells<T>::AddCell(uint16_t x, uint16_t y, int ind, int val, const T& cell){
    AddCell(x, y, ind, val, cell,  std::is_pointer<T>());
}

template<typename T> void Cells<T>::AddCell(uint16_t x, uint16_t y, int ind, int val, const T& cell, std::false_type){
    cells.emplace_back(Cell(x, y, ind, val, cell));
    CheckBackMinMax();
}

template<typename T> void Cells<T>::AddCell(uint16_t x, uint16_t y, int ind, int val, const T& cell, std::true_type){
    
}

template<typename T> void Cells<T>::Clear(){
    cells.clear();
    it_minValCell = it_maxValCell = cells.cend();
}

template<typename T> std::list<T>& Cells<T>::All() {
    return cells;
}

template<typename T> const std::list<T>& Cells<T>::AllConst() const {
    return cells;
}

template<typename T> const T* const Cells<T>::MinValCell() const{
    if(cells.empty())
       return nullptr;
    return(&(*it_minValCell));
}

template<typename T> const T* const Cells<T>::MaxValCell() const{
    if(cells.empty())
        return nullptr;
    return(&(*it_maxValCell));
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
    
    if(cells.empty()  || val < Value(*it_minValCell, std::is_pointer<T>()))
        it_minValCell-- = cells.cend();
    if(cells.empty()  || val > Value(*it_maxValCell, std::is_pointer<T>()))
        it_maxValCell-- = cells.cend();
}

template<typename T> int Cells<T>::Value(const T& cell, std::false_type) const{
    return cell.val;
}

template<typename T> int Cells<T>::Value(const T& cell, std::true_type) const{
    return cell->val;
}

#endif /* cells_hpp */
