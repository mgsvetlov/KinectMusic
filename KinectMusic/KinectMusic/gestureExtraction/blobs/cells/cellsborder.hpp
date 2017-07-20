//
//  cellsborder.hpp
//  KinectMusic
//
//  Created by Mikhail Svetlov on 19/07/17.
//  Copyright © 2017 mgsvetlov. All rights reserved.
//

#ifndef cellsborder_h
#define cellsborder_h

#include <deque>
#include "cells.hpp"

template<typename T>
using Deque = std::deque<T>;

template<template<typename> class TContainer, typename T> class CellsBorder : public  Cells<TContainer,T> {
    using Cells<TContainer,T>::cells;
    using Cells<TContainer,T>::minValInd;
    using Cells<TContainer,T>::maxValInd;
public:
    CellsBorder();
    CellsBorder(const CellsBorder&) = delete;
    CellsBorder(CellsBorder&&);
    void AddCellFront(const T& cell);
protected:
    void CheckFrontMinMax();
};

template<template<typename> class TContainer, typename T> CellsBorder<TContainer,T>::CellsBorder()
{
}

template<template<typename> class  TContainer, typename T> CellsBorder<TContainer,T>::CellsBorder(CellsBorder&& other)
{
}

template<template<typename> class TContainer, typename T> void CellsBorder<TContainer,T>::AddCellFront(const T& cell){
    cells.push_front(cell);
    CheckFrontMinMax();
}

template<template<typename> class  TContainer, typename T> void CellsBorder<TContainer,T>::CheckFrontMinMax(){
    auto val = this->Value(cells.front(), std::is_pointer<T>());
    if(minValInd == NO_DATA_VALUE || val < this->Value(cells[minValInd], std::is_pointer<T>()))
        minValInd = 0;
    if(maxValInd == NO_DATA_VALUE || val < this->Value(cells[maxValInd], std::is_pointer<T>()))
        maxValInd = 0;
}

#endif /* cellsborder_h */
