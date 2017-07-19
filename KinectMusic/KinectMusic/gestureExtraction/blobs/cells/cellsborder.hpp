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
    
};

template<template<typename> class TContainer, typename T> CellsBorder<TContainer,T>::CellsBorder()
{
}

template<template<typename> class  TContainer, typename T> CellsBorder<TContainer,T>::CellsBorder(CellsBorder&& other)
{
}

#endif /* cellsborder_h */
