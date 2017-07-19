//
//  border.hpp
//  KinectMusic
//
//  Created by Mikhail Svetlov on 19/07/17.
//  Copyright © 2017 mgsvetlov. All rights reserved.
//

#ifndef border_h
#define border_h

#include "../cells/cellsborder.hpp"

template<typename T>
using List = std::list<T>;

template<template<typename> class TContainer, template<typename> class TContainer1,typename T> class Border {
public:
    CellsBorder<TContainer, CellBorder>& getBorderCells();
    const CellsBorder<TContainer, CellBorder>& getBorderCellsConst() const;
    List<CellsBorder<TContainer1, CellBorder*>>& getBorders();
    
private:
    CellsBorder<TContainer, CellBorder> borderCells;
    List<CellsBorder<TContainer1, CellBorder*>> borders;
};

template<template<typename> class TContainer, template<typename> class TContainer1,typename T> CellsBorder<TContainer, CellBorder>&  Border<TContainer,TContainer1,T>::getBorderCells()
{
    return borderCells;
}

template<template<typename> class TContainer, template<typename> class TContainer1,typename T> const CellsBorder<TContainer, CellBorder>&  Border<TContainer,TContainer1,T>::getBorderCellsConst() const
{
    return borderCells;
}

template<template<typename> class TContainer, template<typename> class TContainer1,typename T> List<CellsBorder<TContainer1, CellBorder*>>&  Border<TContainer,TContainer1,T>::getBorders()
{
    return borders;
}
#endif /* border_h */
