//
//  cell.cpp
//  KinectMusic
//
//  Created by Mikhail Svetlov on 16/07/17.
//  Copyright © 2017 mgsvetlov. All rights reserved.
//

#include "cell.h"

Cell::Cell(uint16_t x, uint16_t y, int ind, int val):
x(x),
y(y),
ind(ind),
val(val){}

CellExt::CellExt(uint16_t x, uint16_t y, int ind, int val):
Cell(x, y, ind, val)
{}

CellExt::CellExt(uint16_t x, uint16_t y, int ind, int val, const CellExt& cell):
Cell(x, y, ind, val),
dist(cell.dist + distance(cell)){}

float CellExt::distance(const CellExt& cell){
    static constexpr float spaceCoeff(9./6400);
    auto& cellVal = cell.val;
    float dx = (x * val - cell.x * cellVal) * spaceCoeff;
    float dy = (y * val - cell.y * cellVal) * spaceCoeff;
    int dz = val - cellVal;
    return sqrt(dx*dx + dy*dy + dz*dz);
}

CellBorder::CellBorder(uint16_t x, uint16_t y, int ind, int val):
Cell(x, y, ind, val)
{}