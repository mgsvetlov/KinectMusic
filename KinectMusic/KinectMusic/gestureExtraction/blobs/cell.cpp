//
//  cell.cpp
//  KinectMusic
//
//  Created by Mikhail Svetlov on 16/07/17.
//  Copyright © 2017 mgsvetlov. All rights reserved.
//

#include "cell.hpp"

Cell::Cell(){}

Cell::Cell(uint16_t x, uint16_t y, int ind, int val):
x(x),
y(y),
ind(ind),
val(val){}

Cell::Cell(uint16_t x, uint16_t y, int ind, int val, const Cell& cell):
x(x),
y(y),
ind(ind),
val(val),
dist(cell.dist + distance(cell)){}

float Cell::distance(const Cell& cell){
    static constexpr float spaceCoeff(9./6400);
    auto& cellVal = cell.val;
    float dx = (x * val - cell.x * cellVal) * spaceCoeff;
    float dy = (y * val - cell.y * cellVal) * spaceCoeff;
    int dz = val - cellVal;
    return sqrt(dx*dx + dy*dy + dz*dz);
}
