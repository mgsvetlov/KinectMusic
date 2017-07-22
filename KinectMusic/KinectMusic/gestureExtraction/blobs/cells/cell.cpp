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


float Cell::Distance(const Cell& cell1, const Cell& cell2){
    static constexpr float spaceCoeff(9./6400);
    float dx = (cell1.x * cell1.val - cell2.x * cell2.val) * spaceCoeff;
    float dy = (cell1.y * cell1.val - cell2.y * cell2.val) * spaceCoeff;
    int dz = cell1.val - cell2.val;
    return sqrt(dx*dx + dy*dy + dz*dz);
}

