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

/*Cell::Cell(uint16_t x, uint16_t y, int ind, int val, Cell* cellPrec, int dist):
x(x),
y(y),
ind(ind),
val(val),
cellPrec(cellPrec),
dist(dist)
{}*/


float Cell::Distance(const Cell& cell1, const Cell& cell2){
    static constexpr float spaceCoeff(9./6400);
    float dx = (cell1.x * cell1.val - cell2.x * cell2.val) * spaceCoeff;
    float dy = (cell1.y * cell1.val - cell2.y * cell2.val) * spaceCoeff;
    int dz = cell1.val - cell2.val;
    return sqrt(dx*dx + dy*dy + dz*dz);
}

float Cell::Distance(const Cell& cell1, int x, int y, int val){
    static constexpr float spaceCoeff(9./6400);
    float dx = (cell1.x * cell1.val - x * val) * spaceCoeff;
    float dy = (cell1.y * cell1.val - y * val) * spaceCoeff;
    int dz = cell1.val - val;
    return sqrt(dx*dx + dy*dy + dz*dz);
}

CellContour::CellContour(const Cell& cell) :
Cell(cell)
{
    
}

bool CellContour::IsNeighbours(const CellContour& cell1, const CellContour& cell2){
    int dx = abs(cell1.x - cell2.x);
    if( dx > 1)
        return false;
    int dy = abs(cell1.y - cell2.y);
    if( dy > 1)
        return false;
    return dx + dy > 0;
}

std::ostream& operator << (std::ostream& os, const CellContour c){
    os << "<" << c.x << "," << c.y << ">";
    return os;
}