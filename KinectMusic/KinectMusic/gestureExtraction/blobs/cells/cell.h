//
//  cell.h
//  KinectMusic
//
//  Created by Mikhail Svetlov on 16/07/17.
//  Copyright © 2017 mgsvetlov. All rights reserved.
//

#ifndef cell_h
#define cell_h

#include "../../types.h"

struct FLAGS {
    static const unsigned char ADJACENT_BODY = 0x01;
};

struct Cell {
    Cell(uint16_t x, uint16_t y, int ind, int val);
    static float Distance(const Cell& cell1, const Cell& cell2);
    static float Distance(const Cell& cell1, int x, int y, int val);
    
    uint16_t x = 0;
    uint16_t y = 0;
    int ind = NO_DATA_VALUE;
    uint16_t val = NO_DATA_VALUE;
};

struct CellContour : public Cell {
    CellContour(const Cell& cell);
    static bool IsNeighbours(const CellContour& cell1, const CellContour& cell2);
    
    uint16_t valOut = NO_DATA_VALUE;
    unsigned char flags = 0x00;
    
};

std::ostream& operator << (std::ostream& os, const CellContour);


#endif /* cell_h */
