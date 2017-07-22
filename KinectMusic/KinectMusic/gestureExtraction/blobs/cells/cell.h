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

struct Cell {
    Cell(uint16_t x, uint16_t y, int ind, int val);
    static   float Distance(const Cell& cell1, const Cell& cell2);
    
    uint16_t x = 0;
    uint16_t y = 0;
    int ind = NO_DATA_VALUE;
    uint16_t val = NO_DATA_VALUE;
};



#endif /* cell_h */
