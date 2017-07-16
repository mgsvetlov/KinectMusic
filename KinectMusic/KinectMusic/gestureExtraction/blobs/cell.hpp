//
//  cell.h
//  KinectMusic
//
//  Created by Mikhail Svetlov on 16/07/17.
//  Copyright © 2017 mgsvetlov. All rights reserved.
//

#ifndef cell_hpp
#define cell_hpp

#include "../types.h"

class SubBlob;

struct Cell {
    Cell();
    Cell(uint16_t x, uint16_t y, int ind, int val);
    Cell(uint16_t x, uint16_t y, int ind, int val, const Cell& cell);
    uint16_t x = 0;
    uint16_t y = 0;
    int ind = NO_DATA_VALUE;
    int val = NO_DATA_VALUE;
    float dist = 0;
    Cell* parent = nullptr;
    Cell* child = nullptr;
    SubBlob* subBlob = nullptr;
#ifdef USE_CELL_NORMAL
    cv::Vec4f normal = cv::Vec4f(0.f, 0.f, 0.f);
#endif //USE_CELL_NORMAL
private:
    float distance(const Cell& cell);
};


#endif /* cell_hpp */
