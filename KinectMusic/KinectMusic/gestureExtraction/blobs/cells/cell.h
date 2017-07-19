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
    
    uint16_t x = 0;
    uint16_t y = 0;
    int ind = NO_DATA_VALUE;
    int val = NO_DATA_VALUE;
};


struct CellExt : public Cell {
    CellExt(uint16_t x, uint16_t y, int ind, int val);
    CellExt(uint16_t x, uint16_t y, int ind, int val, const CellExt& cell);
private:
    float distance(const CellExt& cell);
    
public:
    float dist = 0;
    Cell* parent = nullptr;
    Cell* child = nullptr;
#ifdef USE_CELL_NORMAL
    cv::Vec4f normal = cv::Vec4f(0.f, 0.f, 0.f);
#endif //USE_CELL_NORMAL

};

struct CellBorder : public Cell {
    CellBorder(uint16_t x, uint16_t y, int ind, int val);
    
    int parentInd = -1;
};
#endif /* cell_h */
