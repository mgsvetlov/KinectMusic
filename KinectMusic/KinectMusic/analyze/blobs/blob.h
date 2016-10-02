//
//  nearestblob.hpp
//  KinectMusic
//
//  Created by Mikhail Svetlov on 01/10/16.
//  Copyright © 2016 mgsvetlov. All rights reserved.
//

#ifndef nearestblob_h
#define nearestblob_h

#include <stdio.h>
#include "../types.h"

struct Cell {
    Cell(){}
    Cell(int ind, int val): ind(ind), val(val){}
    int ind;
    int val = -1;
};

class Blob {
public:
    Blob(){}
private:
    Blob(cv::Mat mat16, int x, int y, int thresh);
public:
    static void findBlobs(cv::Mat mat16, std::list<Blob>& lvBlobs);
    std::list<Cell>& getLCells() {return lCells;}
    const std::list<Cell>& getLCellsConst() const {return lCells;}
    const Cell* getP_maxValCell() {return p_maxValCell;}
    const Cell* getP_minValCell() {return p_minValCell;}
private:
    void addCell(int ind, int val);
private:
    std::list<Cell> lCells;
    const Cell* p_maxValCell = nullptr;
    const Cell* p_minValCell = nullptr;
};
#endif /* nearestblob_hpp */
