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
    Blob(cv::Mat mat16, int x, int y);
public:
    static void findBlobs(cv::Mat mat16, std::list<Blob>& lvBlobs, int mode = 0);
    std::list<Cell>& getLCells() {return lCells;}
    const Cell* getP_minValCell() {return p_minValCell;}
    static cv::Mat blobs2mat(const std::list<Blob>& lBlobs, const cv::Size& size);
    const std::list<Cell>& getLCellsConst() const {return lCells;}
    const Cell* getP_maxValCell() {return p_maxValCell;}
    
    static bool blobsClustering(std::list<Blob>& lBlobs, std::list<Blob>& lBlobsClustered, int xyThresh, int depthThresh);
    
    const Cell& getCentralCell() const {return centralCell;}
    const cv::Size& getMatSize() const {return this->matSize;}
    
    static void extendBlobs(cv::Mat mat16, std::list<Blob>& lBlobs);

private:
    void addCell(int ind, int val);
    void mergeBlob(const Blob& blob);
    bool computeCentralCell();
    bool isBlobNear(const Blob& blob, const int xyThresh, const int depthThresh);
    void extend(cv::Mat mat16, cv::Mat matMap);
private:
    const Cell* p_maxValCell = nullptr;
    const Cell* p_minValCell = nullptr;
    std::list<Cell> lCells;
    Cell centralCell;
    cv::Size matSize;
};
#endif /* nearestblob_hpp */
