//
//  nearestblob.hpp
//  KinectMusic
//
//  Created by Mikhail Svetlov on 01/10/16.
//  Copyright © 2016 mgsvetlov. All rights reserved.
//

#ifndef blob_h
#define blob_h

#include <stdio.h>
#include "../types.h"

struct Cell {
    Cell(){}
    Cell(int ind, int val): ind(ind), val(val){}
    int ind;
    int val = NO_DATA_VALUE;
};

class Blob {
public:
    Blob();
private:
    Blob(cv::Mat mat16, int x, int y);
public:
    static int findBlobs(cv::Mat mat16, std::list<Blob>& lvBlobs, int mode = 0);
    std::list<Cell>& getLCells() {return lCells;}
    const std::list<Cell>& getLCellsConst() const {return lCells;}
    static cv::Mat blobs2mat(const std::list<Blob>& lBlobs, const cv::Size& size);
      static cv::Mat blobs2mat_marked(const std::list<Blob>& lBlobs, const cv::Size& size, int body_depth);
    
    static bool blobsClustering(std::list<Blob>& lBlobs, std::list<Blob>& lBlobsClustered, int xyThresh, int depthThresh);
    
    static void sort(std::list<Blob>& lBlobs);
    static void filterNearBody(std::list<Blob>& lBlobs, int bodyDepth, int minDistToBody);
    
    const Cell& getCentralCell() const {return centralCell;}
    const cv::Size& getMatSize() const {return this->matSize;}
    void setMatSize(cv::Size size) {this->matSize = size;}
    void computeAngle();

private:
    void addCell(int ind, int val);
    void mergeBlob(const Blob& blob);
    bool computeCentralCell();
    int computeAverageValue();
    bool computeCentralNearCell(double med);
    bool isBlobNear(const Blob& blob, const int xyThresh, const int depthThresh);

private:
    const Cell* p_maxValCell = nullptr;
    const Cell* p_minValCell = nullptr;
    std::list<Cell> lCells;
    Cell centralCell;
    float angle;
    cv::Size matSize;
    
    friend class Hand;
    friend class Visualization;
};
#endif /* blob_h */
