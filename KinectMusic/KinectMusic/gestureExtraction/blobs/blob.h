//
//  nearestblob.hpp
//  KinectMusic
//
//  Created by Mikhail Svetlov on 01/10/16.
//  Copyright © 2016 mgsvetlov. All rights reserved.
//

#ifndef blob_h
#define blob_h

#include <limits>
#include "cells.hpp"

/*struct Border {
    std::list<Cell*> borderCells = std::list<Cell*>();
    int level = -1;
};

class SubBlob {
private:
    std::vector<Cell*> vpCells;
    friend class Blob;
    friend class Visualization;
};*/

class Blob {
public:
    Blob();
    
private:
    Blob(cv::Mat mat, int ind);
    Blob(cv::Mat mat, int ind, int blobInd, bool connectivity, float distThresh, int sizeThresh = NO_DATA_VALUE);
    
public:
    Cells<Cell>& getCells() {return cells;}
    const Cells<Cell>& getCellsConst() const {return cells;}
    static cv::Mat blobs2mat(const std::list<Blob>& lBlobs, const cv::Size& size);
    
    const cv::Size& getMatSize() const {return this->matSize;}
    void setMatSize(cv::Size size) {this->matSize = size;}
    
    int indOriginNearest(cv::Mat originalMat) const;
    bool analyzeHand(cv::Mat originalMat);
    
private:
    //void createSubBlobs();
    //void createBorders();
    cv::Mat blob2mat();
    void computeAngle();
    
private:
    Cells<Cell> cells;
    Cells<Cell> border1;
    Cells<Cell> border2;
    //std::list<SubBlob> subBlobs;
    //std::list<Border> borders;
    int angle;
    cv::Size matSize;
    
    friend class BlobsFabrique;
    friend class BlobsClust;
    friend class Hand;
    friend class Visualization;
};

#endif /* blob_h */
