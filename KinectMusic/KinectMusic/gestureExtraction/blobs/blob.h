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

struct Border {
    std::list<Cell*> borderCells = std::list<Cell*>();
    int level = -1;
    /*Border* parent = nullptr;
    std::list<Border*> children = std::list<Border*>();*/
};

class SubBlob {
private:
    std::vector<Cell*> vpCells;
    friend class Blob;
    friend class Visualization;
};

class Blob {
public:
    Blob();
private:
    Blob(cv::Mat mat16, int x, int y);
public:
    static int findBlobs(cv::Mat mat16, std::list<Blob>& lvBlobs, int mode = 0);
    std::list<Cell>& getCells() {return cells.All();}
    const std::list<Cell>& getCellsConst() const {return cells.AllConst();}
    static cv::Mat blobs2mat(const std::list<Blob>& lBlobs, const cv::Size& size);
    static bool blobsClustering(std::list<Blob>& lBlobs, std::list<Blob>& lBlobsClustered, int xyThresh, int depthThresh);
    float distance (int ind1, int val1, int ind2, int val2);
    
    const Cell& getCentralCell() const {return centralCell;}
    void setCentralCell(const Cell cc) {centralCell = cc;}
    const cv::Size& getMatSize() const {return this->matSize;}
    void setMatSize(cv::Size size) {this->matSize = size;}
    
    bool filterLargeBlobs(cv::Mat originalMat);
    bool analyzeHand(cv::Mat originalMat);
    
private:
    bool computeCentralCell();
    bool isBlobNear(const Blob& blob, const int xyThresh, const int depthThresh);
    void createCellsTree(cv::Mat mat, int ind, int val, bool connectivity, float distThresh = std::numeric_limits<float>::max());
    void createSubBlobs();
    void createBorders();
    cv::Mat blob2mat();
    void computeAngle();
    
private:
    Cells<Cell> cells;
    std::list<SubBlob> subBlobs;
    std::list<Border> borders;
    Cell centralCell;
    int angle;
    cv::Size matSize;
    
    friend class Hand;
    friend class Visualization;
    friend std::ostream& operator << (std::ostream& os, const Blob& blob);
};

#endif /* blob_h */
