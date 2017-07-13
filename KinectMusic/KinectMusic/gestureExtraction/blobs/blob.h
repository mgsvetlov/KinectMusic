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
#include "../types.h"

class SubBlob;

struct Cell {
    Cell(){}
    Cell(int ind, int val): ind(ind), val(val){}
    Cell(int ind, int val, float dist): ind(ind), val(val), dist(dist){}
    int ind;
    int val = NO_DATA_VALUE;
    float dist = 0;
    Cell* parent = nullptr;
    Cell* child = nullptr;
    SubBlob* subBlob = nullptr;
    cv::Vec4f normal = cv::Vec4f(0.f, 0.f, 0.f);
};

struct Border {
    std::list<Cell*> borderCells = std::list<Cell*>();
    Border* parent = nullptr;
    std::list<Border*> childs = std::list<Border*>();
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
    std::list<Cell>& getLCells() {return lCells;}
    const std::list<Cell>& getLCellsConst() const {return lCells;}
    static cv::Mat blobs2mat(const std::list<Blob>& lBlobs, const cv::Size& size);
    static bool blobsClustering(std::list<Blob>& lBlobs, std::list<Blob>& lBlobsClustered, int xyThresh, int depthThresh);
    float distance (int ind1, int val1, int ind2, int val2);
    
    const Cell& getCentralCell() const {return centralCell;}
    void setCentralCell(const Cell cc) {centralCell = cc;}
    const cv::Size& getMatSize() const {return this->matSize;}
    void setMatSize(cv::Size size) {this->matSize = size;}
    const std::vector<SubBlob>& getSubBlobs() const {return subBlobs;}
    
    bool filterLargeBlobs(cv::Mat originalMat);
    bool analyzeHand(cv::Mat originalMat);
    
private:
    void addCell(int ind, int val);
    void mergeBlob(const Blob& blob);
    bool computeCentralCell();
    int computeAverageValue();
    bool isBlobNear(const Blob& blob, const int xyThresh, const int depthThresh);
    void createCellsTree(cv::Mat mat, int ind, int val, bool connectivity, float distThresh = std::numeric_limits<float>::max());
    cv::Mat blob2mat();
    void computeAngle();
    
private:
    const Cell* p_maxValCell = nullptr;
    const Cell* p_minValCell = nullptr;
    std::list<Cell> lCells;
    std::vector<SubBlob> subBlobs;
    Cell centralCell;
    const Cell* rootCell = nullptr;
    int angle;
    cv::Size matSize;
    
    friend class Hand;
    friend class Visualization;
    friend std::ostream& operator << (std::ostream& os, const Blob& blob);
};

#endif /* blob_h */
