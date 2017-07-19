
//
//  blobext.h
//  KinectMusic
//
//  Created by Mikhail Svetlov on 18/07/17.
//  Copyright © 2017 mgsvetlov. All rights reserved.
//

#ifndef blobext_hpp
#define blobext_hpp

#include "blob.hpp"

template<typename T> class BlobExt : public Blob<T> {
    using Blob<T>::cells;
public:
    BlobExt();
    BlobExt(cv::Mat mat, int ind, int blobInd, bool connectivity, float distThresh, int sizeThresh = NO_DATA_VALUE);
    
    const Cells<CellBorder>& getBorderCells1Const() const {return borderCells1;}
    const Cells<CellBorder>& getBorderCells2Const() const {return borderCells2;}
    
    bool analyzeHand(cv::Mat originalMat);
private:
    void computeAngle();
    Cells<CellBorder> borderCells1;
    Cells<CellBorder> borderCells2;
    int angle;
    
    friend class Visualization;
};

template<typename T> BlobExt<T>::BlobExt()
{}

//blob extended
template<typename T> BlobExt<T>::BlobExt(cv::Mat mat, int ind, int blobInd, bool connectivity, float distThresh, int sizeThresh){
    cells.All().reserve(sizeThresh);
    const uint16_t maskValue = 65535 - blobInd;
    int w =  mat.cols;
    int h = mat.rows;
    uint16_t* p_mat = (uint16_t*)(mat.data);
    int x = ind % w;
    int y = (ind - x)/ w;
    cells.AddCell(x, y, ind, *(p_mat + ind));
    *(p_mat + ind) = maskValue;
    bool isEndModeGlobal (false);
    auto it = cells.All().begin();
    for(;it != cells.All().end();++it){
        bool isEndMode (false);
        if(isEndModeGlobal || (sizeThresh != NO_DATA_VALUE && cells.Size() >= sizeThresh)
           || (connectivity && it->dist > distThresh )) {
            isEndMode = true;
        }
        const uint16_t val =  it->val;
        int x_ = it->x;
        int y_ = it->y;
        for(int yNeighb = y_ - 1; yNeighb <= y_ + 1; yNeighb++){
            if(yNeighb < 0 || yNeighb >= h)
                continue;
            for(int xNeighb = x_-1; xNeighb <= x_ + 1; xNeighb++){
                if((yNeighb == y_ && xNeighb == x_) || xNeighb < 0 || xNeighb >= w)
                    continue;
                int indNeighb = yNeighb * w + xNeighb;
                uint16_t valNeighb =  *(p_mat + indNeighb);
                if(valNeighb >= MAX_KINECT_DEPTH  || valNeighb == 0 || abs(valNeighb-val) >= MAX_NEIGHB_DIFF_COARSE){
                    if(valNeighb > maskValue){
                        cells.Clear();
                        borderCells1.Clear();
                        borderCells2.Clear();
                        return;
                    }
                    if(valNeighb != maskValue){
                        *(p_mat + indNeighb) = maskValue;
                        borderCells1.AddCell(xNeighb, yNeighb,indNeighb, valNeighb);
                        borderCells1.All().back().parentInd = static_cast<int>(it - cells.All().begin());
                    }
                    continue;
                }
                *(p_mat + indNeighb) = maskValue;
                if(isEndMode){
                    borderCells2.AddCell(xNeighb, yNeighb,indNeighb, valNeighb);
                    borderCells2.All().back().parentInd = static_cast<int>(it - cells.All().begin());
                    continue;
                }
                if(!isEndMode && !isEndModeGlobal){
                    if(connectivity){
                        cells.AddCell(xNeighb, yNeighb,indNeighb, valNeighb, *it);
                        it->child = &cells.All().back();
                        cells.All().back().parent = &(*it);
                    }
                    else {
                        cells.AddCell(xNeighb, yNeighb, indNeighb, valNeighb);
                    }
                    if(cells.Size() == sizeThresh)
                        isEndModeGlobal = true;
                }
            }
        }
    }
}

template<typename T> bool BlobExt<T>::analyzeHand(cv::Mat originalMat){
    
    //PclNormals::estimateNormals(*this);
    //computeAngle();
    return true;
}

template<typename T> void BlobExt<T>::computeAngle(){
    /*if( cells.Size() < 3){
     this->angle = 0.0f;
     return;
     }
     float x(0.0), y(0.0), z(0.0), w(0.0);
     PclPlane::fitPlane(*this, x, y, z, w);
     float angle = std::abs(z);
     float norm = sqrt(y*y + z*z);
     if( norm != 0.0f)
     angle /= norm;
     if(z * y < 0)
     angle *= -1;
     this->angle = angle * 100.f;*/
}

using BlobFinal = BlobExt<CellExt>;

#endif /* blobext_hpp */
