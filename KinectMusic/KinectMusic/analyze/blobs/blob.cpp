//
//  nearestblob.cpp
//  KinectMusic
//
//  Created by Mikhail Svetlov on 01/10/16.
//  Copyright © 2016 mgsvetlov. All rights reserved.
//
#include <iostream>
#include "blob.h"
#include <queue>

Blob::Blob(cv::Mat mat16, int x, int y) :
lCells(std::list<Cell>()),
p_maxValCell(nullptr),
p_minValCell(nullptr)
{
    int w = mat16.cols;
    int h = mat16.rows;
    uint16_t* p_mat16 = (uint16_t*)(mat16.data);
    std::queue<int> q;
    int indStart = y*w + x;
    addCell(indStart, *(p_mat16 + indStart));
    p_minValCell = &lCells.front();
    q.push(indStart);
    while(!q.empty()){
        const int ind = q.front();
        uint16_t val =  *(p_mat16 + ind);
        *(p_mat16 + ind) = MAX_KINECT_VALUE;
        q.pop();
        int x_ = ind % w;
        int y_ = (ind - x_)/ w;
        
        for(int yNeighb = y_ - 1; yNeighb <= y_ + 1; yNeighb++){
            if(yNeighb < 0 || yNeighb >= h)
                continue;
            for(int xNeighb = x_-1; xNeighb <= x_ + 1; xNeighb++){
                if(xNeighb < 0 || xNeighb >= w)
                    continue;
                int indNeighb = yNeighb * w + xNeighb;
                uint16_t valNeighb =  *(p_mat16 + indNeighb);
                if(valNeighb >= MAX_KINECT_VALUE || valNeighb == 0)
                    continue;
                if(abs(valNeighb-val) < MAX_NEIGHB_DIFF_COARSE){
                    addCell(indNeighb, *(p_mat16 + indNeighb));
                    q.push(indNeighb);
                }
            }
        }
    }
}

void Blob::addCell(int ind, int val){
    lCells.push_back(Cell(ind, val));
    if(!p_maxValCell || p_maxValCell->val < val)
        p_maxValCell = &lCells.back();
}

void Blob::findBlobs(cv::Mat mat16, std::list<Blob>& lBlobs){
    cv::Mat mat16_clone = mat16.clone();
    int largeBlobMaxVal(-1), blobsMinVal(-1);
    while(true){
        double minVal = MAX_KINECT_VALUE;
        int minIdx[mat16.dims];
        uint16_t* p_mat16_clone = (uint16_t*)(mat16_clone.data);
        for(int y = 0; y < mat16_clone.rows; y++)
        for(int x = 0 ; x < mat16_clone.cols; x++){
            uint16_t val = *p_mat16_clone++;
            if(val && val < minVal){
                minVal = val;
                minIdx[1] = x;
                minIdx[0] = y;
            }
        }
        
        //cv::minMaxIdx(mat16_clone,  &minVal, NULL, minIdx, NULL);
        if(minVal >= MAX_KINECT_VALUE)
            break;
        
        if(blobsMinVal == -1)
            blobsMinVal = minVal;
        
        Blob nearestBlob(mat16_clone, minIdx[1], minIdx[0]);
        
        if(nearestBlob.getLCells().size() < BLOB_MIN_SIZE)
            continue;
        
        if(largeBlobMaxVal != -1 && nearestBlob.getP_minValCell()->val > /*blobsMinVal*0.7+*/largeBlobMaxVal/* *0.3 */){
            break;
        }
        
        if(nearestBlob.getLCells().size() > BLOB_MIN_SIZE_LAST){
            largeBlobMaxVal = nearestBlob.getP_maxValCell()->val;
            lBlobs.push_front(nearestBlob);
            std::cout << "largest size  " << nearestBlob.getLCells().size() << std::endl;
        }
        else {
            lBlobs.push_back(nearestBlob);
        }
        
    }
    
}