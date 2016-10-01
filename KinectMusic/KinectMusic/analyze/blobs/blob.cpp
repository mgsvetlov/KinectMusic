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

Blob::Blob(cv::Mat mat16, std::list<std::vector<int>>& lvBlobs, cv::Mat matBlobsMap, int x, int y, int thresh, int blobNum){
    lvBlobs.push_back(std::vector<int>());
    int w = mat16.cols;
    int h = mat16.rows;
    uint16_t* p_mat16 = (uint16_t*)(mat16.data);
    uint32_t* p_matBlobsMap = (uint32_t*)(matBlobsMap.data);
    std::queue<int> q;
    int indStart = y*w + x;
    *(p_matBlobsMap + indStart) = blobNum;
    lvBlobs.back().push_back(indStart);
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
                if(*(p_matBlobsMap + indNeighb) != 0)
                    continue;
                uint16_t valNeighb =  *(p_mat16 + indNeighb);
                if(valNeighb >= MAX_KINECT_VALUE || valNeighb == 0)
                    continue;
                //static float ratio = static_cast<float>(valNeighb)/val;
                //static float coeff(1.03);
                //static float coeff_rev(1./coeff);
                if(abs(valNeighb-val) < thresh/*ratio < coeff && ratio > coeff_rev*/){
                    *(p_matBlobsMap + indNeighb) = blobNum;
                    lvBlobs.back().push_back(indNeighb);
                    q.push(indNeighb);
                }
            }
        }
    }
    if(lvBlobs.back().size() < BLOB_MIN_SIZE)
        lvBlobs.pop_back();
}

cv::Mat Blob::findBlobs(cv::Mat mat16, std::list<std::vector<int>>& lvBlobs){
    cv::Mat mat16_clone = mat16.clone();
    cv::Mat matBlobsMap = cv::Mat_<uint32_t>::zeros(mat16_clone.size());
    int num(1);
    while(true){
        double minVal;
        int minIdx[mat16.dims];
        cv::minMaxIdx(mat16_clone,  &minVal, NULL, minIdx, NULL);
        //std::cout << minVal << "\t";
        //std::cout << minIdx[1] << "\t" << minIdx[0] << std::endl;
        if(minVal >= MAX_KINECT_VALUE)
            break;
        
        Blob nearestBlob(mat16_clone, lvBlobs, matBlobsMap, minIdx[1], minIdx[0], 5, num++);
    }
    std::cout << "blobsCount " << num -1 << " large " << lvBlobs.size() << std::endl;
    return matBlobsMap;
}