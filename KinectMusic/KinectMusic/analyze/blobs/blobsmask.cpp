//
//  blobsmask.cpp
//  KinectMusic
//
//  Created by Mikhail Svetlov on 09/10/16.
//  Copyright © 2016 mgsvetlov. All rights reserved.
//
#include <queue>
#include "blobsmask.h"

BlobsMask::BlobsMask(const std::list<Blob>& lBlobs, cv::Size size, cv::Size fullSize) :
size(size),
fullSize(fullSize),
w(fullSize.width),
h(fullSize.height)
{
    mask  = cv::Mat_<uint16_t>::zeros(size);
    for(auto& blob : lBlobs) {
        auto& lCells = blob.getLCellsConst();
        for(auto& cell : lCells) {
            int ind = cell.ind;
            *((uint16_t*)(mask.data) + ind) = cell.val;
        }
    }
    cv::resize(mask, mask, fullSize);
}
cv::Mat BlobsMask::applyMask(cv::Mat mat) {
    cv::Mat mat_result  = cv::Mat_<uint16_t>::zeros(fullSize);
    uint16_t* const p_mat_result = (uint16_t*)(mat_result.data);
    uint16_t* p_mask = (uint16_t*)(mask.data);
    uint16_t* p_mat16 = (uint16_t*)(mat.data);
    const uint16_t* const p_mat16_start = p_mat16;
    for(int i = 0; i < w * h; i++, p_mat16++, p_mask++){
        if(!*p_mask
           ||!*p_mat16
           || *p_mask >= MAX_KINECT_VALUE
           || *p_mat16 >= MAX_KINECT_VALUE
           || *p_mat16 > *p_mask)
            continue;
        std::queue<int> q;
        q.push(i);
        *(p_mat_result + i) = *(p_mat16_start + i);
        bool addFlag(true);
        while(!q.empty()){
            int i1 = q.front();
            q.pop();
            if(q.size() > 100)
                addFlag = false;
            if(!addFlag)
                continue;
            
            for(int y = -w; y <= w; y += w ){
                for(int x = -1; x <= 1; x++ ){
                    if(!y && !x)
                        continue;
                    int ind = i1 + y + x;
                    if(ind  < 0 || ind >= w*h)
                        continue;
                    if(*(p_mat_result +ind)!=0)
                        continue;
                    uint16_t neighb  = *(p_mat16_start + ind);
                    if(neighb >= MAX_KINECT_VALUE)
                        continue;
                    if(abs(neighb - *(p_mat16_start + i1)) <=MAX_NEIGHB_DIFF_FINE){
                        *(p_mat_result + ind) = *(p_mat16_start + ind);
                        q.push(ind);
                    }
                }
            }
        }
    }
    return mat_result;
}