//
//  handsheadextractor.cpp
//  KinectMusic
//
//  Created by Mikhail Svetlov on 22/10/16.
//  Copyright © 2016 mgsvetlov. All rights reserved.
//

#include "handsheadextractor.h"

HandsHeadExtractor::HandsHeadExtractor(cv::Mat mat, int filt_size, int filt_depth, int iterCount) :
mat (mat),
filt_size(filt_size),
filt_depth(filt_depth),
iterCount(iterCount)
{}

cv::Mat HandsHeadExtractor::extractHandsHead()
{
    cv::Mat matDst = cv::Mat_<uint16_t>::zeros(mat.size());
    uint16_t* p_mat = (uint16_t*)(mat.data);
    uint16_t* p_matDst = (uint16_t*)(matDst.data);
    
    for(int y = 0; y < mat.rows; y++){
        for(int x = 0; x < mat.cols; ++x, ++p_mat, ++p_matDst){
            uint16_t val = *p_mat;
            if(!val)
                continue;
            int filt_size_ = sqrt(static_cast<double>(MAX_KINECT_DEPTH) /val) * filt_size;
            bool isValue(true);
            for(int m = -iterCount; m <= iterCount; m++){
                for(int n = -iterCount; n <= iterCount; n++){
                    if(!m && !n)
                        continue;
                    int y_ = y + m * filt_size_;
                    int x_ = x + n * filt_size_;
                    uint16_t val_ = *((uint16_t*)(mat.data) + y_* mat.cols + x_);
                    if(val_ && val_ - val <= filt_depth * iterCount){
                        isValue = false;
                        break;
                    }
                     if(!isValue)
                         break;
                }
            }
            if(isValue)
                *p_matDst = val;
        }
    }
    return matDst;
}