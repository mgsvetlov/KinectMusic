//
//  handsheadextractor.cpp
//  KinectMusic
//
//  Created by Mikhail Svetlov on 22/10/16.
//  Copyright © 2016 mgsvetlov. All rights reserved.
//

#include "handsextractor.h"

cv::Mat HandsExtractor::extractHands(cv::Mat mat, int filt_size, int filt_depth, int core_half_size)
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
            bool isFeature(true);
            for(int m = -core_half_size; m <= core_half_size; m++){
                int y_ = y + m * filt_size_;
                if(y_ < 0)
                    continue;
                if(y_ >= mat.rows)
                    break;
                for(int n = -core_half_size; n <= core_half_size; n++){
                    if(!m && !n)
                        continue;
                    int x_ = x + n * filt_size_;
                    if(x_ < 0)
                        continue;
                    if(x_ >= mat.cols)
                        break;
                    uint16_t val_ = *((uint16_t*)(mat.data) + y_* mat.cols + x_);
                    if(val_ && val_ - val <= filt_depth * (m+n) * 0.5){
                        isFeature= false;
                        break;
                    }
                     if(!isFeature)
                         break;
                }
            }
            if(isFeature)
                *p_matDst = val;
        }
    }
    return matDst;
}