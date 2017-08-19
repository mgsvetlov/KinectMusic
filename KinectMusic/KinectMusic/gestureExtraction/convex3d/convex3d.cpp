//
//  handsheadextractor.cpp
//  KinectMusic
//
//  Created by Mikhail Svetlov on 22/10/16.
//  Copyright © 2016 mgsvetlov. All rights reserved.
//

#include "convex3d.h"
#include "../params.h"

cv::Mat Convex3d::extractConvexities(cv::Mat mat, int filt_size, int filt_depth, int core_half_size)
{
    cv::Mat matDst = cv::Mat_<uint16_t>::zeros(mat.size());
    uint16_t* p_mat = (uint16_t*)(mat.data);
    uint16_t* p_matDst = (uint16_t*)(matDst.data);
    
    for(int y = 0; y < mat.rows; y++){
        for(int x = 0; x < mat.cols; ++x, ++p_mat, ++p_matDst){
            uint16_t val = *p_mat;
            if(!val)
                continue;
            int filt_size_ = sqrt(static_cast<double>(Params::getMaxKinectDepth()) /val) * filt_size;
            bool isConvexity(true);
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
                    if(val_ && val_ - val <= filt_depth * (abs(m)+abs(n)) * 0.5){
                        isConvexity= false;
                        break;
                    }
                     if(!isConvexity)
                         break;
                }
            }
            if(isConvexity)
                *p_matDst = val;
        }
    }
    return matDst;
}

cv::Mat Convex3d::extractConvexities1(cv::Mat mat, std::list<int> inds, int filt_size, int filt_depth, int core_half_size)
{
    cv::Mat matDst = cv::Mat_<uint16_t>::zeros(mat.size());
    uint16_t* p_mat = (uint16_t*)(mat.data);
    uint16_t* p_matDst = (uint16_t*)(matDst.data);
    
    for(auto ind : inds){
        p_mat = (uint16_t*)(mat.data) + ind;
        p_matDst = (uint16_t*)(matDst.data) + ind;
        int x = ind % mat.cols;
        int y = (ind - x) / mat.cols;
            uint16_t val = *p_mat;
        if(!val)
            continue;
        int filt_size_ = sqrt(static_cast<double>(Params::getMaxKinectDepth()) /val) * filt_size;
        int count(0);
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
                if(val_ - val <= filt_depth * (abs(m)+abs(n)) * 0.5){
                    continue;
                }
                ++count;
            }
        }
        if(count >= core_half_size * 8 * 0.875)
            *p_matDst = val;
    }
    return matDst;
}