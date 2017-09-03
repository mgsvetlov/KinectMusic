//
//  handsheadextractor.cpp
//  KinectMusic
//
//  Created by Mikhail Svetlov on 22/10/16.
//  Copyright © 2016 mgsvetlov. All rights reserved.
//
#include <numeric>
#include "convex3d.h"
#include "../params.h"

std::vector<std::pair<int,int>> Convex3d::neighbours {
    {-1, 0}, {-1, 1}, {0, 1}, {1, 1},
    {1, 0}, {1, -1},  {0, -1},  {-1, -1}
};

cv::Mat Convex3d::extractConvexities(cv::Mat mat, int filt_size, int filt_depth, int core_half_size, int count_false_percent, bool isZeroValid, std::list<int>& inds, bool extremumsOnly)
{
    if(inds.empty()){
        inds.resize(mat.total());
        std::iota(inds.begin(), inds.end(), 0);
    }
    cv::Mat matDst = cv::Mat_<uint16_t>::zeros(mat.size());
    size_t countFalseMax(neighbours.size() * core_half_size  * count_false_percent * 1e-2);
    auto it = inds.begin();
    while(it != inds.end()){
        auto ind = *it;
        int x = ind % mat.cols;
        int y = (ind - x) / mat.cols;
        uint16_t val = *((uint16_t*)(mat.data) + ind);
        if(val){
            int filt_size_ = sqrt(static_cast<double>(Params::getMaxKinectDepth()) /val)* filt_size;
            size_t countFalse(0);
            for(auto& neighb : neighbours){
                for(int i = 1; i <= core_half_size; i++) {
                    int x_ = x + neighb.first * i * filt_size_;
                    int y_ = y + neighb.second * i * filt_size_;
                    if(x_ < 0 || x_ >= mat.cols || y_ < 0 || y_ >= mat.rows)
                        continue;
                    uint16_t val_ = *((uint16_t*)(mat.data) + y_* mat.cols + x_);
                    if((!isZeroValid && val_ == 0) || (extremumsOnly && val_ && val_ < val)){
                        countFalse = countFalseMax;
                        break;
                    }
                    if(val_ && val_ - val <= filt_depth * sqrt(abs(i)+abs( i)) * 0.5){
                        ++countFalse;
                        if(countFalse >=  countFalseMax)
                            break;
                    }
                }
                if(countFalse >=  countFalseMax)
                    break;
            }
            if(countFalse < countFalseMax){
                *((uint16_t*)(matDst.data) + ind) =  val;
                ++it;
                continue;
            }
        }
        it = inds.erase(it);
    }
    return matDst;
}

cv::Mat Convex3d::extractConvexities1(cv::Mat mat, int filt_size, int filt_depth, int core_half_size, int count_false_percent, bool isZeroValid, std::list<int>& inds, bool extremumsOnly)
{
    if(inds.empty()){
        inds.resize(mat.total());
        std::iota(inds.begin(), inds.end(), 0);
    }
    cv::Mat matDst = cv::Mat_<uint16_t>::zeros(mat.size());
    auto it = inds.begin();
    while(it != inds.end()){
        auto ind = *it;
        int x = ind % mat.cols;
        int y = (ind - x) / mat.cols;
        uint16_t val = *((uint16_t*)(mat.data) + ind);
        if(val){
            bool isLocalMin(true);
            for(auto& neighb : neighbours){
                int x_ = x + neighb.first;
                int y_ = y + neighb.second;
                if(x_ < 0 || x_ >= mat.cols || y_ < 0 || y_ >= mat.rows)
                    continue;
                uint16_t val_ = *((uint16_t*)(mat.data) + y_* mat.cols + x_);
                if(val_ && val_ < val){
                    isLocalMin = false;
                    break;
                }
            }
            if(isLocalMin) {
                static const int filt_size2 = 4;
                static const int dzThresh = 20;
                static const int countMin = 1;
                int count(0);
                for(int i = 0; i < (neighbours.size() >> 1); ++i){
                    int dx = neighbours[i].first;
                    int dy = neighbours[i].second;
                    bool isFound(true);
                    for(int k = -1; k <= 1; k += 2){
                        int dx1 = k * dx;
                        int dy1 = k * dy;
                        int dx_ =  dx1;
                        int dy_ =  dy1;
                        int j = 1;
                        for(; j <= filt_size2; ++j, dx_ += dx1, dy_ += dy1){
                            int x_ = x + dx_;
                            int y_ = y + dy_;
                            if(x_ < 0 || x_ >= mat.cols || y_ < 0 || y_ >= mat.rows)
                                continue;
                            uint16_t val_ = *((uint16_t*)(mat.data) + y_* mat.cols + x_);
                            if(!val_ || val_ - val >= dzThresh)
                                break;
                        }
                        if( j == filt_size2 + 1){
                            isFound = false;
                            break;
                        }
                    }
                    if(isFound){
                        ++count;
                        if(count == countMin)
                            break;
                    }
                }
                if(count == countMin){
                    *((uint16_t*)(matDst.data) + ind) =  val;
                    ++it;
                    continue;
                }
            }
        }
        it = inds.erase(it);
    }
    return matDst;
}
