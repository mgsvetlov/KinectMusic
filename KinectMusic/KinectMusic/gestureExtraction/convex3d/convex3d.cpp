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
            static const int filt_size2 = 3;
            static const int dzThresh = 40;
            static const int countMin = 6;
            int count(0);
            for(int i = 0; i < neighbours.size(); ++i){
                int dx = neighbours[i].first;
                int dy = neighbours[i].second;
                int dx_ =  dx;
                int dy_ =  dy;
                int j = 0;
                int valPrec = val;
                for(; j < filt_size2; ++j, dx_ += dx, dy_ += dy){
                    int x_ = x + dx_;
                    int y_ = y + dy_;
                    if(x_ < 0 || x_ >= mat.cols || y_ < 0 || y_ >= mat.rows)
                        continue;
                    uint16_t val_ = *((uint16_t*)(mat.data) + y_* mat.cols + x_);
                    if(!val_ || val_ - valPrec  >= dzThresh)
                        break;
                    valPrec = val_;
                }
                if(j == filt_size2){
                    if(i + 1 - count > neighbours.size() - countMin)
                        break;
                    continue;
                }
                ++count;
                if(count == countMin)
                    break;
            }
            if(count == countMin){
                *((uint16_t*)(matDst.data) + ind) =  val;
                ++it;
                continue;
            }
        }
        it = inds.erase(it);
    }
    return matDst;
}
