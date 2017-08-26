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

cv::Mat Convex3d::extractConvexities(cv::Mat mat, int filt_size, int filt_depth, int core_half_size, int count_false_percent, bool isZeroValid, std::list<int>& inds)
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
                    if(!isZeroValid && val_ == 0){
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

cv::Mat Convex3d::extractConvexities1(cv::Mat mat, int filt_size, int filt_depth, int core_half_size, std::list<int>& inds)
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
            int filt_size_ = /*sqrt(static_cast<double>(Params::getMaxKinectDepth()) /val)* */filt_size;
            int count(0);
            for(size_t j = 0; j  < (neighbours.size() >> 1); j +=2){
                std::vector<size_t> neighbInds { j, j + (neighbours.size() >> 1) };
                for(int i = 1; i <= core_half_size; i++) {
                    count = 0;
                    for( auto neighbInd : neighbInds ){
                        int x_ = x + neighbours[neighbInd].first * i * filt_size_;
                        int y_ = y + neighbours[neighbInd].second * i * filt_size_;
                        if(x_ < 0 || x_ >= mat.cols || y_ < 0 || y_ >= mat.rows)
                            break;
                        uint16_t val_ = *((uint16_t*)(mat.data) + y_* mat.cols + x_);
                        if(val_ && val_ - val <= filt_depth * sqrt(abs(i)+abs( i)) * 0.5){
                           
                            break;
                        }
                        ++count;
                    }
                    if(count == 2)
                        break;
                }
                if(count == 2)
                    break;
            }
            if(count == 2){
                *((uint16_t*)(matDst.data) + ind) =  val;
                ++it;
                continue;
            }
        }
        it = inds.erase(it);
    }
    return matDst;
}
