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

cv::Mat Convex3d::extractConvexitiesFine(cv::Mat mat, int start_dist, int end_dist, int dzThresh, int countGlobalMin, std::list<int>& inds)
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
            int countGlobal(0);
            for(int i = 0; i < neighbours.size(); ++i){
                int dx = neighbours[i].first;
                int dy = neighbours[i].second;
                int dx_ =  dx * start_dist;
                int dy_ =  dy * start_dist;
                int j = start_dist;
                int valPrec = val;
                int countLocal(0);
                for(; j < end_dist; ++j, dx_ += dx, dy_ += dy){
                    int x_ = x + dx_;
                    int y_ = y + dy_;
                    if(x_ < 0 || x_ >= mat.cols || y_ < 0 || y_ >= mat.rows)
                        continue;
                    uint16_t val_ = *((uint16_t*)(mat.data) + y_* mat.cols + x_);
                    if(!val_ || val_ - valPrec  >= dzThresh)
                        ++countLocal;
                    valPrec = val_;
                }
                countGlobal += countLocal;
                if(countGlobal >= countGlobalMin)
                    break;
            }
            if(countGlobal >= countGlobalMin){
                *((uint16_t*)(matDst.data) + ind) =  val;
                ++it;
                continue;
            }
        }
        it = inds.erase(it);
    }
    return matDst;
}

int Convex3d::radius(cv::Mat mat, int ind, int start, int end, int dzThresh){
    if(ind < 0 || ind > mat.total())
       return INT_MAX;
    int x = ind % mat.cols;
    int y = (ind - x) / mat.cols;
    uint16_t val = *((uint16_t*)(mat.data) + ind);
    if(!val)
        return INT_MAX;
    std::vector<int> radiusAll(neighbours.size(), INT_MAX);
    for(int i = 0; i < neighbours.size(); ++i){
        int dx = neighbours[i].first;
        int dy = neighbours[i].second;
        int x_ =  x + dx * start;
        int y_ =  y + dy * start;
        int valPrec = val;
        int j = start;
        for(; j <= end; ++j, x_ += dx, y_ += dy){
            if(x_ < 0 || x_ >= mat.cols || y_ < 0 || y_ >= mat.rows){
                break;
            }
            uint16_t val_ = *((uint16_t*)(mat.data) + y_* mat.cols + x_);
            if(!val_ || val_ - valPrec  >= dzThresh){
                radiusAll[i] = j;
                break;
            }
            valPrec = val_;
        }
    }
    std::vector<int> radiusAll1(neighbours.size() >> 1, INT_MAX);
    for(int i = 0; i < (neighbours.size() >> 1); ++i){
        if(radiusAll[i] ==  INT_MAX ||
           radiusAll[i + (neighbours.size() >> 1)] == INT_MAX)
            continue;
        radiusAll1[i] = radiusAll[i] + radiusAll[i + (neighbours.size() >> 1)];
    }
    
    std::sort(radiusAll1.begin(), radiusAll1.end(), [](int r1, int r2){return r1 < r2;});
    if(radiusAll1[1] == INT_MAX)
        return INT_MAX;
    
    return radiusAll1[1];
}

bool Convex3d::isBorderPoint(cv::Mat mat, int ind, int start, int end, int dzThresh){
    if(ind < 0 || ind > mat.total())
        return false;
    int x = ind % mat.cols;
    int y = (ind - x) / mat.cols;
    uint16_t val = *((uint16_t*)(mat.data) + ind);
    if(!val)
        return false;
    int count(0);
    for(int i = start; i <= end; ++i) {
        for(auto& neighb : neighbours){
            int xNeighb = x + neighb.first * i;
            int yNeighb = y + neighb.second * i;
            if(xNeighb < 0 || xNeighb >= mat.cols || yNeighb < 0 || yNeighb >= mat.rows)
                return false;
            int indNeighb = yNeighb * mat.cols + xNeighb;
            uint16_t valNeighb =  *((uint16_t*)(mat.data) + indNeighb);
            if(valNeighb - val >= dzThresh)
                count++;
        }
    }
    return count >= 2;
}
