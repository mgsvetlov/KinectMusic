//
//  hand.cpp
//  KinectMusic
//
//  Created by Mikhail Svetlov on 26/11/16.
//  Copyright © 2016 mgsvetlov. All rights reserved.
//

#include <stdio.h>
#include "hand.h"
#include <queue>
#include <vector>

Hand::Hand(cv::Mat mat, int bbXY, const cv::Point3i& keyPoint) :
    refPoint(keyPoint.x - bbXY, keyPoint.y - bbXY),
    size (bbXY * 2, bbXY * 2)
{
    int x = keyPoint.x;
    int y = keyPoint.y;
    int z = keyPoint.z;
    uint16_t* p_mat = (uint16_t*)(mat.data);
    for(int i = 0; i < mat.total(); i++, p_mat++)
    {
        int x_ = i % mat.cols;
        int y_ = (i - x_)/mat.cols;
        int z_ = *p_mat;
        if(abs(x - x_)<= bbXY &&
           abs(y - y_)<= bbXY &&
           abs(z - z_)<= MAX_NEIGHB_DIFF_COARSE * 2){
            lPoints.push_back(cv::Point3i(x_ - refPoint.x, y_ - refPoint.y, z_));
        }
    }
}

Hand Hand::extractHand() const{
    std::list<Hand> lHands;
    cv::Point2i center (size.width >> 1, size.height >>1);
    int w = size.width;
    int h = size.height;
    cv::Mat matBlob = cv::Mat_<uint16_t>::zeros(size);
    cv::Mat matBlobOrig = cv::Mat_<uint16_t>::zeros(size);
    uint16_t* const p_matBlob = (uint16_t*)(matBlob.data);
    uint16_t* const p_matBlobOrig = (uint16_t*)(matBlobOrig.data);
    for(auto& point : lPoints) {
        int ind = point.y * w + point.x;
        *(p_matBlobOrig + ind) = *(p_matBlob + ind) = point.z;
    }
    uint16_t* p_matBlob1 = (uint16_t*)(matBlob.data);
    for(int i = 0; i< matBlob.total(); i++, p_matBlob1++) {
        if(*p_matBlob1 == 0)
            continue;
        bool centerInside(false);
        Hand hand;
        int x_ = i % w;
        int y_ = (i-x_) / w;
        hand.lPoints.push_back(cv::Point3i(x_, y_,*p_matBlob1));
        *p_matBlob1 = 0;
        std::queue<int> q;
        q.push(i);
        while(!q.empty()){
            const int ind = q.front();
            q.pop();
            int x = ind % w;
            int y = (ind - x)/ w;
            if(x == center.x && y == center.y)
                centerInside = true;
            for(int yNeighb = y - 1; yNeighb <= y + 1; yNeighb++){
                if(yNeighb < 0 || yNeighb >= h)
                    continue;
                for(int xNeighb = x - 1; xNeighb <= x + 1; xNeighb++){
                    if(xNeighb < 0 || xNeighb >= w)
                        continue;
                    int indNeighb = yNeighb * w + xNeighb;
                    uint16_t valNeighb =  *(p_matBlob + indNeighb);
                    if(valNeighb == 0)
                        continue;
                    uint16_t val = *(p_matBlobOrig + ind);
                    if(std::abs(val - valNeighb) > 10)
                        continue;
                    int x1 = indNeighb % w;
                    int y1 = (indNeighb-x_) / w;
                    hand.lPoints.push_back(cv::Point3i(x1, y1,valNeighb));
                    q.push(indNeighb);
                    *(p_matBlob + indNeighb) = 0;
                }
            }
            
        }
        if(!hand.lPoints.empty() && centerInside){
            hand.size = this->size;
            hand.refPoint = this->refPoint;
            hand.findBorderPoints();
            if(hand.checkIsHand())
                return hand;
            else
                return Hand();
        }
    }
    return Hand();
}

void Hand::findBorderPoints(){
    vvPointsBorder.resize(4);
    int w = size.width;
    int h = size.height;
    for(auto& point : lPoints){
        if(point.x == 0)
            vvPointsBorder[0].push_back(point);
        else if(point.x == w-1)
            vvPointsBorder[1].push_back(point);
        if(point.y == 0)
            vvPointsBorder[2].push_back(point);
        else if(point.y == h-1)
            vvPointsBorder[3].push_back(point);
    }
}

bool Hand::checkIsHand(){
    int sum(0);
    for(auto& v : vvPointsBorder)
        sum += v.size();
    if(sum > (size.width >> 1))
        return false;
    return true;
}