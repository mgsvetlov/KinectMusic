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

cv::Mat Hand::matHand, Hand::matHandOrig;

Hand::Hand(cv::Mat mat, int bbXY, const cv::Point3i& keyPoint) :
    keyPoint(keyPoint),
    mat(mat),
    refPoint(keyPoint.x - bbXY, keyPoint.y - bbXY),
    size (bbXY * 2, bbXY * 2)
{
    if(matHand.empty())
        matHand = cv::Mat_<uint16_t>::zeros(size);
    if(matHandOrig.empty())
        matHandOrig = cv::Mat_<uint16_t>::zeros(size);
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
    matHand.setTo(cv::Scalar(0));
    matHandOrig.setTo(cv::Scalar(0));
    for(auto& point : lPoints) {
        matHand.at<uint16_t> (point.y, point.x )=
        matHandOrig.at<uint16_t> (point.y, point.x ) = point.z;
    }

    uint16_t* p_matHand1 = (uint16_t*)(matHand.data);
    for(int i = 0; i< matHand.total(); i++, p_matHand1++) {
        if(*p_matHand1 == 0)
            continue;
        bool centerInside(false);
        Hand hand;
        int x_ = i % w;
        int y_ = (i-x_) / w;
        hand.lPoints.push_back(cv::Point3i(x_, y_,*p_matHand1));
        *p_matHand1 = 0;
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
                    uint16_t valNeighb =  matHand.at<uint16_t> (yNeighb, xNeighb );
                    if(valNeighb == 0)
                        continue;
                    uint16_t val = matHandOrig.at<uint16_t> (yNeighb, xNeighb );
                    if(std::abs(val - valNeighb) > 10)
                        continue;
                    hand.lPoints.push_back(cv::Point3i(xNeighb, yNeighb,valNeighb));
                    q.push(indNeighb);
                    matHand.at<uint16_t> (yNeighb, xNeighb ) = 0;
                }
            }
            
        }
        if(!hand.lPoints.empty() && centerInside){
            hand.size = this->size;
            hand.refPoint = this->refPoint;
            hand.mat = this->mat;
            hand.findBorderPoints();
            if(hand.checkIsHand()){
                hand.keyPoint = this->keyPoint;
                return hand;
            }
            else {
                return Hand();
            }
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
    int w = size.width;
    int h = size.height;
    for(auto& v : vvPointsBorder)
        sum += v.size();
    if(sum > (w >> 1))
        return false;
    if(lPoints.size() < ((w * h ) >> 2))
        return true;
    int minx(w), maxx(0), miny(h), maxy(0);
    for(auto& point : lPoints){
        if(point.x < minx)
            minx = point.x;
        if(point.x > maxx)
            maxx = point.x;
        if(point.y < miny)
            miny = point.y;
        if(point.y > maxy)
            maxy = point.y;
    }
    int s = (maxx - minx) * (maxy - miny);
    if(s > (lPoints.size() << 1))
        return true;
    return false;
}

double Hand::dist2hand(const Hand& hand) const{
    int dx = keyPoint.x - hand.keyPoint.x;
    int dy = keyPoint.y - hand.keyPoint.y;
    return sqrt(dx*dx+dy*dy);
}