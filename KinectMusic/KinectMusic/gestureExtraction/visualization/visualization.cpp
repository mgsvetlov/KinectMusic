//
//  visualization.cpp
//  KinectMusic
//
//  Created by Mikhail Svetlov on 01/10/16.
//  Copyright © 2016 mgsvetlov. All rights reserved.
//
#include <ctime>
#include <stdio.h>
#include "visualization.h"
#include "../blobs/blob.h"
#include "../tracking/tracking.h"
#include "../analyze.h"
#include "../hand/hand.h"
#include "../gesture/gesture.h"

Visualization* Visualization::p_vis = nullptr;
cv::Mat Visualization::matImage;
bool Visualization::isNeedRedraw = false;

Visualization::Visualization() {
    cv::namedWindow( "Display window", cv::WINDOW_AUTOSIZE );
    cv::moveWindow("Display window", 320, 10);
    //cv::resizeWindow("Display window", 640, 480);
    p_vis = this;
}

bool Visualization::showImage() {
    pthread_mutex_lock(&visualisation_mutex);
    isNeedRedraw = false;
    if(p_vis == nullptr){
        Visualization();
    }
    cv::flip(matImage, matImage, 1);
    //cv::Mat matImageRes;
    //cv::resize(matImage, matImageRes, cv::Size(matImage.cols >> BLOBS_RESIZE_POW, matImage.rows >> BLOBS_RESIZE_POW));
    cv::imshow( "Display window", matImage);
    pthread_mutex_unlock(&visualisation_mutex);
    
    if(cv::waitKey(1) == 27) {
        return false;
    }

#ifdef USE_CSOUND
    if(!CSOUND_START)
        CSOUND_START = true;
#endif //USE_CSOUND
    
    /*static int frameCount(0);
    static std::clock_t t = clock();
    if(frameNum % 30 == 0){
        std::clock_t next_t = clock();
        if(frameCount) {
            double elapsed_sec = double(next_t - t)/CLOCKS_PER_SEC;
            std::cout << "output fps " << 30./elapsed_sec  << std::endl;
        }
        t = next_t;
    }
    ++frameCount;*/
    return true;
}


void Visualization::mat2img(cv::Mat mat, cv::Mat& matImg) {
    
    int w = mat.cols;
    int h = mat.rows;
    cv::Mat r  = cv::Mat_<unsigned char>::zeros(cv::Size(w, h));
    cv::Mat g  = cv::Mat_<unsigned char>::zeros(cv::Size(w, h));
    cv::Mat b  = cv::Mat_<unsigned char>::zeros(cv::Size(w, h));
    uint16_t* p_mat16 = (uint16_t*)(mat.data);
    unsigned char* p_r = (unsigned char*)(r.data);
    unsigned char* p_g = (unsigned char*)(g.data);
    unsigned char* p_b = (unsigned char*)(b.data);
    for(int i = 0; i < w*h; i++) {
        uint16_t d16 = *p_mat16;
        
        if(d16 && d16 < MAX_KINECT_VALUE) {
            *p_b = *p_g =*p_r = 255 - d16 * 255. / MAX_KINECT_VALUE;
            //*p_b = 0;
        }
        /*else {
            *p_r = 0;
            *p_b = 255;
        }*/
        p_r++, p_g++, p_b++, p_mat16++;
    }
    
    std::vector<cv::Mat> channels;
    channels.push_back(b);
    channels.push_back(g);
    channels.push_back(r);
    
    cv::merge(channels, matImg);
}

void Visualization::hands2img(const std::list<Hand>& lHands, cv::Mat& matImg, bool drawKeyPoints){
    int count(0);
    for(auto& hand : lHands) {
        cv::Scalar color = count == 0? cv::Scalar(0,1,0) : cv::Scalar(0,1,1);
        hand2img(hand, matImg, color);
        keyPoint2img(hand.keyPoint, matImg, cv::Scalar(127, 127, 127), 10);
        count++;
    }
}

void Visualization::tracks2img(const std::vector<Track>& vTracks, cv::Mat& matImg, bool drawKeyPoints){
    int count(0);
    for(auto& track : vTracks) {
        std::list<Hand> lHands = track.getLHands();
        if(lHands.empty()) {
            count++;
            continue;
        }
        Hand& hand = lHands.back();
        cv::Scalar color = count == 0? cv::Scalar(0,0.5,0) : cv::Scalar(0.5,0,0);
        hand2img(hand, matImg, color);
        if(drawKeyPoints)
            keyPoint2img(hand.keyPoint, matImg, cv::Scalar(127, 127, 127), 10);
        count++;
    }
}

void Visualization::handsTrackedStreams2img(const std::vector<std::vector<HandData>>& handsTrackedStreams, cv::Mat& matImg, size_t length){
    int pointSize(5);
    int count(-1);
    std::vector<cv::Point3i> keyPoints;
    std::vector<int> directions;
    std::vector<cv::Scalar> colors;
    for(auto& handsTrackedStream : handsTrackedStreams) {
        count++;
        if(handsTrackedStream.empty())
            continue;
        size_t length1 = length;
        if(handsTrackedStream.size() < length1)
            length1 = handsTrackedStream.size();
        auto rit = handsTrackedStream.rbegin();
        
        cv::Point3i startKeyPoint(-1,-1,-1);
        int direction(-1);
        cv::Vec3d moveFromStartVec = rit->moveFromStartVec;
        double dx = std::abs(moveFromStartVec[0]);
        double dy = std::abs(moveFromStartVec[1]);
        double dz = std::abs(moveFromStartVec[2]);
        int ind = (dx > dy && dx > dz)? 0 : dy > dz ? 1 : 2;
        if(ind == 2 && moveFromStartVec[2] > 0)
            return;
        switch(ind){
            case 0:
                direction = moveFromStartVec[0] < 0 ? 0 : 1;
                break;
            case 1:
                direction = moveFromStartVec[1] < 0 ? 2 : 3;
                break;
            default:
                direction = 4;
        }
        
        cv::Scalar color1 = cv::Scalar(0,0,255);
        if(rit->phase >= 0 || rit->phase == -100){
            if(rit->phase >= 0)
                moveFromStartVec /= rit->phase + 1;
            else {
                auto ritPrev = rit + 1 ;
                moveFromStartVec /= ritPrev->phase + 2;
            }
            double length = sqrt(static_cast<double>(moveFromStartVec[0] * moveFromStartVec[0] + moveFromStartVec[1] * moveFromStartVec[1] + moveFromStartVec[2] * moveFromStartVec[2]));
            color1 = length >  Gesture::speedThreshFast ? cv::Scalar(0,0,255) : cv::Scalar(204,204,0);
        }
        
        for(int i = 0; i <= length1; i++, rit++){
            const auto& phase = rit->phase;
            const auto& speed = rit->speed;
            if((phase < 0 && phase != -100) || speed < 0)
                break;
            const auto& keyPoint = rit->keyPoint;
            startKeyPoint = keyPoint;
            int pointSize1 = phase == 0 ?  pointSize * 2 : pointSize;
            keyPoint2img(keyPoint, matImg, color1, pointSize1);
            if(phase == 0)
                break;
        }
        if(startKeyPoint.x != -1) {
            keyPoints.push_back(startKeyPoint);
            cv::Scalar color = count == 0? cv::Scalar(0,127,0) : cv::Scalar(127,0,0);
            colors.push_back(color);
            directions.push_back(direction);
        };
    }
    
    if(keyPoints.empty())
        return;
    
    cv::flip(matImg, matImg, 1);
    for(int i = 0; i < keyPoints.size(); i++){
        double x = 1.0 - static_cast<double>(keyPoints[i].x) / matImg.cols;
        auto& direction = directions[i];
        std::string text = direction == 0 ? "Right" :
        direction == 1 ? "Left" :
        direction == 2 ? "Up" :
        direction == 3 ? "Down" : "Forward";
        drawText(matImg, text, 1.5, 3, colors[i], cv::Point2f(x, 0.3));
    }
    cv::flip(matImg, matImg, 1);
}

void Visualization::hand2img(const Hand& hand, cv::Mat& matImg, const cv::Scalar& color){
    for(auto& point : hand.lPoints){
        unsigned int col = 255 - point.z * 255. / MAX_KINECT_VALUE;
        cv::Scalar colorPoint = color;
        for(int i = 0; i < 3; i++)
            colorPoint[i] *= col;
        int x = point.x + hand.refPoint.x;
        int y = point.y + hand.refPoint.y;
        cv::circle(matImg, cv::Point(x, y), 1,  colorPoint, -1);
    }
}

void Visualization::keyPoint2img(const cv::Point3i& keyPoint, cv::Mat& matImg, const cv::Scalar& color, int size) {
    int x = keyPoint.x;
    int y = keyPoint.y;
    if(x < 0 || y < 0)
        return;
    cv::circle(matImg, cv::Point(x, y), size,  color, -1);
}

void Visualization::drawText(cv::Mat& mat, std::string text, double fontScale, int thickness, cv::Scalar color, cv::Point2f textCenter)
{
    int fontFace = cv::FONT_HERSHEY_SIMPLEX;
    
    int baseline = 0;
    cv::Size textSize = cv::getTextSize(text, fontFace,
                                fontScale, thickness, &baseline);
    baseline += thickness;
    
    // center the text
    cv::Point textOrg((mat.cols - textSize.width) * textCenter.x,
                  (mat.rows + textSize.height) * textCenter.y);
    
    cv::putText(mat, text, textOrg, fontFace, fontScale,
            color, thickness, 8);
}
