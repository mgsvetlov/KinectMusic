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
    p_vis = this;
}

bool Visualization::showImage() {
    pthread_mutex_lock(&visualisation_mutex);
    isNeedRedraw = false;
    pthread_mutex_unlock(&visualisation_mutex);
    
    if(p_vis == nullptr){
        Visualization();
    }
    cv::flip(matImage, matImage, 1);
    cv::Mat matImageRes;
    cv::resize(matImage, matImageRes, cv::Size(matImage.cols >> BLOBS_RESIZE_POW, matImage.rows >> BLOBS_RESIZE_POW));
    cv::imshow( "Display window", matImageRes);
    if(cv::waitKey(1) == 27) {
        return false;
    }

#ifdef USE_CSOUND
    if(!CSOUND_START)
        CSOUND_START = true;
#endif //USE_CSOUND
    
    static int frameCount(0);
    static std::clock_t t = clock();
    if(frameNum % 30 == 0){
        std::clock_t next_t = clock();
        if(frameCount) {
            double elapsed_sec = double(next_t - t)/CLOCKS_PER_SEC;
            std::cout << "output fps " << 30./elapsed_sec  << std::endl;
        }
        t = next_t;
    }
    ++frameCount;
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
            *p_r = 255 - d16 * 255. / MAX_KINECT_VALUE;
            *p_b = 0;
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

void Visualization::tracks2img(const std::vector<Tracking>& vTracks, cv::Mat& matImg, bool drawKeyPoints){
    int count(0);
    for(auto& track : vTracks) {
        std::list<Hand> lHands = track.getLHands();
        if(lHands.empty()) {
            count++;
            continue;
        }
        Hand& hand = lHands.back();
        cv::Scalar color = count == 0? cv::Scalar(0,1,0) : cv::Scalar(0,1,1);
        hand2img(hand, matImg, color);
        if(drawKeyPoints)
            keyPoint2img(hand.keyPoint, matImg, cv::Scalar(127, 127, 127), 10);
        count++;
    }
}

void Visualization::handsTrackedStreams2img(const std::vector<std::vector<HandData>>& handsTrackedStreams, cv::Mat& matImg, size_t length){
    int pointSize(10);
    int count(0);
    for(auto& handsTrackedStream : handsTrackedStreams) {
        cv::Scalar color = count == 1? cv::Scalar(127,255,127) : cv::Scalar(127,255,255);
        size_t length1 = length;
        if(handsTrackedStream.size() < length1)
            length1 = handsTrackedStream.size();
        auto rit = handsTrackedStream.rbegin();
        for(int i = 0; i <= length1; i++, rit++){
            const auto& type = rit->type;
            if(type< 0)
                continue;
            const auto& keyPoint =rit->keyPoint;
            cv::Scalar color1 = type == 1 ? cv::Scalar(255, 0, 0) : color;
            int pointSize1 = type == 0 ? pointSize : pointSize * 2;
            keyPoint2img(keyPoint, matImg, color1, pointSize1);
            if(type == 1)
                break;
        }
        count++;
    }
    
}

void Visualization::hand2img(const Hand& hand, cv::Mat& matImg, const cv::Scalar& color){
    for(auto& point : hand.lPoints){
        unsigned int col = 255 - point.z * 255. / MAX_KINECT_VALUE;
        cv::Scalar colorPoint = color;
        for(int i = 0; i < 3; i++)
            colorPoint[i] *= col;
        int x = point.x + hand.refPoint.x;
        int y = point.y + hand.refPoint.y;
        circle(matImg, cv::Point(x, y), 1,  colorPoint, -1);
    }
}

void Visualization::keyPoint2img(const cv::Point3i& keyPoint, cv::Mat& matImg, const cv::Scalar& color, int size) {
    int x = keyPoint.x;
    int y = keyPoint.y;
    if(x < 0 || y < 0)
        return;
    circle(matImg, cv::Point(x, y), size,  color, -1);
}


