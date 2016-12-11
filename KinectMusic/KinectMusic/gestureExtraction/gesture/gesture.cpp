//
//  gesture.cpp
//  KinectMusic
//
//  Created by Mikhail Svetlov on 04/12/16.
//  Copyright © 2016 mgsvetlov. All rights reserved.
//

#include "gesture.h"
#include "../tracking/tracking.h"

std::vector<std::vector<HandData>> Gesture::handsTrackedStreams (2);

void Gesture::addDataToStream(const std::vector<Tracking>& tracks){
    for(int i = 0; i < 2; i++){
        auto& track = tracks[i];
        auto& handsTrackedStream = handsTrackedStreams[i];
        const std::list<Hand>& lHands = track.getLHands();
        if(lHands.empty()){
            handsTrackedStream.push_back(HandData(cv::Point3i(-1,-1,-1), -2));
        }
        else {
            const cv::Point3i& keyPoint = lHands.back().getKeyPoint();
            handsTrackedStream.push_back(HandData(keyPoint, -1));
        }
        if(handsTrackedStream.size() > 300)
            handsTrackedStream.erase(handsTrackedStream.begin());
    }
}

void Gesture::analyzeGestures(){
    for(auto& handsTrackedStream : handsTrackedStreams) {
        analyzeGesture(handsTrackedStream);
    }
}

void Gesture::analyzeGesture(std::vector<HandData>& handsTrackedStream){
    if(handsTrackedStream.empty())
        return;
    HandData& handDataBack = handsTrackedStream.back();
    int& type = handDataBack.type;
    size_t N = 4;
    size_t size = handsTrackedStream.size();
    if(N >= size - 1)
        N = size - 1;
    if(type == -2) {
        int& type1 = handsTrackedStream[size - 1 - N].type;
        if( type1 == 0 && type1 == 1 )
            type1 = 2; //конец
        return;
    }
    auto rit = handsTrackedStream.rbegin();
    rit++;
    for(size_t i = 0; i < N; i++, rit++){
        int type1 = rit->type;
        if(type1 == -2)
            continue;
        cv::Vec3i dist = handDataBack.keyPoint - rit->keyPoint;
        double length = static_cast<double>(dist[0]*dist[0] + dist[1]*dist[1]) / (i+1);
        if((type1 == -1 || type1 == 2) && length > 400){
            type = 1; //начало
            break;
        }
        else if( type1 == 0 || type1 == 1){
            if(length < 10) {
                type = 2; //конец
            }
            else {
                type = 0; //середина
            }
            break;
        }
    }
}