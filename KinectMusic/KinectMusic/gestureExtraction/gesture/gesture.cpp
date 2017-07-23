//
//  gesture.cpp
//  KinectMusic
//
//  Created by Mikhail Svetlov on 04/12/16.
//  Copyright © 2016 mgsvetlov. All rights reserved.
//
/*
#include "gesture.h"
#include "gesturefabrique.h"
#include "../tracking/tracking.h"

size_t Gesture::threshUnrecogn = 2;

void Gesture::addData(const Track& track){
    const std::list<Hand>& lHands = track.getLHandsConst();
    if(lHands.empty()){
        handsData.push_back(HandData(cv::Point3i(NO_DATA_VALUE,NO_DATA_VALUE,NO_DATA_VALUE), NO_DATA_VALUE, NO_DATA_VALUE));
    }
    else {
        const cv::Point3i& point = lHands.back().getKeyPoint();
        int angle = lHands.back().getAngle();
        handsData.push_back(HandData(point, NO_DATA_VALUE, angle));
    }
}


void Gesture::eraseHandsData(int nonErasedAtEndCount){
    if(handsData.empty())
        return;
    auto it = handsData.end();
    for(int i = 0; i < nonErasedAtEndCount; i++)
        --it;
    handsData.erase(handsData.begin(), it);
}
*/