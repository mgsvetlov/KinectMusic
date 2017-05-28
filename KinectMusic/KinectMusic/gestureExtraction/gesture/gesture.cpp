//
//  gesture.cpp
//  KinectMusic
//
//  Created by Mikhail Svetlov on 04/12/16.
//  Copyright © 2016 mgsvetlov. All rights reserved.
//

#include "gesture.h"
#include "gesturefabrique.h"
#include "../tracking/tracking.h"
#include "../analyze.h"

size_t Gesture::threshUnrecogn = 2;

void Gesture::addData(const Track& track){
    const std::list<Hand>& lHands = track.getLHands();
    if(lHands.empty()){
        handsData.push_back(HandData(cv::Point3i(NO_DATA_VALUE,NO_DATA_VALUE,NO_DATA_VALUE), NO_DATA_VALUE));
    }
    else {
        const cv::Point3i& point = lHands.back().getKeyPoint();
        handsData.push_back(HandData(point/*GestureFabrique::convertToRealSpace(point)*/, NO_DATA_VALUE));
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
