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

void Gesture::addData(const Track& track){
    const std::list<Hand>& lHands = track.getLHands();
    if(lHands.empty()){
        handsData.push_back(HandData(cv::Point3i(-1,-1,-1), -1));
    }
    else {
        const cv::Point3i& point = lHands.back().getKeyPoint();
        handsData.push_back(HandData(GestureFabrique::convertToRealSpace(point), -1));
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

void Gesture::log(){
    std::ofstream& logFile = GestureFabrique::gesturesLog;
    logFile << "hand\t" << handInd << "\n";
    for( int i = 0; i < handsData.size(); ++i){
        if(handsData[i].phase >=0)
        logFile << handsData[i].phase
        << "\t" << handsData[i].point << "\n";
    }
        
    logFile << std::endl;
}