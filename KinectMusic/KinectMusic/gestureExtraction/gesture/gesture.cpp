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

void Gesture::addData(const Track& track){
    const std::list<Hand>& lHands = track.getLHands();
    if(lHands.empty()){
        handsData.push_back(HandData(cv::Point3i(-1,-1,-1), -2));
    }
    else {
        const cv::Point3i& point = lHands.back().getKeyPoint();
        handsData.push_back(HandData(GestureFabrique::convertToRealSpace(point), -1));
    }
}

void Gesture::analyze(){
    size_t size = handsData.size();
    if(size < 2)
        return;
    
    auto rit = handsData.rbegin();
    int& phase = handsData.rbegin()->phase;
    const cv::Point3d& point = rit->point;
    
    auto ritPrev = handsData.rbegin();
    ritPrev++;
    int phasePrev = ritPrev->phase;
    
    if(point.x == -1){
        if( phasePrev >= 0 ){
            phase = -100; //конец
            log();
            eraseHandsData(1); //erased all but last
        }
        else {
            eraseHandsData(1); //erased all but last
            return;
        }
        return;
    }
    
    cv::Vec3d moveVec = point - ritPrev->point;
    double length = sqrt(static_cast<double>(moveVec[0] * moveVec[0] + moveVec[1] * moveVec[1] + moveVec[2] * moveVec[2]));
    
    if( phasePrev >= 0 ){
        if(length < GestureFabrique::speedThreshEnd) {
            phase = -100; //конец
            log();
            eraseHandsData(1); //erased all but last
        }
        else {
            phase = phasePrev + 1; //середина
        }
        return;
    }
    
    rit++;
    double lengthMax (0);
    cv::Vec3d distMax (0,0,0);
    size_t N = 4;
    if(N >= size - 1)
        N = size - 1;
    int iMax (-1);
    for(size_t i = 0; i < N; i++, rit++){
        int phase1 = rit->phase;
        if(phase1 == -2)
            continue;
        cv::Vec3d dist = point - rit->point;
        double length1 = sqrt(static_cast<double>(dist[0]*dist[0] + dist[1]*dist[1] + dist[2]*dist[2])) / (i+1);
        if(length1 > lengthMax){
            lengthMax = length1;
            distMax = dist;
            iMax = static_cast<int>(i);
        }
    }
    if(iMax == -1){
        eraseHandsData(1); //erased all but last
        return;
    }
        
    if(lengthMax > GestureFabrique::speedThreshSlow ) {
        phase = 0; //начало
        eraseHandsData(2); //erased all but 2 lasts
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
    bool isGesture(false);
    for( auto& handData : handsData){
        if(handData.phase >= 0){
            isGesture = true;
            break;
        }
    }
    if(!isGesture)
        return;
    
    std::ofstream& logFile = GestureFabrique::gesturesLog;
    logFile << "hand\t" << handInd << "\n";
    bool startFound(false);
    for( auto& handData : handsData){
        if(handData.phase == 0)
            startFound = true;
        else if(handData.phase == -100 && !startFound)
            continue;
        logFile << handData.phase << "\t" << handData.point << "\n";
    }
        
    logFile << std::endl;
}