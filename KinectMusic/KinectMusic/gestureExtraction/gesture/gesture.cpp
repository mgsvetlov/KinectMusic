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
double Gesture::spaceCoeff(9./6400);
double Gesture::speedThreshSlow (8e3 * Gesture::spaceCoeff), Gesture::speedThreshFast(16e3 * Gesture::spaceCoeff), Gesture::speedThreshEnd(2e3 * Gesture::spaceCoeff);

void Gesture::analyzeGestures(const std::vector<Track>& tracks){
    Gesture::addDataToStream(tracks);
    for(auto& handsTrackedStream : handsTrackedStreams) {
        analyzeGesture(handsTrackedStream);
    }
}

void Gesture::addDataToStream(const std::vector<Track>& tracks){
    for(int i = 0; i < 2; i++){
        auto& track = tracks[i];
        auto& handsTrackedStream = handsTrackedStreams[i];
        const std::list<Hand>& lHands = track.getLHands();
        if(lHands.empty()){
            handsTrackedStream.push_back(HandData(cv::Point3i(-1,-1,-1),cv::Point3d(-1,-1,-1), -2));
        }
        else {
            const cv::Point3i& keyPoint = lHands.back().getKeyPoint();
            double z = keyPoint.z;
            double x = keyPoint.x * z * Gesture::spaceCoeff;
            double y = keyPoint.y * z * Gesture::spaceCoeff;
            handsTrackedStream.push_back(HandData(keyPoint, cv::Point3d(x,y,z), -1));
        }
        if(handsTrackedStream.size() > 300)
            handsTrackedStream.erase(handsTrackedStream.begin());
    }
}

void Gesture::analyzeGesture(std::vector<HandData>& handsTrackedStream){
    size_t size = handsTrackedStream.size();
    if(size < 2)
        return;
    
    auto rit = handsTrackedStream.rbegin();
    int& phase = rit->phase;
    int& speed = rit->speed;
    cv::Vec3d& moveFromStartVec = rit->moveFromStartVec;
    cv::Point3d& keyPointCalc = rit->keyPointCalc;
    
    rit++;
    int phasePrev = rit->phase;
    
    cv::Vec3d moveVec = keyPointCalc - rit->keyPointCalc;
    double length = sqrt(static_cast<double>(moveVec[0] * moveVec[0] + moveVec[1] * moveVec[1] + moveVec[2] * moveVec[2]));
    
    if( phasePrev >= 0 ){
       auto ritStart = handsTrackedStream.rbegin();
        while(ritStart->phase != 0)
            ritStart++;
        
        moveFromStartVec = keyPointCalc - rit->keyPointCalc;
        moveFromStartVec += rit->moveFromStartVec;
        if(length < speedThreshEnd) {
            phase = -100; //конец
            speed = length >  Gesture::speedThreshFast ? 1 : 0;
        }
        else {
            phase = phasePrev + 1; //середина
            speed = length > Gesture::speedThreshFast ? 1 : 0;
        }
        return;
    }
    
    rit = handsTrackedStream.rbegin();
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
        cv::Vec3d dist = keyPointCalc - rit->keyPointCalc;
        double length1 = sqrt(static_cast<double>(dist[0]*dist[0] + dist[1]*dist[1] + dist[2]*dist[2])) / (i+1);
        if(length1 > lengthMax){
            lengthMax = length1;
            distMax = dist;
            iMax = static_cast<int>(i);
        }
    }
    if(iMax == -1)
        return;
    
    if(lengthMax > Gesture::speedThreshSlow ) {
        phase = 0; //начало
        speed = lengthMax > Gesture::speedThreshFast ? 1 : 0;
        moveVec = moveFromStartVec = distMax / (iMax + 1);
    }
    
}