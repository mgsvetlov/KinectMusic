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
double Gesture::speedThreshSlow (12e3 * Gesture::spaceCoeff), Gesture::speedThreshFast(16e3 * Gesture::spaceCoeff), Gesture::speedThreshEnd(4e3 * Gesture::spaceCoeff);

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
    HandData& handDataBack = *rit;
    int& phase = handDataBack.phase;
    int& speed = handDataBack.speed;
    int& direction = handDataBack.direction;
    cv::Point3d& keyPointCalc = handDataBack.keyPointCalc;
    size_t N = 4;
    if(N >= size - 1)
        N = size - 1;
    
    rit++;
    int phasePrev = rit->phase;
    if( phasePrev == 0 || phasePrev == 1){
        cv::Vec3d dist = keyPointCalc - rit->keyPointCalc;
        double length = sqrt(static_cast<double>(dist[0]*dist[0] + dist[1]*dist[1] + dist[2]*dist[2]));
        if(length < speedThreshEnd) {
            phase = 2; //конец
            speed = length >  Gesture::speedThreshFast ? 1 : 0;
        }
        else {
            phase = 0; //середина
            speed = length > Gesture::speedThreshFast ? 1 : 0;
        }
        return;
    }
    
    rit = handsTrackedStream.rbegin();
    rit++;
    double lengthMax (0);
    cv::Vec3d distMax (0,0,0);
    for(size_t i = 0; i < N; i++, rit++){
        int phase1 = rit->phase;
        if(phase1 >= 0)
            break;
        if(phase1 == -2)
            continue;
        cv::Vec3d dist = keyPointCalc - rit->keyPointCalc;
        double length = sqrt(static_cast<double>(dist[0]*dist[0] + dist[1]*dist[1] + dist[2]*dist[2])) / (i+1);
        if(length > lengthMax){
            lengthMax = length;
            distMax = dist;
        }
    }
    
    if(lengthMax > Gesture::speedThreshSlow) {
        double dx = std::abs(distMax[0]);
        double dy = std::abs(distMax[1]);
        double dz = std::abs(distMax[2]);
        int ind = (dx > dy && dx > dz)? 0 : dy > dz ? 1 : 2;
        if(ind == 2 && distMax[2] > 0)
            return;
        switch(ind){
            case 0:
                direction = distMax[0] < 0 ? 0 : 1;
                break;
            case 1:
                direction = distMax[1] < 0 ? 2 : 3;
                break;
            default:
                direction = 4;
        }
        phase = 1; //начало
        speed = lengthMax > Gesture::speedThreshFast ? 1 : 0;
       
    }
    
}