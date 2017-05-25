//
//  GestureReturn.cpp
//  KinectMusic
//
//  Created by Mikhail Svetlov on 25/05/17.
//  Copyright © 2017 mgsvetlov. All rights reserved.
//

#include "gestureReturn.h"
#include "gesturefabrique.h"
#include "../tracking/tracking.h"
#include "../analyze.h"

void GestureReturn::extract(){
    size_t size = handsData.size();
    if(size < 2)
        return;
    
    auto rit = handsData.rbegin();
    int& phase = handsData.rbegin()->phase;
    const cv::Point3d& point = rit->point;
    
    auto ritPrev = handsData.rbegin();
    ritPrev++;
    int& phasePrev = ritPrev->phase;

    static int countUnrecogn(0);
    if(point.x != -1)
        countUnrecogn = 0;
    else
        countUnrecogn++;
    
    if( phasePrev >= 0 ){
        if(countUnrecogn > 2 || (handsData.size() > 2 && isThreshPassed && isInThreshold(point, 0))){
                phase = -1; //конец
                log();
                eraseHandsData(1); //erased all but last
            }
            else {
                if(!isThreshPassed && isOutThreshold(point, 1))
                   isThreshPassed = true;
                phase = frameNum; //середина
            }

    }
    else if(point.x != -1 && point.z - ritPrev->point.z < -10 ) {
            phasePrev = frameNum - 1;
            phase = frameNum; //начало
            startPoint = point;
            isThreshPassed = false;
            eraseHandsData(2); //erased all but 2 lasts
    }
    return;
}

bool GestureReturn::isOutThreshold (const cv::Point3d& point, int ind) const {
    cv::Vec3d v = point - startPoint;
    for(int i = 0; i < 3; i ++){
        if(std::abs(v[i]) < thresholds[ind][i])
            return false;
    }
    return true;
}

bool GestureReturn::isInThreshold (const cv::Point3d& point, int ind) const {
    cv::Vec3d v = point - startPoint;
    for(int i = 0; i < 3; i ++){
        if(std::abs(v[i]) > thresholds[ind][i])
            return false;
    }
    return true;
}