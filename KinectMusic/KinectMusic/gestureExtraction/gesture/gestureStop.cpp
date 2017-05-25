//
//  gestureStop.cpp
//  KinectMusic
//
//  Created by Mikhail Svetlov on 25/05/17.
//  Copyright © 2017 mgsvetlov. All rights reserved.
//

#include "gestureStop.h"
#include "gesturefabrique.h"
#include "../tracking/tracking.h"
#include "../analyze.h"

void GestureStop::extract(){
    size_t size = handsData.size();
    if(size < 2)
        return;
    
    auto rit = handsData.rbegin();
    int& phase = handsData.rbegin()->phase;
    const cv::Point3d& point = rit->point;
    
    auto ritPrev = handsData.rbegin();
    ritPrev++;
    int& phasePrev = ritPrev->phase;
    const cv::Point3d& pointPrev = ritPrev->point;
    
    if(point.x == -1 || pointPrev.x == -1){
        if( phasePrev >= 0 ){
            phase = -1; //конец
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
        if(handsData.size() > 2 && length < GestureFabrique::speedThreshEnd) {
            rit++, ritPrev++;
            cv::Vec3d moveVec = rit->point - ritPrev->point;
             double length = sqrt(static_cast<double>(moveVec[0] * moveVec[0] + moveVec[1] * moveVec[1] + moveVec[2] * moveVec[2]));
             if(length < GestureFabrique::speedThreshEnd){
             phase = -1; //конец
             log();
             eraseHandsData(1); //erased all but last
             }
             else {
                 phase = frameNum; //середина
             }
        }
        else {
            phase = frameNum; //середина
        }
    }
    else if(length > GestureFabrique::speedThreshSlow ) {
            phasePrev = frameNum - 1;
            phase = frameNum; //начало
            eraseHandsData(2); //erased all but 2 lasts
        
    }
    return;
}