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

double GestureStop::speedThreshStart (8e3 * GestureFabrique::spaceCoeff),  GestureStop::speedThreshEnd(2e3 * GestureFabrique::spaceCoeff);
int GestureStop::endIterationsCount = 1;

bool GestureStop::extract(){
    size_t size = handsData.size();
    if(size < 2)
        return false;
    
    auto rit = handsData.rbegin();
    int& phase = handsData.rbegin()->phase;
    const auto& point = rit->point;
    
    auto ritPrev = handsData.rbegin();
    ritPrev++;
    int& phasePrev = ritPrev->phase;
    
    if( phasePrev == NO_DATA_VALUE || phasePrev ==END_GESTURE_VALUE ) {  //no gesture
        const auto& moveVec = point - ritPrev->point;
        double length = sqrt(static_cast<double>(moveVec.x * moveVec.x + moveVec.y * moveVec.y + moveVec.z * moveVec.z));
        if(length > speedThreshStart ) {
            phasePrev = START_GESTURE_VALUE;
            phase = INSIDE_GESTURE_VALUE; //начало
            countUnrecogn = 0;
            eraseHandsData(2); //erased all but 2 lasts
            return true;
        }
        return false;
    }
    else {
        bool isEnd (true);
        if(point.x != NO_DATA_VALUE)
            countUnrecogn = 0;
        else
            countUnrecogn++;
        if(countUnrecogn <= threshUnrecogn) {
            for(int i = 0; i < endIterationsCount; ++i, ++rit, ++ritPrev){
                if(ritPrev == handsData.rend() || rit->point.x == NO_DATA_VALUE || ritPrev->point.x == NO_DATA_VALUE){
                    isEnd = false;
                    break;
                }
                const auto moveVec = rit->point - ritPrev->point;
                double length = sqrt(static_cast<double>(moveVec.x * moveVec.x + moveVec.y * moveVec.y + moveVec.z * moveVec.z));
                if(length > speedThreshEnd){
                    isEnd = false;
                    break;
                }
            }
        }
        if(!isEnd){
            phase = INSIDE_GESTURE_VALUE;  //середина
        }
        else {
            phase = END_GESTURE_VALUE ; //конец

            eraseHandsData(1); //erased all but last
            countUnrecogn = 0;
        }
        return true;
    }
}