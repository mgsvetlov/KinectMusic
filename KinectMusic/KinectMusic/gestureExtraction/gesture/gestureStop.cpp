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
    const cv::Point3d& point = rit->point;
    
    auto ritPrev = handsData.rbegin();
    ritPrev++;
    int& phasePrev = ritPrev->phase;
    
    if( phasePrev < 0 ) {  //no gesture
        cv::Vec3d moveVec = point - ritPrev->point;
        double length = sqrt(static_cast<double>(moveVec[0] * moveVec[0] + moveVec[1] * moveVec[1] + moveVec[2] * moveVec[2]));
        if(length > speedThreshStart ) {
            phasePrev = frameNum - 1;
            phase = frameNum; //начало
            countUnrecogn = 0;
            eraseHandsData(2); //erased all but 2 lasts
            return true;
        }
        return false;
    }
    else {
        bool isEnd (true);
        if(point.x != -1)
            countUnrecogn = 0;
        else
            countUnrecogn++;
        if(countUnrecogn <= threshUnrecogn) {
            for(int i = 0; i < endIterationsCount; ++i, ++rit, ++ritPrev){
                if(ritPrev == handsData.rend() || rit->point.x == -1 || ritPrev->point.x == -1){
                    isEnd = false;
                    break;
                }
                cv::Vec3d moveVec = rit->point - ritPrev->point;
                double length = sqrt(static_cast<double>(moveVec[0] * moveVec[0] + moveVec[1] * moveVec[1] + moveVec[2] * moveVec[2]));
                if(length > speedThreshEnd){
                    isEnd = false;
                    break;
                }
            }
        }
        if(!isEnd){
            phase = frameNum;  //середина
        }
        else {
            phase = -1; //конец
            log();
            eraseHandsData(1); //erased all but last
            countUnrecogn = 0;
        }
        return true;
    }
}