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

cv::Vec3d GestureReturn::threshStart = cv::Vec3d(-1e6,-1e6,10);
cv::Vec3d GestureReturn::threshFar = cv::Vec3d(-1e6,-1e6,60);
cv::Vec3d GestureReturn::threshReturn = cv::Vec3d(1e6,1e6,30);

bool GestureReturn::extract(){
    size_t size = handsData.size();
    if(size < 2)
        return false;
    
    auto rit = handsData.rbegin();
    int& phase = handsData.rbegin()->phase;
    const cv::Point3d& point = rit->point;
    
    auto ritPrev = handsData.rbegin();
    ritPrev++;
    int& phasePrev = ritPrev->phase;
    
    if( phasePrev < 0 ) {
        if(point.x != -1 && isOutThreshold(ritPrev->point, point, threshStart)) {
            phasePrev = frameNum - 1;
            phase = frameNum; //начало
            countUnrecogn = 0;
            startPoint = point;
            hasBeenFar = false;
            eraseHandsData(2); //erased all but 2 lasts
            return true;
        }
        return false;
    }
    else {
        if(point.x != -1)
            countUnrecogn = 0;
        else
            countUnrecogn++;
        if(countUnrecogn > threshUnrecogn || (hasBeenFar && isInThreshold(startPoint,point,  threshReturn))){
                phase = -1; //конец
                log();
                eraseHandsData(1); //erased all but last
            }
            else {
                if(point.x != -1 && !hasBeenFar && isOutThreshold(startPoint, point, threshFar))
                   hasBeenFar = true;
                phase = frameNum; //середина
            }
        return true;
    }
}

bool GestureReturn::isOutThreshold (const cv::Point3d& point1, const cv::Point3d& point2, const cv::Vec3d& thresh)  {
    cv::Vec3d v = point1 - point2;
    for(int i = 0; i < 3; i ++){
        if(v[i] < thresh[i])
            return false;
    }
    return true;
}

bool GestureReturn::isInThreshold (const cv::Point3d& point1, const cv::Point3d& point2, const cv::Vec3d& thresh)  {
    cv::Vec3d v = point1 - point2;
    for(int i = 0; i < 3; i ++){
        if(v[i] > thresh[i])
            return false;
    }
    return true;
}