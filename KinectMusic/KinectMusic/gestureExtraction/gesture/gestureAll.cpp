//
//  gestureAll.cpp
//  KinectMusic
//
//  Created by Mikhail Svetlov on 03/06/17.
//  Copyright © 2017 mgsvetlov. All rights reserved.
//

#include "gestureAll.h"

bool GestureAll::extract(){
    size_t size = handsData.size();
    if(size < 2)
        return false;
    auto rit = handsData.rbegin();
    int& phase = handsData.rbegin()->phase;
    const cv::Point3d& point = rit->point;
    if(point.x != NO_DATA_VALUE)
        phase  = INSIDE_GESTURE_VALUE;
    return true;
}