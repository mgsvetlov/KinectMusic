//
//  gesture.hpp
//  KinectMusic
//
//  Created by Mikhail Svetlov on 04/12/16.
//  Copyright © 2016 mgsvetlov. All rights reserved.
//

#ifndef gesture_h
#define gesture_h

#include <stdio.h>
#include "../types.h"

class Track;

struct HandData {
    cv::Point3d point;
    int phase = -1; // -1 not in gest, else frame number
    HandData() : point (-1,-1,-1), phase(-1) {}
    HandData(const cv::Point3d& point, int phase) :
    point(point),
    phase(phase)
    {}
};

class Gesture{
public:
    Gesture() : handInd(0), handsData(){}
    Gesture(size_t ind) : handInd(ind), handsData() {}
    void addData(const Track& track);
    void extract();
private:
    void eraseHandsData(int nonErasedAtEndCount);
    void log();
private:
    size_t handInd;
    std::vector<HandData> handsData;
    
friend class Visualization;
};


#endif /* gesture_h */
