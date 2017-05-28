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



class Gesture{
public:
    Gesture() : handInd(0), handsData(){}
    Gesture(size_t ind) : handInd(ind), handsData() {}
    void addData(const Track& track);
    const std::vector<HandData>& getHandsData() const {return handsData;}
    virtual bool extract() = 0;
protected:
    void eraseHandsData(int nonErasedAtEndCount);
protected:
    size_t handInd;
    std::vector<HandData> handsData;
    size_t countUnrecogn = 0;
    static size_t threshUnrecogn;
friend class Visualization;
};


#endif /* gesture_h */
