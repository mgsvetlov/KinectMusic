//
//  Gesture.hpp
//  KinectMusic
//
//  Created by Mikhail Svetlov on 29/10/16.
//  Copyright © 2016 mgsvetlov. All rights reserved.
//

#ifndef Gesture_h
#define Gesture_h

#include <stdio.h>
#include "../types.h"

#include "../hand/hand.h"

struct HandData{
    float x, y, z;
    Hand hand;
    HandData(){}
    HandData(float x, float y, float z, const Hand& hand) :
    x(x), y(y), z(z), hand(hand){}
};

class Tracking {
public:
    Tracking(){}
    Tracking(const Hand& hand);
    static void analyzeFrame(const std::list<Hand>& lBlobs);
    static const std::vector<Tracking>& getTracksConst() {return tracks;}
    const std::list<HandData>& getLHandData() const { return lHandData;}
private:
    double dist2hand(const Hand& hand);
    void addHandData(const Hand& hand);
private:
    static std::vector<Tracking> tracks;
    bool isHandFound = false;
    std::list<HandData> lHandData;
};

#endif /* Gesture_h */
