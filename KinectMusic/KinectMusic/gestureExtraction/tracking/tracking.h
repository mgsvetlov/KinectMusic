//
//  Gesture.hpp
//  KinectMusic
//
//  Created by Mikhail Svetlov on 29/10/16.
//  Copyright © 2016 mgsvetlov. All rights reserved.
//

#ifndef tracking_h
#define tracking_h

#include <stdio.h>
#include "../types.h"

#include "../hand/hand.h"


class Track {
public:
    Track(){}
    Track(const Hand& hand);
    static void analyzeFrame(const std::list<Hand>& lBlobs);
    static const std::vector<Track>& getTracksConst() {return tracks;}
    const std::list<Hand>& getLHands() const { return lHands;}
private:
    double dist2hand(const Hand& hand);
    void addHandData(const Hand& hand);
private:
    static std::vector<Track> tracks;
    bool isHandFound = false;
    std::list<Hand> lHands;
public:
    static const size_t trackCount = 2;
};

#endif /* tracking_h */
