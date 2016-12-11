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

class Tracking;

struct HandData {
    cv::Point3i keyPoint;
    int type; //-2 no data, -1 not in gest, 0 gest, 1 start, 2 end
    HandData() {}
    HandData(const cv::Point3i& keyPoint, int type) :
    keyPoint(keyPoint),
    type(type)
    {}
};

class Gesture {
public:
    static void addDataToStream(const std::vector<Tracking>& tracks);
    static void analyzeGestures();
    static void analyzeGesture(std::vector<HandData>& handsTrackedStream);
    static const std::vector<std::vector<HandData>>& getHandsTrackedStreams() {return handsTrackedStreams;}
private:
    static std::vector<std::vector<HandData>> handsTrackedStreams;
};
#endif /* gesture_h */
