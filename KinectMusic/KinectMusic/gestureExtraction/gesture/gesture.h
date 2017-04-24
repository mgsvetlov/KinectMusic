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
    cv::Point3i keyPoint;
    cv::Point3d keyPointCalc;
    int phase = -1; //-2 no data, -1 not in gest, 0 gest, 1 start, 2 end
    int speed = -1;
    int direction = -1;
    HandData() {}
    HandData(const cv::Point3i& keyPoint, const cv::Point3d& keyPointCalc, int phase) :
    keyPoint(keyPoint),
    keyPointCalc(keyPointCalc),
    phase(phase)
    {}
};

class Gesture {
public:
    static void analyzeGestures(const std::vector<Track>& tracks);
    static const std::vector<std::vector<HandData>>& getHandsTrackedStreams() {return handsTrackedStreams;}
private:
    static void addDataToStream(const std::vector<Track>& tracks);
    static void analyzeGesture(std::vector<HandData>& handsTrackedStream);

private:
    static double speedThreshSlow, speedThreshFast, speedThreshEnd;
    static std::vector<std::vector<HandData>> handsTrackedStreams;
    static double spaceCoeff;
};
#endif /* gesture_h */
