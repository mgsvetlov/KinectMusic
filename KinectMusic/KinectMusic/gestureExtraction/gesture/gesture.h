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
    cv::Vec3d moveFromStartVec= cv::Point3d(0,0,0);
    int phase = -1; //-2 no data, -1 not in gest, 0 gest, 1 start, -100 end
    int speed = -1;
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

public:
    static double speedThreshSlow, speedThreshFast, speedThreshEnd;
private:
    static std::vector<std::vector<HandData>> handsTrackedStreams;
    static double spaceCoeff;
};
#endif /* gesture_h */
