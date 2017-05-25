//
//  GestureReturn.hpp
//  KinectMusic
//
//  Created by Mikhail Svetlov on 25/05/17.
//  Copyright © 2017 mgsvetlov. All rights reserved.
//

#ifndef GestureReturn_h
#define GestureReturn_h

#include "gesture.h"

class GestureReturn : public Gesture {
public:
    GestureReturn() {}
    GestureReturn(size_t ind) : Gesture(ind) {}
    virtual void extract();
private:
    bool isOutThreshold (const cv::Point3d& point, int ind) const;
    bool isInThreshold (const cv::Point3d& point, int ind) const;
private:
    cv::Point3d startPoint = cv::Point3d(-1, -1, -1);
    std::vector<cv::Vec3d> thresholds = { cv::Vec3d(1e6,1e6,30), cv::Vec3d(0,0,60) };
    bool isThreshPassed = false;
};

#endif /* GestureReturn_h */
