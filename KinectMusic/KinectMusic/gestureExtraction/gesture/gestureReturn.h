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
    virtual bool extract();
private:
    static bool isOutThreshold (const cv::Point3d& point1, const cv::Point3d& point2, const cv::Vec3d& thresh);
    static bool isInThreshold (const cv::Point3d& point1, const cv::Point3d& point2, const cv::Vec3d& thresh);
private:
    cv::Point3d startPoint = cv::Point3d(-1, -1, -1);
    bool hasBeenFar = false;
    static cv::Vec3d threshStart;
    static cv::Vec3d threshFar;
    static cv::Vec3d threshReturn;
    
};

#endif /* GestureReturn_h */
