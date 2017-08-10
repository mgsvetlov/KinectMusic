//
//  angles3d.cpp
//  KinectMusic
//
//  Created by Mikhail Svetlov on 10/08/17.
//  Copyright © 2017 mgsvetlov. All rights reserved.
//
#include  "../pcl/pclplane.h"
#include "../../log/logs.h"
#include "angles3d.h"

Angles3d::Angles3d(std::list<cv::Point3i>& points){
    float x(0.f), y(0.f), z(0.f), w(0.f);
    if(points.size() > 2){
        PclPlane::fitPlane(points, x, y, z, w);
        /*std::stringstream ss;
        ss << "plane " << x << " " << y << " " << z << " " << w << " " ;
        Logs::writeLog("gestures", ss.str());*/
        data.emplace_back(Plane(x, y, z, w), averagePoint(points), points);
    }
}

cv::Point3i Angles3d::averagePoint(std::list<cv::Point3i>& points){
    cv::Point3i avgPoint(0,0,0);
    if(points.empty())
        return avgPoint;
    for(auto& p : points)
        avgPoint += p;
    size_t count = points.size();
    avgPoint.x /= count;
    avgPoint.y /= count;
    avgPoint.z /= count;
    return avgPoint;
}