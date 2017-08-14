//
//  angles3d.cpp
//  KinectMusic
//
//  Created by Mikhail Svetlov on 10/08/17.
//  Copyright © 2017 mgsvetlov. All rights reserved.
//
#include <sys/stat.h>
#include <limits>
#include  "../pcl/pclplane.h"
#include "../../log/logs.h"
#include "angles3d.h"
#include "../../config/config.h"

Angles3d::Angles3d(std::list<cv::Point3i>& points){
    float x(0.f), y(0.f), z(0.f), w(0.f);
    if(points.size() > 2){
        PclPlane::fitPlane(points, x, y, z, w);
        if(z > 0)
            x = -x, y = -y, z = -z;
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

std::vector<cv::Point2f>  Angles3d::projectPointsToPlane(std::vector<cv::Point3f>& points, const Plane& plane){
    std::vector<cv::Point2f> projPoints;
    projPoints.reserve(points.size());
    const auto& x = plane.x;
    const auto& y = plane.y;
    const auto& z = plane.z;
    int maxDim = x > y ? (x > z ? 0 : 2) : y > z ? 1 : 2;
    float divXY (sqrt(x*x + y*y)), divXZ (sqrt(x*x + z*z)),
    divYZ(sqrt(y*y + z*z));
    cv::Vec3f axe2 = maxDim == 0? cv::Vec3f(-y/divXY, x/divXY, 0.f) : maxDim == 1? cv::Vec3f(0.f, -z/divYZ, y/divYZ) : cv::Vec3f(-z/divXZ, 0.f, x/divXZ);
    cv::Vec3f axe1(-y*axe2[2]+z*axe2[1], -z*axe2[0]+x*axe2[2],-x*axe2[1]+y*axe2[0]);
    /*float check1 = x*axe1[0] + y*axe1[1] + z*axe1[2];
    float check2 = x*axe2[0] + y*axe2[1] + z*axe2[2];
    float check12 = axe1[0]*axe2[0] + axe1[1]*axe2[1] + axe1[1]*axe2[2];
    std::stringstream ss;
     ss << "x " << x << " y " << y << " z " << z << " axe1 " << axe1 <<  " axe2 " << axe2 << " " << check1 << " " << check2 << " " << check12;
    Logs::writeLog("gestures", ss.str());*/
    cv::Point3f origin (points.front());
    float minX(FLT_MAX), minY(FLT_MAX);
    for(auto& p : points){
        float x = p.z;//coordinate(p, axe1, origin);
        float y = p.y;//coordinate(p, axe2, origin);
        if(x < minX)
            minX = x;
        
        if(y < minY)
            minY = y;
        
        projPoints.emplace_back(x,y);
    }
    for(auto& p : projPoints){
        p.x -= minX;
        p.y -= minY;
        
    }
    /*std::stringstream ss1;
    for(int i = 0; i < points.size(); ++i){
        ss1 << points[i] << projPoints[i] << "\n";
    }
    ss1 << "\n" << "minX " << minX << " maxX " << maxX << " minY " << minY << " maxY " << maxY;
    Logs::writeLog("gestures", ss1.str());*/
    return projPoints;
}

float Angles3d::coordinate(const cv::Point3f& point, const cv::Vec3f& axe, const cv::Point3f& origin) {
    if(point == origin)
        return 0;
    cv::Vec3f v = point - origin;
    cv::Vec3f vNorm = cv::normalize(v);
    float cosAngle = vNorm[0]*axe[0] + vNorm[1]*axe[1] + vNorm[2]*axe[2];
    return cv::norm(v) * cosAngle;
}
