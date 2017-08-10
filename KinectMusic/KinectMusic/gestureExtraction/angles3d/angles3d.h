//
//  angles3d.hpp
//  KinectMusic
//
//  Created by Mikhail Svetlov on 10/08/17.
//  Copyright © 2017 mgsvetlov. All rights reserved.
//

#ifndef angles3d_hpp
#define angles3d_hpp

#include "../types.h"

struct Plane {
    Plane(): x(0), y(0), z(0), w(0) {};
    Plane(float x, float y, float z, float w): x(x), y(y), z(z), w(w) {};
    float x, y, z, w;
};

class Angles3d {
public:
    Angles3d(std::list<cv::Point3i>& points);
    const std::list<std::tuple<Plane, cv::Point3i, std::list<cv::Point3i>>>& getDataConst() const { return data;}
private:
    cv::Point3i averagePoint(std::list<cv::Point3i>& points);
private:
    std::list<std::tuple<Plane, cv::Point3i, std::list<cv::Point3i>>> data;
};

#endif /* angles3d_hpp */
