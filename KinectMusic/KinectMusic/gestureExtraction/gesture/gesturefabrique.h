//
//  gesturefabrique.hpp
//  KinectMusic
//
//  Created by Mikhail Svetlov on 21/05/17.
//  Copyright © 2017 mgsvetlov. All rights reserved.
//

#ifndef gesturefabrique_h
#define gesturefabrique_h

#include "gesture.h"
#include <fstream>

class GestureFabrique {
public:
    static void extractGestures(const std::vector<Track>& tracks);
    static const std::vector<Gesture>& getGestures() {return gestureFabriquePtr->gestures;}
    static cv::Point3d convertToRealSpace(const cv::Point3i& p);
    static cv::Point3i convertToCameraSpace(const cv::Point3d& p);
    static void destroy();
private:
    GestureFabrique(size_t gestureCount);
    void addDataToGestures(const std::vector<Track>& tracks);
    void extractGestures();
    std::string getCurrentTime();
public:
    static const double spaceCoeff;
    static const double speedThreshSlow, speedThreshFast, speedThreshEnd;
    static std::ofstream gesturesLog;
private:
    std::vector<Gesture> gestures;
    static GestureFabrique* gestureFabriquePtr;
};


#endif /* gesturefabrique_h */
