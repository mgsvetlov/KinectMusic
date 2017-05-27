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
#include "gestureStop.h"
#include "gestureReturn.h"

#include <memory>

class GestureFabrique {
public:
    static FrameData extractGestures(const std::vector<Track>& tracks);
    static const std::vector<std::shared_ptr<Gesture>>& getGestures() {return gestureFabriquePtr->gestures;}
    static cv::Point3d convertToRealSpace(const cv::Point3i& p);
    static cv::Point3i convertToCameraSpace(const cv::Point3d& p);
    static void destroy();
private:
    GestureFabrique(size_t gestureCount);
    void addDataToGestures(const std::vector<Track>& tracks);
    FrameData extractGestures();
public:
    static const double spaceCoeff;

    
private:
    std::vector<std::shared_ptr<Gesture>> gestures;
    static GestureFabrique* gestureFabriquePtr;
};


#endif /* gesturefabrique_h */
