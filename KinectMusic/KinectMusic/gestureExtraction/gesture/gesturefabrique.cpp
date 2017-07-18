//
//  gesturefabrique.cpp
//  KinectMusic
//
//  Created by Mikhail Svetlov on 21/05/17.
//  Copyright © 2017 mgsvetlov. All rights reserved.
//


/*
#include "gesturefabrique.h"
#include "../tracking/tracking.h"
#include "../analyze.h"
#include "../../config/config.h"

GestureFabrique* GestureFabrique::gestureFabriquePtr (nullptr)
;

//const double GestureFabrique::spaceCoeff(9./6400);


FrameData GestureFabrique::extractGestures(const std::vector<Track>& tracks){
    if(!gestureFabriquePtr)
        gestureFabriquePtr = new GestureFabrique(Track::trackCount);
    gestureFabriquePtr->addDataToGestures(tracks);
    return gestureFabriquePtr->extractGestures();
}

GestureFabrique::GestureFabrique(size_t gestureCount) {
    for(size_t ind = 0; ind < gestureCount; ind++){
        std::shared_ptr<Gesture> pGesture;
        int gestureType = Config::instance()->getGestureType();
        switch(gestureType) {
            default:
            case 0: pGesture.reset ( new GestureAll(ind)); break;
            case 1: pGesture.reset ( new GestureStop(ind)); break;
            case 2: pGesture.reset ( new GestureReturn(ind)); break;
        }
        
        gestures.push_back(pGesture);
    }
}

void GestureFabrique::destroy(){
    if(gestureFabriquePtr) {
        delete gestureFabriquePtr;
        gestureFabriquePtr = nullptr;
    }
}

void GestureFabrique::addDataToGestures(const std::vector<Track>& tracks){
    for(int i = 0; i < Track::trackCount; i++)
        gestures[i]->addData(tracks[i]);
}

FrameData  GestureFabrique::extractGestures(){
    FrameData frameData;
    frameData.frameNum = frameNum;
    for(auto& gesture : gestures){
        gesture->extract();
        frameData.data.push_back(gesture->getHandsData().back());
    }
    return frameData;
}

cv::Point3d GestureFabrique::convertToRealSpace(const cv::Point3i& p){
    double z = p.z;
    double x = p.x;// * z * spaceCoeff;
    double y = p.y;// * z * spaceCoeff;
    return cv::Point3d (x,y,z);
}

cv::Point3i GestureFabrique::convertToCameraSpace(const cv::Point3d& p){
    int z = p.z;
    int x = p.x;// /( z * spaceCoeff);
    int y = p.y;// /( z * spaceCoeff);
    return cv::Point3d (x,y,z);
}
*/
