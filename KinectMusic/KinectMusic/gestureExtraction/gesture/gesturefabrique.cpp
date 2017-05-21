//
//  gesturefabrique.cpp
//  KinectMusic
//
//  Created by Mikhail Svetlov on 21/05/17.
//  Copyright © 2017 mgsvetlov. All rights reserved.
//

#include <ctime>
#include <sstream>

#include "gesturefabrique.h"
#include "../tracking/tracking.h"

GestureFabrique* GestureFabrique::gestureFabriquePtr (nullptr)
;

const double GestureFabrique::spaceCoeff(9./6400);
const double GestureFabrique::speedThreshSlow (8e3 * GestureFabrique::spaceCoeff), GestureFabrique::speedThreshFast(16e3 * GestureFabrique::spaceCoeff), GestureFabrique::speedThreshEnd(2e3 * GestureFabrique::spaceCoeff);
std::ofstream GestureFabrique::gesturesLog;

void GestureFabrique::extractGestures(const std::vector<Track>& tracks){
    if(!gestureFabriquePtr)
        gestureFabriquePtr = new GestureFabrique(Track::trackCount);
    gestureFabriquePtr->addDataToGestures(tracks);
    gestureFabriquePtr->extractGestures();
}

GestureFabrique::GestureFabrique(size_t gestureCount) {
    for(size_t ind = 0; ind < gestureCount; ind++)
        gestures.push_back(Gesture(ind));
    gesturesLog.open("gestures" + getCurrentTime() + ".log");
}

void GestureFabrique::destroy(){
    gesturesLog.close();
    if(gestureFabriquePtr)
        delete gestureFabriquePtr;
}

void GestureFabrique::addDataToGestures(const std::vector<Track>& tracks){
    for(int i = 0; i < Track::trackCount; i++)
        gestures[i].addData(tracks[i]);
}

void GestureFabrique::extractGestures(){
    for(auto& gesture : gestures)
        gesture.extract();
}

std::string GestureFabrique::getCurrentTime(){
    time_t t = time(0);   // get time now
    struct tm * now = localtime( & t );
    std::stringstream ss;
    ss << '_' << (now->tm_year + 1900) << '_' << (now->tm_mon + 1) << '_'
    <<  now->tm_mday << '_'<< now->tm_hour << '_'<< now->tm_min;
    return ss.str();
}

cv::Point3d GestureFabrique::convertToRealSpace(const cv::Point3i& p){
    double z = p.z;
    double x = p.x * z * spaceCoeff;
    double y = p.y * z * spaceCoeff;
    return cv::Point3d (x,y,z);
}

cv::Point3i GestureFabrique::convertToCameraSpace(const cv::Point3d& p){
    int z = p.z;
    int x = p.x /( z * spaceCoeff);
    int y = p.y /( z * spaceCoeff);
    return cv::Point3d (x,y,z);
}

