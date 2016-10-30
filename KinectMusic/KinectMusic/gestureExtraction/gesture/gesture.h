//
//  Gesture.hpp
//  KinectMusic
//
//  Created by Mikhail Svetlov on 29/10/16.
//  Copyright © 2016 mgsvetlov. All rights reserved.
//

#ifndef Gesture_h
#define Gesture_h

#include <stdio.h>
#include "../types.h"
#include "../blobs/blob.h"

struct HandData{
    float x, y, z;
    Blob blob;
    HandData(){}
    HandData(float x, float y, float z, const Blob& blob) :
    x(x), y(y), z(z), blob(blob){}
};

class Gesture {
public:
    Gesture(){}
    Gesture(const Blob& blob);
    static void analyzeFrame(const std::list<Blob>& lBlobs);
    static const std::vector<Gesture>& getGesturesConst() {return Gesture::gestures;}
    const std::list<HandData>& getLHandData() const { return lHandData;}
private:
    double dist2blob(const Blob& blob);
    void addHandData(const Blob& blob);
private:
    static std::vector<Gesture> gestures;
    bool isBlobFound = false;
    std::list<HandData> lHandData;
};
#endif /* Gesture_h */
