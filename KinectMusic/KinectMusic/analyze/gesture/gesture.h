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

class Gesture {
public:
    Gesture(){}
    Gesture(const Blob& blob);
    static void analyzeFrame(const std::list<Blob>& lBlobs);
    static const std::vector<Gesture>& getGesturesConst() {return Gesture::gestures;}
    const std::list<Blob>& getLGestureBlobsConst() const {return this->lGestureBlobs;}
private:
    double dist2blob(const Blob& blob);
    void addBlob(const Blob& blob);
private:
    static std::vector<Gesture> gestures;
    std::list<Blob> lGestureBlobs;
    bool isBlobFound = false;
};
#endif /* Gesture_h */
