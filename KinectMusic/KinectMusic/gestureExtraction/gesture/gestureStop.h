//
//  gestureStop.hpp
//  KinectMusic
//
//  Created by Mikhail Svetlov on 25/05/17.
//  Copyright © 2017 mgsvetlov. All rights reserved.
//

#ifndef gestureStop_h
#define gestureStop_h

#include "gesture.h"

class GestureStop : public Gesture {
public:
    GestureStop() {}
    GestureStop(size_t ind) : Gesture(ind) {}
    virtual void extract();
private:

};

#endif /* gestureStop_h */
