//
//  gestureAll.hpp
//  KinectMusic
//
//  Created by Mikhail Svetlov on 03/06/17.
//  Copyright © 2017 mgsvetlov. All rights reserved.
//

#ifndef gestureAll_hpp
#define gestureAll_hpp

#include "gesture.h"

class GestureAll : public Gesture {
public:
    GestureAll() {}
    GestureAll(size_t ind) : Gesture(ind) {}
    virtual bool extract();
    
};

#endif /* gestureAll_hpp */
