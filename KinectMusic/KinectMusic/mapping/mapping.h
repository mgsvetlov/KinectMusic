//
//  mapping.hpp
//  KinectMusic
//
//  Created by Mikhail Svetlov on 30/10/16.
//  Copyright © 2016 mgsvetlov. All rights reserved.
//

#ifndef mapping_h
#define mapping_h

#include <stdio.h>
#include "../gestureExtraction/types.h"

class Gesture;

class Mapping {
public:
    static void MapDirect(const std::vector<Gesture>& gestures);
    static bool setPitchVol (const std::vector<std::vector<double>>& data);
};

#endif /* mapping_h */
