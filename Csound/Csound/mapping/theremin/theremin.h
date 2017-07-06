//
//  Sinus.hpp
//  Csound
//
//  Created by Mikhail Svetlov on 30/05/17.
//  Copyright © 2017 mgsvetlov. All rights reserved.
//

#ifndef theremin_h
#define theremin_h

#include <vector>
#include "../mapping.h"

class Theremin : public Mapping {
public:
    Theremin();
    virtual void initialScoreEvents();
    virtual void mappingData();

private:
    double midiMin;
    double midiMax;
};

#endif /* theremin_h */
