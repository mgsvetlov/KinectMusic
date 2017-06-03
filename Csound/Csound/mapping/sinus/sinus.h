//
//  Sinus.hpp
//  Csound
//
//  Created by Mikhail Svetlov on 30/05/17.
//  Copyright © 2017 mgsvetlov. All rights reserved.
//

#ifndef sinus_h
#define sinus_h

#include <vector>
#include "../mapping.h"

class Sinus : public Mapping {
public:
    Sinus();
    virtual void mappingData();

private:
    double freqMin;
    double freqMax;
};

#endif /* sinus_h */
