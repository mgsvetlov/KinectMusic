//
//  noises.hpp
//  Csound
//
//  Created by Mikhail Svetlov on 11/06/17.
//  Copyright © 2017 mgsvetlov. All rights reserved.
//

#ifndef noises_h
#define noises_h

#include <vector>
#include "../mapping.h"

class Noises : public Mapping {
public:
    Noises();
    virtual void initialScoreEvents();
    virtual void mappingData();
    
private:
    std::vector<int> gestureCurrState;
};

#endif /* noises_h */
