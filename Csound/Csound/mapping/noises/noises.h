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
#include "../../csound/csound_.h"
#include "../mapping.h"

class Noises : public Mapping {
    enum class GestureType : unsigned char { SLOW, MIDDLE, FAST};
public:
    Noises();
    virtual void initialScoreEvents();
    virtual void mappingData();
private:
    GestureType distance (const HandData& handData1, const HandData& handData2);
    void generateScoreEvent(int handNum, GestureType gestureType);
private:
    std::vector<HandData> handsDataPrev;
    
};

#endif /* noises_h */
