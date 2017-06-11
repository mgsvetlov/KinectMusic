//
//  noises.cpp
//  Csound
//
//  Created by Mikhail Svetlov on 11/06/17.
//  Copyright © 2017 mgsvetlov. All rights reserved.
//

#include "noises.h"

#include <sstream>
#include <cmath>
#include "../../csound/csound_.h"
#include "../../log/logs.h"
#include "../../config/config.h"

Noises::Noises() {
//protected
    csdName = "noises";
    csound_data = {
        {"amp0", ParamData(1.0, 1.0)},  //amp
        {"amp1", ParamData(1.0, 1.0)} //amp
    };
    gestureCurrState = { NO_DATA_VALUE, NO_DATA_VALUE};
};

void Noises::initialScoreEvents(){
  
}

void Noises::mappingData() {
    if(frameData.frameNum == frameNum){
        return ;
    }
    frameNum = frameData.frameNum;
    for(int i = 0; i < frameData.hands.size(); ++i){
        if(gestureCurrState[i] == NO_DATA_VALUE && frameData.hands[i].phase == INSIDE_GESTURE_VALUE){
            gestureCurrState[i] = START_GESTURE_VALUE;
        }
        else if((gestureCurrState[i] == START_GESTURE_VALUE ||
                 gestureCurrState[i] == INSIDE_GESTURE_VALUE)
                && frameData.hands[i].phase == NO_DATA_VALUE){
            gestureCurrState[i] = END_GESTURE_VALUE;
        }
        else {
            gestureCurrState[i] = frameData.hands[i].phase;
        }
        std::stringstream ss;
        ss << i;
        std::string iStr (ss.str());
        if(gestureCurrState[i] == START_GESTURE_VALUE) {
            csound_data["amp" + iStr].param = 1.0;
            //csound_data["amp" + iStr].rampCoeff = 1.0;
            if(i == 0){
                MYFLT pFields[] = {static_cast<double>(i+1), 0, 2.132, 1, 2};
                scoreEvent(pFields, 5);
            }
            else {
                MYFLT pFields[] = {static_cast<double>(i+1), 0, 3.957, 3, 2};
                scoreEvent(pFields, 5);
            }
        }
        else if(gestureCurrState[i] == END_GESTURE_VALUE) {
            csound_data["amp" + iStr].param = 0.001;
            //csound_data["amp" + iStr].rampCoeff = 1e-2;
        }
        else {
            //csound_data["amp" + iStr].param = 1.0;
        }
    }
}
