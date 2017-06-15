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

#include "../../log/logs.h"
#include "../../config/config.h"

Noises::Noises()
{
//protected
    csdName = "noises";
    csound_data = {
        {"amp0", ParamData(1.0, 1.0)},  //amp
        {"amp1", ParamData(1.0, 1.0)} //amp
    };
    handsDataPrev = { HandData(), HandData() };
};

void Noises::initialScoreEvents(){
  
}

void Noises::mappingData() {
    if(frameData.frameNum == frameNum){
        return ;
    }
    frameNum = frameData.frameNum;
    
    std::vector<int> gestureCurrState { NO_DATA_VALUE, NO_DATA_VALUE};
    for(int i = 0; i < frameData.hands.size(); ++i){
        if(handsDataPrev[i].phase == NO_DATA_VALUE && frameData.hands[i].phase == INSIDE_GESTURE_VALUE){
            gestureCurrState[i] = START_GESTURE_VALUE;
        }
        else if((handsDataPrev[i].phase == START_GESTURE_VALUE ||
                 handsDataPrev[i].phase == INSIDE_GESTURE_VALUE)
                && frameData.hands[i].phase == NO_DATA_VALUE){
            gestureCurrState[i] = END_GESTURE_VALUE;
        }
        else {
            gestureCurrState[i] = frameData.hands[i].phase;
        }
        std::stringstream ss;
        ss << i;
        std::string iStr (ss.str());
        if(handsDataPrev[i].phase == START_GESTURE_VALUE && gestureCurrState[i] == INSIDE_GESTURE_VALUE) {
            GestureType gestureType = distance(handsDataPrev[i], frameData.hands[i]);
            csound_data["amp" + iStr].param = 1.0;
            generateScoreEvent(i, gestureType);
            
        }
        else if(gestureCurrState[i] == END_GESTURE_VALUE) {
            csound_data["amp" + iStr].param = 0.001;
        }
        else {
            
        }
        handsDataPrev[i] = { gestureCurrState[i], frameData.hands[i].x, frameData.hands[i].y, frameData.hands[i].z };
    }
}

Noises::GestureType Noises::distance (const HandData& handData1, const HandData& handData2){
    double dx = std::abs(handData1.x - handData2.x) * 640;
    double dy = std::abs(handData1.y - handData2.y) * 480;
    double dz = std::abs(handData1.z - handData2.z);
    double dist = sqrt(dx*dx + dy*dy + dz*dz);
    Noises::GestureType gestureType = dist < 50? GestureType::SLOW :
                                    //dist < 50 ? GestureType::MIDDLE:
                                                GestureType::FAST;
    return gestureType;
}

void Noises::generateScoreEvent(int handNum, GestureType gestureType){
    static const std::vector<std::vector<MYFLT>> notes = {
        {1., 0., 3.957, 1., 2.},
        {2., 0, 7.486, 2., 2.},
        {1., 0., 0.548, 5., 2.},
        {2., 0, 0.617, 6., 2.},
        {1., 0., 0.275, 7., 2.},
        {2., 0, 0.258, 8., 2.}};
    
    int noteNum  = gestureType == GestureType::SLOW ? handNum :
                  gestureType == GestureType::MIDDLE ? handNum + 2:
    handNum + 4;
    const std::vector<MYFLT>& note = notes[noteNum];
    MYFLT pFields[] = { note[0], note[1], note[2], note[3], note[4]};
    scoreEvent(pFields, 5);
}

/*
 f 1 0 0 1 "sounds/long/001.aiff" 0 0 0 		;3.957
 f 2 0 0 1 "sounds/long/002.aiff" 0 0 0 		;7.486
 f 3 0 0 1 "sounds/long/003.aiff" 0 0 0		    ;2.132
 f 4 0 0 1 "sounds/long/004.aiff" 0 0 0		     ;1.995
 f 5 0 0 1 "sounds/medium/001.aiff" 0 0 0         ;0.548
 f 6 0 0 1 "sounds/medium/002.aiff" 0 0 0         ;0.617
 f 7 0 0 1 "sounds/short/001.aiff" 0 0 0 		;0.275
 f 8 0 0 1 "sounds/short/002.aiff" 0 0 0 		;0.258
 */
