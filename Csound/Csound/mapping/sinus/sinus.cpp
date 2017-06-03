//
//  Sinus.cpp
//  Csound
//
//  Created by Mikhail Svetlov on 30/05/17.
//  Copyright © 2017 mgsvetlov. All rights reserved.
//

#include <sstream>
#include <cmath>
#include "sinus.h"
#include "../../csound/csound_.h"
#include "../../log/logs.h"

Sinus::Sinus() {
    csound_dataDst = { {440, 0.01, 0.0, 1.0}};
    csdName = "sinus";
    freqMin = 110;
    freqMax = 880;

    startGestureData = {HandData (NO_DATA_VALUE, NO_DATA_VALUE,NO_DATA_VALUE,NO_DATA_VALUE)};
};

void Sinus::mappingData() {
    if(frameData.frameNum == frameNum){
        return ;
    }
    frameNum = frameData.frameNum;
    
    static FrameData frameDataPrev;
    if(frameData.hands[0].phase == NO_DATA_VALUE || frameData.hands[1].phase == NO_DATA_VALUE){
        csound_dataDst[0][1] = 0.;
    }
    else {
        //freq
        int pitchInd = frameData.hands[0].x > frameData.hands[1].x ?
        0 : 1;
        csound_dataDst[0][0] = frameData.hands[pitchInd].y * (freqMax - freqMin) + freqMin;
        //vol
        int volInd = 1 - pitchInd;
        double y = frameData.hands[volInd].y;
        y = y > 0.5 ? 0.5 : y;
        double vol = (0.5 - frameData.hands[volInd].y) * 2;
        csound_dataDst[0][1] = pow(vol, 2);
        //vibr
        csound_dataDst[0][2] = 10. + ((rand() % 100) / 100. - 0.5) * 4;
        //side mod
        double mod = 1 - frameData.hands[volInd].x  * 2.;
        mod  = mod  < 0. ? 0. : mod;
        csound_dataDst[0][3] = mod * 2;
    }
    
}