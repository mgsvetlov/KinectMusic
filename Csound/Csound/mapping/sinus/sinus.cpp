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
    csound_data = { {ParamData(69, 2e-2),  //midi
                    ParamData(0.01, 1e-2), //amp
                    ParamData(10., 2e-3), //vibr rate
                    ParamData(1.0, 2e-3)},
                    {ParamData(69, 2e-2),  //midi
                    ParamData(0.01, 1e-2), //amp
                    ParamData(10., 2e-3), //vibr rate
                    ParamData(1.0, 2e-3)}}; //mod side
    csdName = "sinus";
    midiMin = 60;
    midiMax = 90;

    startGestureData = {HandData (NO_DATA_VALUE, NO_DATA_VALUE,NO_DATA_VALUE,NO_DATA_VALUE)};
};

void Sinus::mappingData() {
    if(frameData.frameNum == frameNum){
        return ;
    }
    frameNum = frameData.frameNum;
    
    if(frameData.hands[0].phase == NO_DATA_VALUE || frameData.hands[1].phase == NO_DATA_VALUE){
        csound_data[0][1].param = csound_data[1][1].param = 0.;
    }
    else {
        double freq[2];
        for(int i = 0; i < frameData.hands.size(); ++i){
            //freq
            freq[i] = frameData.hands[i].y * (midiMax - midiMin) + midiMin;
            //vol
            int dz = frameData.hands[i].z - frameData.bodyDepth;
            double vol = pow(-dz / 300., 1);
            if(vol < 0.01)
                vol = 0.01;
            else if(vol > 1.0)
                vol = 1.0;
            vol = pow(vol, 2) * 0.5;
            csound_data[i][1].param = vol;
            //vibr
            csound_data[i][2].param = 10. + ((rand() % 100) / 100. - 0.5) * 4;
            //side mod
            double x = frameData.hands[i].x;
            x = x < 0.5 ? 0.5 - x : x - 0.5;
            double mod = x  * 2.;
            mod  = mod  < 0. ? 0. : mod;
            csound_data[i][3].param = mod * 2;
        }
        csound_data[0][0].param = csound_data[1][0].param = (freq[0]+ freq[1]) * 0.5;

    }
}