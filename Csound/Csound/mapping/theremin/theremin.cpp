//
//  Sinus.cpp
//  Csound
//
//  Created by Mikhail Svetlov on 30/05/17.
//  Copyright © 2017 mgsvetlov. All rights reserved.
//

#include "theremin.h"

#include <sstream>
#include <cmath>
#include "../../csound/csound_.h"
#include "../../log/logs.h"
#include "../../config/config.h"

Theremin::Theremin() {
//protected:
    csdName = "theremin";
    csound_data = {
                    {"midiPitch0",ParamData( 69, 2e-2)},  //midi
                    {"amp0", ParamData(0.01, 1e-2)}, //amp
                    {"vibrRate0", ParamData( 10., 2e-3)}, //vibr rate
                    {"fmodSide0",ParamData( 1.0, 2e-3)}, //mod side
                    {"midiPitch1", ParamData( 69, 2e-2)},  //midi
                    {"amp1",ParamData( 0.01, 1e-2)}, //amp
                    {"vibrRate1", ParamData(10., 2e-3)}, //vibr rate
                    {"fmodSide1", ParamData(1.0, 2e-3)} //mod side
                };
//private:
    midiMin = Config::instance()->getThereminMidiMin();
    midiMax = Config::instance()->getThereminMidiMax();
};

void Theremin::initialScoreEvents(){
    for(int i = 0; i < csound_data.size(); i ++){
        MYFLT pFields[] = {static_cast<double>(i+1), 0, -1};
        scoreEvent(pFields, 3);
    }
}

void Theremin::mappingData() {
    if(frameData.frameNum == frameNum){
        return ;
    }
    frameNum = frameData.frameNum;
    
    if(frameData.hands[0].phase == NO_DATA_VALUE || frameData.hands[1].phase == NO_DATA_VALUE){
        csound_data["amp0"].param = csound_data["amp1"].param = 0.;
    }
    else {
        double freq[2];
        for(int i = 0; i < frameData.hands.size(); ++i){
            //freq
            freq[i] = frameData.hands[i].y * (midiMax - midiMin) + midiMin;
            //amp
            int dz = frameData.hands[i].z - frameData.bodyDepth;
            double vol = pow(-dz / 300., 1);
            if(vol < 0.01)
                vol = 0.01;
            else if(vol > 1.0)
                vol = 1.0;
            vol = pow(vol, 2) * 0.5;
            std::stringstream ss;
            ss << i;
            std::string iStr (ss.str());
            csound_data["amp"+iStr].param = vol;
            //vibr
            csound_data["vibrRate"+iStr].param = 2. + ((rand() % 100) / 100. - 0.5) * 4;
            //side mod
            double x = frameData.hands[i].x;
            x = x < 0.5 ? 0.5 - x : x - 0.5;
            double mod = x  * 2.;
            mod  = mod  < 0. ? 0. : mod;
            csound_data["fmodSide"+iStr].param = mod * 2;
        }
        csound_data["midiPitch0"].param = csound_data["midiPitch1"].param = (freq[0]+ freq[1]) * 0.5;

    }
}