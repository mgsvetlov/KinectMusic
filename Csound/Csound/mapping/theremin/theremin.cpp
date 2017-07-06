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
                    {"midiPitch0",ParamData( 69, 1e-1)},  //midi
                    {"amp0", ParamData(0.001, 1e-1)}, //amp
                    {"vibrRate0", ParamData( 10., 2e-3)}, //vibr rate
                    {"fmodSide0",ParamData( 1.0, 1e-1)}, //mod side
                    {"midiPitch1", ParamData( 69, 1e-1)},  //midi
                    {"amp1",ParamData( 0.001, 1e-1)}, //amp
                    {"vibrRate1", ParamData(10., 2e-3)}, //vibr rate
                    {"fmodSide1", ParamData(1.0, 1e-1)} //mod side
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
    if(frameData.hands[0].phase == NO_DATA_VALUE || frameData.hands[1].phase == NO_DATA_VALUE ){
        csound_data["amp0"].param = csound_data["amp1"].param = 0.001;
    }
    else {
        int rightHandInd = frameData.hands[0].x < frameData.hands[1].x ? 0 : 1;
        
        //vibr
        csound_data["vibrRate0"].param = 2. + ((rand() % 100) / 100. - 0.5) * 2;
        csound_data["vibrRate1"].param = 2. + ((rand() % 100) / 100. - 0.5) * 2;
        for(int i = 0; i < frameData.hands.size(); ++i){
            if(i == rightHandInd){
                //freq
                csound_data["midiPitch0"].param = csound_data["midiPitch1"].param = frameData.hands[i].y * (midiMax - midiMin) + midiMin;
            }
            else {
                //amp
                csound_data["amp0"].param = csound_data["amp1"].param =  sqrt(frameData.hands[i].y);
                //side mod
                float mod = frameData.hands[i].angle;
                if(mod < -50.f)
                    mod = -50.f;
                else if(mod > 50.f)
                    mod = 50.f;
                mod += 50.f;
                csound_data["fmodSide0"].param = csound_data["fmodSide1"].param   = mod * 0.01f;
                
            }
        }
    }
    
}