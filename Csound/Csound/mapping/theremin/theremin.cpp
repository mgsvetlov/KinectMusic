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
                    {"midiPitch0",ParamData( 69, 1e-2)},  //midi
                    {"amp0", ParamData(0.001, 1e-2)}, //amp
                    {"vibrRate0", ParamData( 10., 2e-3)}, //vibr rate
                    {"fmodSide0",ParamData( 1.0, 1e-1)}, //mod side
                    {"midiPitch1", ParamData( 69, 1e-2)},  //midi
                    {"amp1",ParamData( 0.001, 1e-2)}, //amp
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
    
    static FrameData frameDataPrev;
    
    frameNum = frameData.frameNum;
    if(frameData.hands[0].x == NO_DATA_VALUE || frameData.hands[1].x == NO_DATA_VALUE ){
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
                if(frameDataPrev.frameNum != NO_DATA_VALUE){
                    double amp = frameData.hands[i].n_y < 0 ? 1.0 : 0.0;
                    /*double dy = std::abs(frameData.hands[i].y - frameDataPrev.hands[i].y);
                    double dx = std::abs(frameData.hands[i].x - frameDataPrev.hands[i].x);
                    double amp = dy + dx;// > 0.01 ? 1.0 : 0.0;
                    static double thresh = 0.01;
                    if(amp > thresh){
                        amp *= pow(amp * 1. /thresh, 15);
                    }
                    if(amp > 1)
                        amp = 1;
                    amp *= (1.0 - frameData.hands[i].y);*/
                    csound_data["amp0"].param = csound_data["amp1"].param = amp;
                }
                //csound_data["amp0"].param = csound_data["amp1"].param =  sqrt(frameData.hands[i].y);
                //side mod
                float mod = 0.0f;
                if(mod < -50.f)
                    mod = -50.f;
                else if(mod > 50.f)
                    mod = 50.f;
                mod += 50.f;
                csound_data["fmodSide0"].param = csound_data["fmodSide1"].param   = mod * 0.01f;
                
            }
        }
    }
    
    frameDataPrev = frameData;
}