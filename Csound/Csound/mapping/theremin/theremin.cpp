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
                    {"amp0", ParamData(0.01, 1e-1)}, //amp
                    {"vibrRate0", ParamData( 10., 2e-3)}, //vibr rate
                    {"fmodSide0",ParamData( 1.0, 2e-3)}, //mod side
                    {"midiPitch1", ParamData( 69, 1e-2)},  //midi
                    {"amp1",ParamData( 0.01, 1e-1)}, //amp
                    {"vibrRate1", ParamData(10., 2e-3)}, //vibr rate
                    {"fmodSide1", ParamData(1.0, 2e-3)} //mod side
                };
//private:
    midiMin = Config::instance()->getThereminMidiMin();
    midiMax = Config::instance()->getThereminMidiMax();
    handsDataPrev = { HandData(), HandData() };
    vol = {0.0, 0.0};
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
    static bool isSound(false);
    static int z (0), y(0);
    if(frameData.hands[0].phase == NO_DATA_VALUE || frameData.hands[1].phase == NO_DATA_VALUE /*|| handsDataPrev[0].phase == NO_DATA_VALUE || handsDataPrev[1].phase == NO_DATA_VALUE*/){
        //csound_data["amp0"].param = csound_data["amp1"].param = 0.001;
        //isSound = false;
        //z = 0;
    }
    else {
        int minInd = frameData.hands[0].x < frameData.hands[1].x ? 0 : 1;
       
        //double freq[2];
        for(int i = 0; i < frameData.hands.size(); ++i){
            std::stringstream ss;
            ss << i;
            std::string iStr (ss.str());
            //freq
            if(i == minInd){
                csound_data["midiPitch0"].param = csound_data["midiPitch1"].param = frameData.hands[i].y * (midiMax - midiMin) + midiMin;
            }
            else {
                if(handsDataPrev[i].z != -1){
                    int angle = frameData.hands[i].angle;
                    if(isSound){
                        if(angle > 50 ){
                            csound_data["amp0"].param = csound_data["amp1"].param = 0.001;
                            isSound = false;
                            y = frameData.hands[i].y;
                            z = frameData.hands[i].z;
                        }
                    }
                    else {
                        if(z == 0 || (/*angle < 40 &&*/ frameData.hands[i].z < z - 5) ){
                            csound_data["amp0"].param = csound_data["amp1"].param = 1.;
                            isSound = true;
                            //z = frameData.hands[i].z;
                        }
                    }
                    
                    //csound_data["amp0"].param = csound_data["amp1"].param = vol;
                }
            }
            
            
            //vibr
            csound_data["vibrRate"+iStr].param = 2. + ((rand() % 100) / 100. - 0.5) * 4;
            //side mod
            double x = frameData.hands[i].x;
            x = x < 0.5 ? 0.5 - x : x - 0.5;
            double mod = x  * 2.;
            mod  = mod  < 0. ? 0. : mod;
            csound_data["fmodSide"+iStr].param = mod * 2;
        }
        handsDataPrev = frameData.hands;
    }
    
}