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
    static double  y(1.);
    if(frameData.hands[0].phase == NO_DATA_VALUE || frameData.hands[1].phase == NO_DATA_VALUE ){
        csound_data["amp0"].param = csound_data["amp1"].param = 0.001;
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
                    if(isSound){
                        if(frameData.hands[i].angle  > 0
                            && frameData.hands[i].y > y + 0.03 ){
                            csound_data["amp0"].param = csound_data["amp1"].param = 0.001;
                            isSound = false;
                            y = frameData.hands[i].y;
                            std::stringstream ss;
                            ss << "OFF angle " << frameData.hands[i].angle<< " y " << frameData.hands[i].y;
                            Logs::writeLog("csound", ss.str());
                        }
                    }
                    else {
                        if(frameData.hands[i].angle < -30
                            && frameData.hands[i].y < y - 0.03
                           ){
                            csound_data["amp0"].param = csound_data["amp1"].param = 1.;
                            isSound = true;
                            y = frameData.hands[i].y;
                            std::stringstream ss;
                            ss << "ON angle " << frameData.hands[i].angle << " y " << frameData.hands[i].y;
                            Logs::writeLog("csound", ss.str());
                        }
                    }
                    
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