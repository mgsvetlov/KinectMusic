//
//  Csound_.cpp
//  KinectMusic
//
//  Created by Mikhail Svetlov on 20/09/15.
//  Copyright (c) 2015 mgsvetlov. All rights reserved.
//

#include <unistd.h>
#include <string>
#include <sstream>
#include <cmath>

#include "csound_.h"
#include "../mapping/mapping.h"


pthread_t csound_thread;
bool die = 0;
CSOUND* csound = nullptr;

void *csound_threadfunc(void *arg){
    //http://sourceforge.net/projects/csound/?source=typ_redirect
    
    Mapping* mapping = reinterpret_cast<Mapping*> (arg);
    
    const std::string& name = mapping->getCsdName();
    const std::string fullPath = std::string("..//..//..//Csound//Csound//mapping//") + name + std::string("//") + name + std::string(".csd");
    int argc_ = 2;
    char* argv_[2];
    argv_[0] = (char*)("Csound");
    argv_[1] = (char*)(fullPath.c_str());
    
    Csound *cs = new Csound();
    csound = cs->GetCsound();
    int result = csoundCompile(csound, argc_, argv_);
    
    if(result == 0){
        
        mapping->initialScoreEvents();
        
        const auto& csound_data = mapping->getCsound_data();
        auto csound_dataPrev = csound_data;
        
        while(!die && csoundPerformKsmps(csound) == 0)
        {
            for(const auto& pair : csound_data) {
                const std::string& name = pair.first;
                const ParamData& paramData = pair.second;
                MYFLT *p;
                if(csoundGetChannelPtr(csound, &p, name.c_str(), CSOUND_INPUT_CHANNEL | CSOUND_CONTROL_CHANNEL) == 0){
                    ramp(csound_dataPrev[name].param, paramData.param, paramData.ramp);
                    *p = csound_dataPrev[name].param;
                }
            }
        }
        
        csoundCleanup(csound);
        
    }
    delete cs;
    return NULL;
}

void ramp(double& param, const double paramDst,  double ramp){
    double delta = paramDst - param;
    if(std::abs(delta) < ramp){
        param = paramDst;
    }
    else {
        param = delta > 0. ? param + ramp : param - ramp;
    }
}

void scoreEvent(MYFLT arr[], long numField){
    csoundScoreEvent(csound, 'i', arr, numField);
}

