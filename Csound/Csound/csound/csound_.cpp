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

        const auto& csound_dataDst = mapping->getCsound_dataDst();
        
        for(int i = 0; i < csound_dataDst.size(); i ++){
            MYFLT pFields[] = {static_cast<double>(i+1), 0, -1};
            csoundScoreEvent(csound, 'i', pFields, 3);
        }
        
        std::vector<std::vector<double>> csound_data = csound_dataDst;
        int sample_count(0);
        int sec_count(0);
        
        while(!die && csoundPerformKsmps(csound) == 0)
        {
            for(int i = 0; i < csound_dataDst.size(); i++){
                std::stringstream ssinstr;
                ssinstr << i + 1;
                for(int j = 0; j < csound_dataDst[i].size(); j++){
                    std::stringstream ssparam;
                    ssparam << j;
                    std::string chn =  "p" + ssinstr.str() + ssparam.str();
                    MYFLT *p;
                    if(csoundGetChannelPtr(csound, &p, chn.c_str(), CSOUND_INPUT_CHANNEL | CSOUND_CONTROL_CHANNEL) == 0){
                        ramp(csound_data[i][j], csound_dataDst[i][j], 3e-2);
                        *p = csound_dataDst[i][j];
                    }
                }
            }
            if(++sample_count == 4410){
                ++sec_count;
                sample_count = 0;
            }
        }
        csoundCleanup(csound);
    }
    delete cs;
    return NULL;
}

void ramp(double& param, const double paramDst,  double rampCoeff){
    double delta = (paramDst - param) * rampCoeff;
    if(param < 1.0 && std::abs(delta) > 0.001)
        delta = delta > 0 ? 0.01 : -0.01;
        
    param += delta;
    if(param < 0)
        param = 0;
}

