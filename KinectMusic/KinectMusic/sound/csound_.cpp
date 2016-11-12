//
//  Csound_.cpp
//  KinectMusic
//
//  Created by Mikhail Svetlov on 20/09/15.
//  Copyright (c) 2015 mgsvetlov. All rights reserved.
//

#ifdef USE_CSOUND

#include <unistd.h>
#include "csound_.h"
#include "../gestureExtraction/types.h"
#include "../mapping/mapping.h"

pthread_t csound_thread;
CSOUND* csound = nullptr;
std::vector<std::vector<double>> csound_dataDst(2,std::vector<double>(2, 0) );

volatile bool CSOUND_START(false);

void *csound_threadfunc(void *arg){
    //http://sourceforge.net/projects/csound/?source=typ_redirect

    while(!CSOUND_START)
        usleep(10);
    
    int argc_ = 2;
    char* argv_[2];
    argv_[0] = (char*)("Csound");
    argv_[1] = (char*)("//Users//mikhailsvetlov//Dropbox//Politech//Diplom//prototype//KinectMusic//KinectMusic//sound//test.csd");
    

    Csound *cs = new Csound();
    csound = cs->GetCsound();
    int result = csoundCompile(csound, argc_, argv_);
    
    if(result == 0){
        std::vector<std::vector<double>> data = { {440, 0.1}, {440, 0.1}};
        Mapping::setPitchVol(data);
        std::vector<std::vector<double>> csound_data = csound_dataDst;
        
        for(int i = 0; i < 2; i ++){
            MYFLT pFields[] = {static_cast<double>(i), 0, -1};
            csoundScoreEvent(csound, 'i', pFields, 3);
        }
        
        int sample_count(0);
        int sec_count(0);
        while(!die && csoundPerformKsmps(csound) == 0)
        {
            for(int i = 0; i < 2; i++){
                std::stringstream ss;
                ss << i;
                std::string chns[] = { "pitch" + ss.str(), "vol" + ss.str()};
                for(int j = 0; j < 2; j++){
                    MYFLT *p;
                    if(csoundGetChannelPtr(csound, &p, chns[j].c_str(), CSOUND_INPUT_CHANNEL | CSOUND_CONTROL_CHANNEL) == 0){
                        ramp(csound_data[i][j], csound_dataDst[i][j], 5e-1);
                        *p = csound_data[i][j];
                    }
                }
            }
            if(++sample_count == 441){
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

#endif //USE_CSOUND