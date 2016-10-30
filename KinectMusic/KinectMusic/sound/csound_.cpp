//
//  Csound_.cpp
//  KinectMusic
//
//  Created by Mikhail Svetlov on 20/09/15.
//  Copyright (c) 2015 mgsvetlov. All rights reserved.
//
#include <unistd.h>
#include "csound_.h"
#include "../gestureExtraction/types.h"
#include "../mapping/mapping.h"

pthread_t csound_thread;
CSOUND* csound = nullptr;

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
        std::vector<std::vector<double>> data = { {0, 0}, {0, 0}};
        Mapping::setPitchVol(data);
        
        cs->Perform();
        
        while(!die /*&& csoundPerformKsmps(csound) == 0*/)
        {
            usleep(10);
        }
        csoundCleanup(csound);
    }
    delete cs;
    return NULL;
}
