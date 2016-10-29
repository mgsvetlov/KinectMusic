//
//  Csound_.cpp
//  KinectMusic
//
//  Created by Mikhail Svetlov on 20/09/15.
//  Copyright (c) 2015 mgsvetlov. All rights reserved.
//
#include <unistd.h>
#include "csound_.h"
#include "analyze/types.h"

pthread_t csound_thread;

volatile bool CSOUND_START(false);

void *csound_threadfunc(void *arg){
    //http://sourceforge.net/projects/csound/?source=typ_redirect

    while(!CSOUND_START)
        usleep(10);
    
    int argc_ = 2;
    char* argv_[2];
    argv_[0] = (char*)("Csound");
    argv_[1] = (char*)("//Users//mikhailsvetlov//Dropbox//Politech//Diplom//prototype//KinectMusic//KinectMusic//drums2//test.csd");
    
    /*void *csound = csoundCreate(0);
    int result = csoundCompile(csound, argc_, argv_);
    if(!result) {
        while(csoundPerformKsmps(csound) == 0){}
        csoundCleanup(csound);
    }
    // Destroy Csound.
    csoundDestroy(csound);*/

    Csound *cs = new Csound();
    int res = cs->Compile(argc_, argv_);

    if (res == 0) {
        res = cs->Perform();
    }
   

    return NULL;
}
