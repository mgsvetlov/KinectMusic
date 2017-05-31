//
//  csound_.h
//  KinectMusic
//
//  Created by Mikhail Svetlov on 20/09/15.
//  Copyright (c) 2015 mgsvetlov. All rights reserved.
//

#ifndef KinectMusic_csound__h
#define KinectMusic_csound__h


#include <pthread.h>
#include <csound.hpp>
#include <vector>

extern pthread_t csound_thread;

void *csound_threadfunc(void *arg);
extern bool die;
extern CSOUND* csound;

void ramp(double& param, const double paramDst,  double rampCoeff);


#endif
