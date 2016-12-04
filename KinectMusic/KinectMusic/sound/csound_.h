//
//  csound_.h
//  KinectMusic
//
//  Created by Mikhail Svetlov on 20/09/15.
//  Copyright (c) 2015 mgsvetlov. All rights reserved.
//

#ifndef KinectMusic_csound__h
#define KinectMusic_csound__h

#include "../gestureExtraction/types.h"

#ifdef USE_CSOUND
#include <pthread.h>
#include <csound.hpp>
#include <vector>

extern pthread_t csound_thread;

void *csound_threadfunc(void *arg);

extern CSOUND* csound;
extern std::vector<std::vector<double>> csound_dataDst;

void ramp(double& param, const double paramDst,  double rampCoeff);

#endif //USE_CSOUND
#endif