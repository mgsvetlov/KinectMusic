//
//  Sinus.cpp
//  Csound
//
//  Created by Mikhail Svetlov on 30/05/17.
//  Copyright © 2017 mgsvetlov. All rights reserved.
//

#include <sstream>
#include "sinus.h"
#include "../../sound/csound_.h"
#include "../../log/logs.h"

void Sinus::mappingData() {
    if(frameData.frameNum == frameNum){
        return ;
    }
    frameNum = frameData.frameNum;
    
    int* dataPtr = reinterpret_cast<int*>(&frameData);
    std::stringstream ss;
    for(int i = 0; i < sizeof(FrameData) / sizeof(int); ++i, ++dataPtr)
        ss << *dataPtr << " ";
    Logs::writeLog("csound", ss.str());
    
    if(frameData.phase1 == NO_DATA_VALUE ){
        csound_dataDst[0][1] = 0.;
    }
    else if(frameData.x1 != NO_DATA_VALUE ){
        csound_dataDst[0][1] = 0.4;
        csound_dataDst[0][0] = (480 - frameData.y1) + 440;
    }
    if(frameData.phase2 == NO_DATA_VALUE){
        csound_dataDst[1][1] = 0.;
    }
    else if(frameData.x2 != NO_DATA_VALUE ){
        csound_dataDst[1][1] = 0.4;
        csound_dataDst[1][0] = (480 - frameData.x2) + 440;
    }
}