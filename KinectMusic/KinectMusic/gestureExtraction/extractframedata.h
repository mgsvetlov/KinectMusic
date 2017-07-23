//
//  extractframedata.h
//  KinectMusic
//
//  Created by Mikhail Svetlov on 17/09/16.
//  Copyright © 2016 mgsvetlov. All rights reserved.
//

#ifndef extractframedata_h
#define extractframedata_h

#include  <stdint.h>
#include <pthread.h>

struct ExtractFrameData {
public:
    static void *threadfunc(void *arg);
private:
    static void fpsLog(int frameNumExtract);
    static void startLog();

public:
    
    static volatile int die_gesture;
};

//struct FrameData;
//void log(FrameData& frameData);

#endif /* extractframedata_h */
