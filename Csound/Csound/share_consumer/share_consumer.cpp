//
//  share_consumer.cpp
//  Csound
//
//  Created by Mikhail Svetlov on 28/05/17.
//  Copyright © 2017 mgsvetlov. All rights reserved.
//



#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <sys/shm.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <unistd.h>
#include <sys/types.h>
#include <iostream>
#include <sstream>
#include "share_consumer.h"
#include "../log/logs.h"
#include "../mapping/mapping.h"

ShareConsumer* ShareConsumer::shareConsumerPtr = nullptr;

bool ShareConsumer::share(){
    if(!shareConsumerPtr){
        try {
            shareConsumerPtr = new ShareConsumer();
        }
        catch(int e){
            return false;
        }
    }
    return shareConsumerPtr->share_data_consume();
}

void ShareConsumer::destroy(){
    if(shareConsumerPtr)
        delete shareConsumerPtr;
}

ShareConsumer::ShareConsumer() {
    
    SIZE = static_cast<int>(4 * sizeof(int) + 4 * sizeof(int) * 2);
    
    if ((shm_id = shmget( key, SIZE,  0666)) < 0){
        Logs::writeLog("csound","shmget error");
        throw 1;
    }
    if ((ptr = shmat(shm_id, NULL, 0)) == (char *) -1) {
        Logs::writeLog("csound","shmat error");
        throw 2;
    }
    
    if ( (sem = sem_open(SEMAPHORE_NAME,  0)) == SEM_FAILED ) {
        Logs::writeLog("csound","sem_open error");
        throw 3;
    }
}

ShareConsumer::~ShareConsumer(){

}

bool ShareConsumer::share_data_consume(){
    
    sem_wait(sem);
    if(shmget(key, SIZE, 0) < 0){
        
        Logs::writeLog("csound", "Can\'t find shared memory");
        sem_post(sem);
        return false;
    }
    
    static int prevFrameNum (0);
    int* intPtr = static_cast<int*>(ptr);
    if(*intPtr == prevFrameNum){
        sem_post(sem);
        return true;
    }
    prevFrameNum = *intPtr;
   
    FrameData& frameData = Mapping::getData();
    for(int i = 0; i < SIZE / sizeof(int); ++i, intPtr++){
        switch(i){
            case 0:
                frameData.frameNum = *intPtr;
                break;
            case 4:
                frameData.phase1 = *intPtr;
                break;
            case 5:
                frameData.x1 = *intPtr;
                break;
            case 6:
                frameData.y1 = *intPtr;
                break;
            case 7:
                frameData.z1 = *intPtr;
                break;
            case 8:
                frameData.phase2 = *intPtr;
                break;
            case 9:
                frameData.x2 = *intPtr;
                break;
            case 10:
                frameData.y2 = *intPtr;
                break;
            case 11:
                frameData.z2 = *intPtr;
                break;
            default:
                break;
        }
    }
    sem_post(sem);
    

    return true;
}