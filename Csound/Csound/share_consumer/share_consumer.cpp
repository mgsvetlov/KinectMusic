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
        std::stringstream ss;
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
    std::stringstream ss;
    for(int i = 0; i < SIZE / sizeof(int); ++i, intPtr++){
        ss << *intPtr<< " ";
    }
    sem_post(sem);
    Logs::writeLog("csound", ss.str());
    return true;
}