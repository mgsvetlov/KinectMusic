//
//  share.cpp
//  KinectMusic
//
//  Created by Mikhail Svetlov on 27/05/17.
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
#include <future>
#include "share.h"
#include "../log/logs.h"
#include "../config/config.h"
#include "processframedata.h"

Share* Share::sharePtr = nullptr;

bool Share::share(const FrameData& frameData){
    if(!sharePtr){
        try {
            sharePtr = new Share(frameData);
        }
        catch(int e){
            return false;
        }
    }
    return sharePtr->share_data(frameData);
}

void Share::destroy(){
    if(sharePtr) {
        delete sharePtr;
        sharePtr = nullptr;
    }
}

Share::Share(const FrameData& frameData){

    SIZE = static_cast<int>(4 * sizeof(int) + 8 * sizeof(int) * frameData.data.size());
    
    if ((shm_id = shmget( key, SIZE, IPC_CREAT | 0666)) < 0){
        Logs::writeLog("gestures","shmget error");
        throw 1;
    }
    if ((ptr = shmat(shm_id, NULL, 0)) == (char *) -1) {
        Logs::writeLog("gestures","shmat error");
        throw 2;
    }
    if ( (sem = sem_open(SEMAPHORE_NAME, O_CREAT, 0777, 0)) == SEM_FAILED ) {
        Logs::writeLog("gestures","sem_open error");
        throw 3;
    }
    
    if(Config::instance()->getIsCsound()){
        const char    *my_argv[64] = {"./Csound", NULL};
        if(exec_prog(my_argv) == -1){
            Logs::writeLog("gestures","./Csound launch error");
            std::cout << "./Csound launch error" << std::endl;
            throw 4;
        }
        std::cout << "./Csound launched\n" << std::endl;

    }
}

Share::~Share(){
    sem_wait(sem);
    if ( shmctl( shm_id, IPC_RMID, NULL ) == -1 )
        Logs::writeLog("gestures","shmctl failed to destroy");
    sem_post(sem);
    if ( sem_close(sem) < 0 ){
        Logs::writeLog("gestures", "sem_close failed to destroy");
    }
}

bool Share::share_data(const FrameData& frameData){
    sem_wait(sem);
    int* intPtr = static_cast<int*>(ptr);
    *intPtr++ = frameData.frameNum;
    *intPtr++ = frameData.averagedBodyPoint.x;
    *intPtr++ = frameData.averagedBodyPoint.y;
    *intPtr++ = frameData.averagedBodyPoint.z;
    for(auto& gestureData: frameData.data){
        *intPtr++ = NO_DATA_VALUE;
        *intPtr++ = gestureData.keyPoint.x;
        *intPtr++ = gestureData.keyPoint.y;
        *intPtr++ = gestureData.keyPoint.z;
        *intPtr++ = NO_DATA_VALUE;
        *intPtr++ = static_cast<int>(gestureData.normal[0] *1e6);
        *intPtr++ = static_cast<int>(gestureData.normal[1] *1e6);
        *intPtr++ = static_cast<int>(gestureData.normal[2] *1e6);
    }
    sem_post(sem);
    return true;
}

int Share::exec_prog(const char **argv){
    pid_t   my_pid;
    
    if (0 == (my_pid = fork())) {
        if (-1 == execve(argv[0], (char **)argv , NULL)) {
            Logs::writeLog("gestures", "child process execve failed");
            return -1;
        }
    }
    return 0;
}
