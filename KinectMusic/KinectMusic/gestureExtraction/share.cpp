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
#include "share.h"

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
    sharePtr->share_data(frameData);
    return true;
}

void Share::destroy(){
    if(sharePtr)
        delete sharePtr;
}

Share::Share(const FrameData& frameData){
    /* create the shared memory segment */
    shm_fd = shm_open(name, O_CREAT | O_RDWR, 0666);
    
    /* configure the size of the shared memory segment */
    SIZE = static_cast<int>(4 * sizeof(int) + 4 * sizeof(int) * frameData.data.size());
    ftruncate(shm_fd, SIZE);
    
    /* now map the shared memory segment in the address space of the process */
    ptr = mmap(0, SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0);
    if (ptr == MAP_FAILED) {
        throw 1;
    }
}

Share::~Share(){
    shm_unlink(name);
}

void Share::share_data(const FrameData& frameData){
    int* intPtr = static_cast<int*>(ptr);
    *intPtr = frameData.frameNum;
    for(int i = 0; i < 4; ++i){
        *intPtr++ = 0;
    }
    for(auto& gestureData: frameData.data){
        *intPtr++ = gestureData.phase;
        *intPtr++ = gestureData.point.x;
        *intPtr++ = gestureData.point.y;
        *intPtr++ = gestureData.point.z;
    }
}

