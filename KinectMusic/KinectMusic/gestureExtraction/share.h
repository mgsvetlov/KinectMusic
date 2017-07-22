//
//  share.hpp
//  KinectMusic
//
//  Created by Mikhail Svetlov on 27/05/17.
//  Copyright © 2017 mgsvetlov. All rights reserved.
//

#ifndef share_h
#define share_h
/*
#include "types.h"
#include <semaphore.h>  

class Share {
public:
    static bool share(const FrameData& frameData);
    static void destroy();
private:
    Share(const FrameData& frameData);
    ~Share();
    bool share_data(const FrameData& frameData);
    static int exec_prog(const char **argv);
private:
    static Share* sharePtr;
    int SIZE = 0;
    int shm_id = -1;
    key_t key = 5678;
    void *ptr = nullptr;
    sem_t *sem;
    const char* SEMAPHORE_NAME = "sem";
};
*/

#endif /* share_h */
