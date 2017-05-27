//
//  share.hpp
//  KinectMusic
//
//  Created by Mikhail Svetlov on 27/05/17.
//  Copyright © 2017 mgsvetlov. All rights reserved.
//

#ifndef share_h
#define share_h

#include "types.h"

class Share {
public:
    static bool share(const FrameData& frameData);
    static void destroy();
private:
    Share(const FrameData& frameData);
    ~Share();
    void share_data(const FrameData& frameData);
private:
    static Share* sharePtr;
    int SIZE = 0;
    const char *name = "/share_gesture";
    int shm_fd;
    void *ptr = nullptr;
};

#endif /* share_h */
