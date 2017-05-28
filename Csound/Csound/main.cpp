//
//  main.cpp
//  Csound
//
//  Created by Mikhail Svetlov on 28/05/17.
//  Copyright © 2017 mgsvetlov. All rights reserved.
//

#include <iostream>
#include <unistd.h>
#include "sound/csound_.h"
#include "share_consumer/share_consumer.h"
#include "log/logs.h"

int main(int argc, const char * argv[]) {
    
    int res = pthread_create(&csound_thread, NULL, csound_threadfunc, NULL);
    if (res) {
        printf("pthread_create csound_thread failed\n");
        return 1;
    }
    
    while(true){
        if(!ShareConsumer::share())
            break;
        usleep(1000);
    }
    
    die = 1;
    csoundStop(csound);
    pthread_join(csound_thread, NULL);
    
    ShareConsumer::destroy();
    
    Logs::closeLogs();
    
    return 0;
}
