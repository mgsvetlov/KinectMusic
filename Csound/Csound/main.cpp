//
//  main.cpp
//  Csound
//
//  Created by Mikhail Svetlov on 28/05/17.
//  Copyright © 2017 mgsvetlov. All rights reserved.
//

#include <iostream>
#include <unistd.h>
#include "csound/csound_.h"
#include "share_consumer/share_consumer.h"
#include "log/logs.h"
#include "config/config.h"
#include "mapping/mapping.h"
#include "mapping/theremin/theremin.h"
#include "mapping/noises/noises.h"

int main(int argc, const char * argv[]) {
    
    Config::setFileName("../../../config.ini");
    Config* config = Config::instance();
    if(config == nullptr){
        Logs::closeLogs();
        return -1;
    }
    
    Mapping* mapping;
    if(Config::instance()->getMappingType() == 0)
        mapping = new Theremin();
    else
        mapping = new Noises();
    
    int res = pthread_create(&csound_thread, NULL, csound_threadfunc, mapping);
    if (res) {
        printf("pthread_create csound_thread failed\n");
        return 1;
    }
    
    while(true){
        if(!ShareConsumer::share(mapping))
            break;
        mapping->mappingData();
        usleep(1000);
    }
    
    die = 1;
    csoundStop(csound);
    pthread_join(csound_thread, NULL);
    
    ShareConsumer::destroy();
    delete mapping;
    Logs::closeLogs();
    
    return 0;
}
