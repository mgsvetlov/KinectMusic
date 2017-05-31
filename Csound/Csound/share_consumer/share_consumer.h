//
//  share_consumer.hpp
//  Csound
//
//  Created by Mikhail Svetlov on 28/05/17.
//  Copyright © 2017 mgsvetlov. All rights reserved.
//

#ifndef share_consumer_h
#define share_consumer_h
#include <semaphore.h>

class Mapping;

class ShareConsumer {
public:
    static bool share(Mapping* );
    static void destroy();
private:
    ShareConsumer();
    ~ShareConsumer();
    bool share_data_consume(Mapping* );
private:
    static ShareConsumer* shareConsumerPtr;
    int SIZE = 0;
    int shm_id = -1;
    key_t key = 5678;
    void *ptr = nullptr;
    sem_t *sem;
    const char* SEMAPHORE_NAME = "sem";
};

#endif /* share_consumer_h */
