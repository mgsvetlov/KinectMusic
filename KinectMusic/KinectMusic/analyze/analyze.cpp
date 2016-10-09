//
//  analyze.c
//  KinectMusic
//
//  Created by Mikhail Svetlov on 17/09/16.
//  Copyright © 2016 mgsvetlov. All rights reserved.
//

#include <sys/mman.h>   /* shared memory and mmap() */
#include <unistd.h>     /* for getopt() */
#include <errno.h>      /* errno and perror */
#include <fcntl.h>      /* O_flags */
#include <iostream>
#include <queue>

#include "types.h"
#include "visualization/visualization.h"
#include "analyze.h"
#include "blobs/blob.h"
#include "blobs/blobsmask.h"

pthread_t freenect_thread;
pthread_mutex_t depth_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t video_mutex = PTHREAD_MUTEX_INITIALIZER;
volatile int die = 0;

int w = 640, h = 480;
uint16_t * const depthAnalyze = new uint16_t[w*h];

void analyzeLoop(){
    
    pthread_mutex_lock(&depth_mutex);
    cv::Mat mat16(h, w, CV_16U, depthAnalyze);
    pthread_mutex_unlock(&depth_mutex);

    cv::Mat mat16_resized;
    cv::resize(mat16, mat16_resized, cv::Size(w>>BLOBS_RESIZE_POW, h>>BLOBS_RESIZE_POW));
    
    std::list<Blob> lBlobs;
    Blob::findBlobs(mat16_resized, lBlobs);
  
    Visualization::visualizeMap(mat16_resized.size(), mat16.size(), lBlobs);
    
    BlobsMask blobsMask (lBlobs, mat16_resized.size(), mat16.size());
    cv::Mat mat_result = blobsMask.applyMask(mat16);
    Visualization::visualize(mat_result);
  
    if(cv::waitKey(30) == 27) {
        die = 1;
        pthread_join(freenect_thread, NULL);
        //pthread_join(csound_thread, NULL);
        free(rgb_back);
        free(rgb_mid);
        free(rgb_front);
        exit(0);
    }
}