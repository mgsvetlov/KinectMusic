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

pthread_mutex_t depth_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t video_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t visualisation_mutex = PTHREAD_MUTEX_INITIALIZER;
volatile int die = 0;

volatile bool newFrame = false;

int w = 640, h = 480;
uint16_t * const depthAnalyze = new uint16_t[w*h];

int MAX_KINECT_VALUE  = 2047;
int BLOBS_RESIZE_POW  = 2;
int BLOB_MIN_SIZE = 25;
int BLOB_MIN_SIZE_LAST = 3000;
int MAX_NEIGHB_DIFF_COARSE  = 4;
int MAX_NEIGHB_DIFF_FINE  = 1;

volatile int frameNum = 0;
volatile int frameNum_analyze = 0;
volatile int analyzeThreadId = 0;

void *analyze_threadfunc(void *arg) {
    //int threadId = analyzeThreadId++;
    int frameNum_analyze_local (0);
    //int frameCount_analyze_local (0);
    while (!die){
        pthread_mutex_lock(&depth_mutex);
        if(frameNum_analyze == frameNum) {
            pthread_mutex_unlock(&depth_mutex);
            usleep(100);
            continue;
        }
        frameNum_analyze_local = frameNum_analyze = frameNum;
        cv::Mat mat16(h, w, CV_16U, depthAnalyze);
        pthread_mutex_unlock(&depth_mutex);
        
        cv::Mat mat16_resized;
        cv::resize(mat16, mat16_resized, cv::Size(w>>BLOBS_RESIZE_POW, h>>BLOBS_RESIZE_POW));
        
        std::list<Blob> lBlobs;
        Blob::findBlobs(mat16_resized, lBlobs);
        
        //cv::Mat imgResized = Visualization::blobs2img_mark(lBlobs, mat16_resized.size());
        
        cv::Mat mat = Blob::blobs2mat(lBlobs, mat16_resized.size());
        
        cv::Mat matDst = cv::Mat_<uint16_t>::zeros(mat.size());
        uint16_t* p_mat = (uint16_t*)(mat.data);
        uint16_t* p_matDst = (uint16_t*)(matDst.data);
        
        for(int y = 0; y < mat.rows; y++){
            uint16_t* p_mat_row = p_mat;
            for(int x = 0; x < mat.cols; ++x, ++p_mat, ++p_matDst){
                uint16_t val = *p_mat;
                if(!val)
                    continue;
                int filt_size = 10;//val*1e-2;
                int filt_depth = 60;
                int min_x = x - filt_size;
                if(min_x < 0)
                    min_x = 0;
                int max_x = x + filt_size;
                if(max_x >= mat.cols)
                    max_x = mat.cols - 1;
                
                uint16_t val_min = *(p_mat_row + min_x);
                uint16_t val_max = *(p_mat_row + max_x);
                if((!val_min ||val_min - val > filt_depth) && (!val_max ||val_max - val > filt_depth)){
                   *p_matDst = val;
                    continue;
                }
                
                int min_y = y - filt_size;
                if(min_y < 0)
                    min_y = 0;
                int max_y = y + filt_size;
                if(max_y >= mat.rows)
                    max_y = mat.rows - 1;
                
               val_min = *(p_mat + (min_y - y) * mat.cols);
               val_max = *(p_mat + (max_y - y) * mat.cols);
                if((!val_min ||val_min - val > filt_depth) && (!val_max ||val_max - val > filt_depth)){
                    *p_matDst = val;
                }
            }
        }
        
        std::list<Blob> lBlobs1;
        Blob::findBlobs(matDst, lBlobs1, 1);
        cv::Mat imgResized = Visualization::blobs2img_mark(lBlobs1, matDst.size());
        
        //cv::Mat imgResized = Visualization::mat2img(matDst);
        
        pthread_mutex_lock(&visualisation_mutex);
        Visualization::setMatImage(imgResized);
        Visualization::setIsNeedRedraw(true);
        pthread_mutex_unlock(&visualisation_mutex);
        
        /*BlobsMask blobsMask (lBlobs, mat16_resized.size(), mat16.size());
         cv::Mat mat_result = blobsMask.applyMask(mat16);
         Visualization::visualize(mat_result);*/
        
        //std::cout << "id " << threadId << " frameNum_analyze "<< frameNum_analyze_local << " thread count " << ++frameCount_analyze_local << std::endl;
    }
    return NULL;
}
