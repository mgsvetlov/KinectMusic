//
//  processframedata.hpp
//  KinectMusic
//
//  Created by Mikhail Svetlov on 22/07/17.
//  Copyright © 2017 mgsvetlov. All rights reserved.
//

#ifndef processframedata_h
#define processframedata_h
#include <pthread.h>
#include "types.h"

class ProcessFrameData {
public:
    static void Init(int w);
    ProcessFrameData(cv::Mat mat, int frameNum);
    
    static int getMaxKinectDepth() { return MAX_KINECT_DEPTH;}
    static int getMinKinectDepth() { return MIN_KINECT_DEPTH;}
    static int getBlobsResizePow() { return BLOBS_RESIZE_POW;}
    static int getBlobsMinSize() { return BLOB_MIN_SIZE;}
    static int getBlobsMinSizeLast() { return BLOB_MIN_SIZE_LAST;}
    static int getBlobExtMaxSize() { return BLOB_EXT_MAX_SIZE;}

private:
    void filterFar();
    void resize();
   
private:
    static int MAX_KINECT_DEPTH;
    static int MIN_KINECT_DEPTH;
    static int BLOBS_RESIZE_POW;
    static int BLOB_MIN_SIZE;
    static int BLOB_MIN_SIZE_LAST;
    static int BLOB_EXT_MAX_SIZE;
    
     static pthread_mutex_t visualisation_mutex;
    
    cv::Mat mat;
    cv::Mat matFilt;
    cv::Mat matResized;
    
    friend class Visualization;
};


#endif /* processframedata_h */
