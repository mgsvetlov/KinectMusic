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
#include "blobs/blobext.hpp"

class ProcessFrameData {
public:
    ProcessFrameData(cv::Mat mat, int frameNum);

private:
    void filterFar();
    void resize();
    void createBlobsAndBorders();
    void tracking();
    void visualize();
   
private:
     static pthread_mutex_t visualisation_mutex;
    
    cv::Mat mat;
    int frameNum;
    cv::Mat matFilt;
    cv::Mat matResized;
    std::list<BlobFinal> blobsExt;
    
    friend class Visualization;
};


#endif /* processframedata_h */
