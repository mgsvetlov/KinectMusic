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

struct HandData {
    cv::Point3i keyPoint;
    cv::Vec3f normal;
    HandData() :
    keyPoint (cv::Point3i(NO_DATA_VALUE, NO_DATA_VALUE, NO_DATA_VALUE)),
    normal (cv::Vec3f(NO_DATA_VALUE, NO_DATA_VALUE, NO_DATA_VALUE))
    {}
    HandData(const cv::Point3i& point, const cv::Vec3f& normal) : keyPoint (point), normal(normal){}
   
};

struct FrameData {
    int frameNum;
    cv::Point3i averagedBodyPoint;
    std::vector<HandData> data;
    FrameData(int frameNum) : frameNum(frameNum), averagedBodyPoint(cv::Point3i(NO_DATA_VALUE)) {}
};

class ProcessFrameData {
public:
    ProcessFrameData(cv::Mat mat, int frameNum);

private:
    void filterFar();
    void resize();
    void createBlobsAndBorders();
    void tracking();
    void shareFrameData();
    void visualize();
   
private:
     static pthread_mutex_t visualisation_mutex;
    
    cv::Mat mat;
    FrameData frameData;
    cv::Mat matFilt;
    cv::Mat matResized;
    std::list<BlobFinal> blobsExt;
    
    friend class Visualization;
};


#endif /* processframedata_h */
