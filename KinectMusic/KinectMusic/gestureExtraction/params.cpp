//
//  params.cpp
//  KinectMusic
//
//  Created by Mikhail Svetlov on 23/07/17.
//  Copyright © 2017 mgsvetlov. All rights reserved.
//

#include "params.h"
#include "../config/config.h"

int Params::MAX_KINECT_VALUE;
int Params::MAX_NEIGHB_DIFF_COARSE;
int Params::MATRIX_WIDTH = 640;
int Params::MATRIX_HEIGHT = 480;
int Params::MAX_KINECT_DEPTH;
int Params::MIN_KINECT_DEPTH = 1;
int Params::BLOBS_RESIZE_POW;
int Params::BLOB_MIN_SIZE;
int Params::BLOB_MIN_SIZE_LAST;
int Params::BLOB_EXT_MAX_SIZE;
bool Params::isInit;
size_t Params::TRACKS_COUNT = 2;
size_t Params::TRACKING_DIST_THRESH;

void Params::Init(){
    //0 FREENECT_DEPTH_REGISTERED MAX_KINECT_VALUE 10000 MAX_NEIGHB_DIFF_COARSE 80,
    //1 FREENECT_DEPTH_11BIT     MAX_KINECT_VALUE  2018 MAX_NEIGHB_DIFF_COARSE 4
    int depthFormatIndex = Config::instance()->getDepthFormat();
    MAX_KINECT_VALUE = depthFormatIndex == 0 ? 10000 : 2018;
    MAX_NEIGHB_DIFF_COARSE = depthFormatIndex == 0 ? 80 : 4;
    MAX_KINECT_DEPTH = depthFormatIndex == 0 ? 2000 : 900;
    MATRIX_WIDTH = Config::instance()->getMatrixWidth();
    BLOBS_RESIZE_POW = MATRIX_WIDTH == 640 ? 3 : 2;
    BLOB_MIN_SIZE = (MATRIX_WIDTH >> BLOBS_RESIZE_POW)  * 0.15625 * 0.5;
    BLOB_MIN_SIZE_LAST = (MATRIX_WIDTH >> BLOBS_RESIZE_POW)  * 4;
    BLOB_EXT_MAX_SIZE = MATRIX_WIDTH * 6.4;
    TRACKING_DIST_THRESH = MATRIX_WIDTH * 0.125;
    isInit = true;
}