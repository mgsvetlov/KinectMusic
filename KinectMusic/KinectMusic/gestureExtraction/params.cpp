//
//  params.cpp
//  KinectMusic
//
//  Created by Mikhail Svetlov on 23/07/17.
//  Copyright © 2017 mgsvetlov. All rights reserved.
//

#include "params.h"
#include "../config/config.h"

int Params::MATRIX_WIDTH = 640;
int Params::MATRIX_HEIGHT = 480;
int Params::MAX_KINECT_DEPTH = 2000;
int Params::MIN_KINECT_DEPTH = 1;
int Params::BLOBS_RESIZE_POW;
int Params::BLOB_MIN_SIZE;
int Params::BLOB_MIN_SIZE_LAST;
int Params::BLOB_EXT_MAX_SIZE;
bool Params::isInit;

void Params::Init(){
    MATRIX_WIDTH = Config::instance()->getMatrixWidth();
    BLOBS_RESIZE_POW = MATRIX_WIDTH == 640 ? 3 : 2;
    BLOB_MIN_SIZE = (MATRIX_WIDTH >> BLOBS_RESIZE_POW)  * 0.15625 * 0.5;
    BLOB_MIN_SIZE_LAST = (MATRIX_WIDTH >> BLOBS_RESIZE_POW)  * 18.75;
    BLOB_EXT_MAX_SIZE = MATRIX_WIDTH * 6.4;
    isInit = true;
}