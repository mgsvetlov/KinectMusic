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
int Params::BLOB_RESIZE_POW;
int Params::BLOB_MIN_SIZE;
int Params::BLOB_MIN_SIZE_LAST;
int Params::CONVEX3D_FILTER_SIZE;
int Params::CONVEX3D_FILTER_DEPTH;
int Params::CONVEX3D_CORE_HALF_SIZE;
int Params::CONVEX3D_COUNT_FALSE_PERCENT;
int Params::BLOB_CLUST_XY_THRESH;
int Params::BLOB_CLUST_DEPTH_THRESH;
int Params::BLOB_CLUST_MIN_SIZE;
int Params::BLOB_CONNECTIVITY_XY_THRESH1;
int Params::BLOB_CONNECTIVITY_XY_THRESH2;
int Params::BLOB_CONNECTIVITY_DEPTH_THRESH;
int Params::BLOB_EXT_MAX_SIZE;
int Params::BLOB_EXT_MAX_DEPTH_RANGE;
int Params::BLOB_EXT_DEPTH_COEFF;
int Params::BLOB_EXT_MAX_DEPTH_THRESH;
int Params::BLOB_EXT_DIST_TO_ADJACENT_BORDER_THRESH;
int Params::BLOB_EXT_FRONT_CELLS_COUNT;

bool Params::isInit;
size_t Params::TRACKS_COUNT = 2;
size_t Params::TRACKING_DIST_THRESH;

void Params::Init(){
    //0 FREENECT_DEPTH_REGISTERED MAX_KINECT_VALUE 10000 MAX_NEIGHB_DIFF_COARSE 80,
    //1 FREENECT_DEPTH_11BIT     MAX_KINECT_VALUE  2018 MAX_NEIGHB_DIFF_COARSE 4
    int depthFormatIndex = Config::instance()->getDepthFormat();
    MAX_KINECT_VALUE = depthFormatIndex == 0 ? 10000 : 2018;
    MAX_NEIGHB_DIFF_COARSE = depthFormatIndex == 0 ? 80 : 6;//80 : 4
    MAX_KINECT_DEPTH = depthFormatIndex == 0 ? 2500 : 900;
    MATRIX_WIDTH = Config::instance()->getMatrixWidth();
    BLOB_RESIZE_POW = MATRIX_WIDTH == 640 ? 3 : 2;
    BLOB_MIN_SIZE = (MATRIX_WIDTH >> BLOB_RESIZE_POW)  * 0.15625 * 0.5;
    BLOB_MIN_SIZE_LAST = (MATRIX_WIDTH >> BLOB_RESIZE_POW)  * 4;
    CONVEX3D_FILTER_SIZE = (MATRIX_WIDTH >> BLOB_RESIZE_POW) * 0.06;
    CONVEX3D_FILTER_DEPTH = depthFormatIndex == 0 ? 40 : 6;
    CONVEX3D_CORE_HALF_SIZE = 1;
    CONVEX3D_COUNT_FALSE_PERCENT = 25;
    BLOB_CLUST_XY_THRESH = (MATRIX_WIDTH >> BLOB_RESIZE_POW) * 0.125;
    BLOB_CLUST_DEPTH_THRESH = depthFormatIndex == 0 ? 100 : 40;
    BLOB_CLUST_MIN_SIZE = (MATRIX_WIDTH >> BLOB_RESIZE_POW) * 0.1;
    BLOB_EXT_DEPTH_COEFF = depthFormatIndex == 0 ? 1200 : 450;
    BLOB_CONNECTIVITY_XY_THRESH1 = (MATRIX_WIDTH >> BLOB_RESIZE_POW) * 0.1 + 1;
    BLOB_CONNECTIVITY_XY_THRESH2 = (MATRIX_WIDTH >> BLOB_RESIZE_POW) * 0.2;
    BLOB_CONNECTIVITY_DEPTH_THRESH = depthFormatIndex == 0 ? -20 : -8;
    BLOB_EXT_MAX_SIZE = MATRIX_WIDTH * 6;
    BLOB_EXT_MAX_DEPTH_RANGE = depthFormatIndex == 0 ? 200 : 90;
    BLOB_EXT_MAX_DEPTH_THRESH = depthFormatIndex == 0 ? 800 : 300;
    BLOB_EXT_DIST_TO_ADJACENT_BORDER_THRESH = MATRIX_WIDTH * 0.15625;
    BLOB_EXT_FRONT_CELLS_COUNT = MATRIX_WIDTH * 1.5625;
    TRACKING_DIST_THRESH = MATRIX_WIDTH * 0.125;
    isInit = true;
}