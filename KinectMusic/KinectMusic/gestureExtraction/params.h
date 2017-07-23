//
//  params.hpp
//  KinectMusic
//
//  Created by Mikhail Svetlov on 23/07/17.
//  Copyright © 2017 mgsvetlov. All rights reserved.
//

#ifndef params_h
#define params_h

class Params {
public:
    static void Init();
    static int getMaxKinectValue(){return MAX_KINECT_VALUE;}
    static int getMaxHeighbDiffCoarse(){return MAX_NEIGHB_DIFF_COARSE;}
    static int getMatrixWidth() { return MATRIX_WIDTH;}
    static int getMatrixHeight() { return MATRIX_HEIGHT;}
    static int getMaxKinectDepth() { return MAX_KINECT_DEPTH;}
    static int getMinKinectDepth() { return MIN_KINECT_DEPTH;}
    static int getBlobsResizePow() { return BLOBS_RESIZE_POW;}
    static int getBlobsMinSize() { return BLOB_MIN_SIZE;}
    static int getBlobsMinSizeLast() { return BLOB_MIN_SIZE_LAST;}
    static int getBlobExtMaxSize() { return BLOB_EXT_MAX_SIZE;}
    static int getIsInit() { return isInit;}
private:
    static int MAX_KINECT_VALUE;
    static int MAX_NEIGHB_DIFF_COARSE;
    static int MATRIX_WIDTH;
    static int MATRIX_HEIGHT;
    static int MAX_KINECT_DEPTH;
    static int MIN_KINECT_DEPTH;
    static int BLOBS_RESIZE_POW;
    static int BLOB_MIN_SIZE;
    static int BLOB_MIN_SIZE_LAST;
    static int BLOB_EXT_MAX_SIZE;
    static bool isInit;
};

#endif /* params_h */
