//
//  params.hpp
//  KinectMusic
//
//  Created by Mikhail Svetlov on 23/07/17.
//  Copyright © 2017 mgsvetlov. All rights reserved.
//

#ifndef params_h
#define params_h

#include <cstddef>

class Params {
public:
    static void Init();
    static int getMaxKinectValue(){return MAX_KINECT_VALUE;}
    static size_t getMaxNeighbDiffCoarse(){return MAX_NEIGHB_DIFF_COARSE;}
    static int getMatrixWidth() { return MATRIX_WIDTH;}
    static int getMatrixHeight() { return MATRIX_HEIGHT;}
    static int getMaxKinectDepth() { return MAX_KINECT_DEPTH;}
    static int getMinKinectDepth() { return MIN_KINECT_DEPTH;}
    static int getBlobResizePow() { return BLOB_RESIZE_POW;}
    static int getBlobMinSize() { return BLOB_MIN_SIZE;}
    static int getBlobMinSizeLast() { return BLOB_MIN_SIZE_LAST;}
    static int getConvex3dFilterSize() { return CONVEX3D_FILTER_SIZE; }
    static int getConvex3dFilterDepth() { return CONVEX3D_FILTER_DEPTH; }
    static int getConvex3dCoreHalfSize() { return CONVEX3D_CORE_HALF_SIZE; }
    static int getConvex3dCountFalsePercent() { return CONVEX3D_COUNT_FALSE_PERCENT; }
    static int getBlobClustXYThresh() {return BLOB_CLUST_XY_THRESH; }
     static int getBlobClustDepthThresh() {return BLOB_CLUST_XY_THRESH; }
    static int getBlobClustMinSize() {return BLOB_CLUST_MIN_SIZE; }
    static int getBlobConnectivityXYThresh1() {return BLOB_CONNECTIVITY_XY_THRESH1; }
    static int getBlobConnectivityXYThresh2() {return BLOB_CONNECTIVITY_XY_THRESH2; }
    static int getBlobConnectivityDepthThresh() {return BLOB_CONNECTIVITY_DEPTH_THRESH; }
    static int GET_BLOB_EXT_RESIZE_POW() { return BLOB_EXT_RESIZE_POW; }
    static int getBlobExtDepthCoeff() { return BLOB_EXT_DEPTH_COEFF;}
    static int getBlobExtMaxSize() { return BLOB_EXT_MAX_SIZE;}
    static int getBlobExtMaxDepthThresh() { return BLOB_EXT_MAX_DEPTH_THRESH;}
    static int getBlobExtFrontCellsCount() { return BLOB_EXT_FRONT_CELLS_COUNT;}
    static int GET_BLOB_EXT_CONVEX3D_FILTER_SIZE_COARSE() { return BLOB_EXT_CONVEX3D_FILTER_SIZE_COARSE; }
    static int GET_BLOB_EXT_CONVEX3D_FILTER_DEPTH_COARSE() { return BLOB_EXT_CONVEX3D_FILTER_DEPTH_COARSE; }
    static int GET_BLOB_EXT_CONVEX3D_CORE_HALF_SIZE_COARSE() { return BLOB_EXT_CONVEX3D_CORE_HALF_SIZE_COARSE; }
    static int GET_BLOB_EXT_CONVEX3D_COUNT_FALSE_PERCENT_COARSE() { return BLOB_EXT_CONVEX3D_COUNT_FALSE_PERCENT_COARSE; }
    static int GET_BLOB_EXT_FEATURE_INDS_COARSE_MIN_SIZE() {
        return BLOB_EXT_FEATURE_INDS_COARSE_MIN_SIZE;
    };
    static int GET_BLOB_EXT_CONVEX3D_FILTER_SIZE_FINE() { return BLOB_EXT_CONVEX3D_FILTER_SIZE_FINE; }
    static int GET_BLOB_EXT_CONVEX3D_FILTER_DEPTH_FINE() { return BLOB_EXT_CONVEX3D_FILTER_DEPTH_FINE; }
    static int GET_BLOB_EXT_CONVEX3D_CORE_HALF_SIZE_FINE() { return BLOB_EXT_CONVEX3D_CORE_HALF_SIZE_FINE; }
    static int GET_BLOB_EXT_CONVEX3D_COUNT_FALSE_PERCENT_FINE() { return BLOB_EXT_CONVEX3D_COUNT_FALSE_PERCENT_FINE; }
    static int GET_BLOB_FINGER_MAX_SIZE() { return BLOB_FINGER_MAX_SIZE;}
    static int getIsInit() { return isInit;}
    static size_t getTracksCount() { return TRACKS_COUNT;}
    static size_t getTrackingDistThresh() { return TRACKING_DIST_THRESH;}
private:
    static int MAX_KINECT_VALUE;
    static size_t MAX_NEIGHB_DIFF_COARSE;
    static int MATRIX_WIDTH;
    static int MATRIX_HEIGHT;
    static int MAX_KINECT_DEPTH;
    static int MIN_KINECT_DEPTH;
    static int BLOB_RESIZE_POW;
    static int BLOB_MIN_SIZE;
    static int BLOB_MIN_SIZE_LAST;
    static int CONVEX3D_FILTER_SIZE;
    static int CONVEX3D_FILTER_DEPTH;
    static int CONVEX3D_CORE_HALF_SIZE;
    static int CONVEX3D_COUNT_FALSE_PERCENT;
    static int BLOB_CLUST_XY_THRESH;
    static int BLOB_CLUST_DEPTH_THRESH;
    static int BLOB_CLUST_MIN_SIZE;
    static int BLOB_CONNECTIVITY_XY_THRESH1;
    static int BLOB_CONNECTIVITY_XY_THRESH2;
    static int BLOB_CONNECTIVITY_DEPTH_THRESH;
    static int BLOB_EXT_RESIZE_POW;
    static int BLOB_EXT_DEPTH_COEFF;
    static int BLOB_EXT_MAX_SIZE;
    static int BLOB_EXT_MAX_DEPTH_THRESH;
    static int BLOB_EXT_FRONT_CELLS_COUNT;
    static int BLOB_EXT_CONVEX3D_FILTER_SIZE_COARSE;
    static int BLOB_EXT_CONVEX3D_FILTER_DEPTH_COARSE;
    static int BLOB_EXT_CONVEX3D_CORE_HALF_SIZE_COARSE;
    static int BLOB_EXT_CONVEX3D_COUNT_FALSE_PERCENT_COARSE;
    static int BLOB_EXT_FEATURE_INDS_COARSE_MIN_SIZE;
    static int BLOB_EXT_CONVEX3D_FILTER_SIZE_FINE;
    static int BLOB_EXT_CONVEX3D_FILTER_DEPTH_FINE;
    static int BLOB_EXT_CONVEX3D_CORE_HALF_SIZE_FINE;
    static int BLOB_EXT_CONVEX3D_COUNT_FALSE_PERCENT_FINE;
    static int BLOB_FINGER_MAX_SIZE;
    static size_t TRACKS_COUNT;
    static size_t TRACKING_DIST_THRESH;
    static bool isInit;
};

#endif /* params_h */
