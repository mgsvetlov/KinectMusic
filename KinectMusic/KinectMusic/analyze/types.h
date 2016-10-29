//
//  types.h
//  KinectMusic
//
//  Created by Mikhail Svetlov on 01/10/16.
//  Copyright © 2016 mgsvetlov. All rights reserved.
//

#ifndef types_h
#define types_h

extern int MAX_KINECT_VALUE;
extern int MAX_KINECT_DEPTH;
extern int BLOBS_RESIZE_POW;
extern int BLOB_MIN_SIZE;
extern int BLOB_MIN_SIZE_LAST;
extern int MAX_NEIGHB_DIFF_COARSE;
extern int MAX_NEIGHB_DIFF_FINE;

extern volatile bool CSOUND_START;

#include "opencv/cv.h"
#include "opencv/highgui.h"
#include "opencv2/opencv.hpp"

#endif /* types_h */
