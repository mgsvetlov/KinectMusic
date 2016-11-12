//
//  handsfrompoints.hpp
//  KinectMusic
//
//  Created by Mikhail Svetlov on 12/11/16.
//  Copyright © 2016 mgsvetlov. All rights reserved.
//

#ifndef handsfrompoints_h
#define handsfrompoints_h

#include <stdio.h>
#include "../types.h"
#include "../blobs/blob.h"


class HandsFromPoints{
public:
    HandsFromPoints(cv::Mat mat, const std::list<Blob>& lBlobs, int bbXY, int bbZ);
    std::list<Blob> extractHandBlobs();
private:
    cv::Mat mat;
    const std::list<Blob>& lBlobs;
    int bbXY, bbZ;
};

#endif /* handsfrompoints_h */
