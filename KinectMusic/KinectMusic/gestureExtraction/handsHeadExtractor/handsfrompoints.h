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

class Hand;

class HandsFromPoints{
public:
    static std::list<Hand> extractHandBlobs(cv::Mat mat, const std::list<Blob>& lBlobs, int bbXY, int bbZ);

};

#endif /* handsfrompoints_h */
