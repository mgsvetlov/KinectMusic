//
//  blobsmask.hpp
//  KinectMusic
//
//  Created by Mikhail Svetlov on 09/10/16.
//  Copyright © 2016 mgsvetlov. All rights reserved.
//

#ifndef blobsmask_h
#define blobsmask_h

#include <stdio.h>
#include "../types.h"
#include "blob.h"

class BlobsMask{
public:
    BlobsMask(const std::list<Blob>& lBlobs, cv::Size size, cv::Size fullSize);
    cv::Mat applyMask(cv::Mat mat);
private:
    const cv::Size size, fullSize;
    const int w, h;
    cv::Mat mask;
};



#endif /* blobsmask_h*/
